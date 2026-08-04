#include "kart_control.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_steer_ctrl.h"
#include <math.h>

/*
 * 卡丁车速度环 —— 实现
 * ------------------------------------------------------------------
 * 移植自 TopSpeed 速度环。数据流(一拍 5ms):
 *   编码器 delta → 左右取平均 = 实测速度(脉冲/5ms)
 *   → 10 点滑动平均滤波
 *   → 目标 - 实测 = 误差 → 位置式 PID
 *   → 直接赋值 duty → power_set_rear_duty() 下发两后轮
 * ------------------------------------------------------------------
 */

/* 速度环默认 PID 参数已移到 kart_control.h(kart_params 表要引用 KP/IMAX 当出厂值)。 */

kart_speed_ctrl_t kart_speed = {0};

/* =========================== 初始化 =========================== */
void kart_control_init(void)
{
    /* 整块清零:目标、滤波缓冲、指针、输出全归零 */
    for(int i = 0; i < KART_SPEED_LPF_LEN; i++)
    {
        kart_speed.lpf_buf[i] = 0.0f;
        kart_speed.lpf_right[i] = 0.0f;
    }
    kart_speed.lpf_idx     = 0;
    kart_speed.target      = 0.0f;
    kart_speed.target_cmd  = 0.0f;
    kart_speed.ramp_step   = KART_SPEED_RAMP_STEP_DEFAULT;
    kart_speed.left_target = 0.0f;
    kart_speed.right_target = 0.0f;
    kart_speed.meas_raw    = 0.0f;
    kart_speed.meas        = 0.0f;
    kart_speed.meas_left   = 0.0f;
    kart_speed.meas_right  = 0.0f;
    kart_speed.output_duty = 0;
    kart_speed.output_left = 0;
    kart_speed.output_right = 0;
    kart_speed.open_loop   = 0;         // 默认闭环支路
    kart_speed.open_duty   = 0;
    kart_speed.enable      = 0;         // 默认不输出,等串口/上层显式使能,防一上电就冲

    kart_pid_init(&kart_speed.pid,
                  KART_SPEED_KP_DEFAULT,
                  KART_SPEED_KI_DEFAULT,
                  KART_SPEED_KD_DEFAULT,
                  KART_SPEED_IMAX_DEFAULT,
                  KART_SPEED_OUTMAX_DEFAULT);
    kart_pid_init(&kart_speed.pid_right,
                  KART_SPEED_KP_DEFAULT,
                  KART_SPEED_KI_DEFAULT,
                  KART_SPEED_KD_DEFAULT,
                  KART_SPEED_IMAX_DEFAULT,
                  KART_SPEED_OUTMAX_DEFAULT);
}

/* 说明:10 点滑动平均滤波已内联进 kart_control_speed_update()(左右轮独立缓冲
 * lpf_buf/lpf_right),此处不再保留独立 kart_speed_lpf() 函数,避免死代码警告。 */

static float kart_control_get_steer_norm(void);

/* =========================== 速度环一拍(放 5ms 中断)=========================== */
void kart_control_speed_update(void)
{
    /* 1. 更新编码器(唯一数据源:整个工程只有这里调 update,清零权归它,
     *    避免和调试输出互抢 delta) */
    kart_encoder_update();

    /* 2. 实测速度 = 左右后轮 delta 的平均(脉冲/5ms) */
    int16 l = kart_encoder_get_left_delta();
    int16 r = kart_encoder_get_right_delta();
    kart_speed.meas_raw = (float)(l + r) / 2.0f;

    /* 3. 滑动平均滤波 */
    kart_speed.lpf_buf[kart_speed.lpf_idx] = (float)l;
    kart_speed.lpf_right[kart_speed.lpf_idx] = (float)r;
    kart_speed.lpf_idx++;
    if(kart_speed.lpf_idx >= KART_SPEED_LPF_LEN) { kart_speed.lpf_idx = 0; }

    float left_sum = 0.0f, right_sum = 0.0f;
    for(int i = 0; i < KART_SPEED_LPF_LEN; i++)
    {
        left_sum += kart_speed.lpf_buf[i];
        right_sum += kart_speed.lpf_right[i];
    }
    kart_speed.meas_left = left_sum / (float)KART_SPEED_LPF_LEN;
    kart_speed.meas_right = right_sum / (float)KART_SPEED_LPF_LEN;
    kart_speed.meas = (kart_speed.meas_left + kart_speed.meas_right) * 0.5f;

    /* 4. 没使能就输出 0、清 PID 记忆,直接返回(悬空/急停态) */
    if(!kart_speed.enable)
    {
        kart_speed.output_duty = 0;
        kart_speed.output_left = 0;
        kart_speed.output_right = 0;
        kart_pid_reset(&kart_speed.pid);
        kart_pid_reset(&kart_speed.pid_right);
        power_set_rear_duty(0, 0);
        return;
    }

    /* 4.5 开环支路(科目二固定动作):跳过 PID,固定 duty 直下发。
     *   完成判据在上层是路程/累计 yaw,不是时间,所以车速准不准只影响"跑多久",
     *   不影响"跑多远/转多少度" —— 电量导致的车速漂移被路程判据吸收。
     *   PID 记忆每拍清,保证退出开环切回闭环时不带旧积分。
     *   实测速度 meas 上面已算完,VOFA 波形照常有数据。 */
    if(kart_speed.open_loop)
    {
        kart_pid_reset(&kart_speed.pid);
        kart_pid_reset(&kart_speed.pid_right);
        kart_speed.output_duty  = kart_speed.open_duty;
        kart_speed.output_left  = kart_speed.open_duty;
        kart_speed.output_right = kart_speed.open_duty;
        power_set_rear_duty(kart_speed.output_left, kart_speed.output_right);
        return;
    }

    /* 4.8 目标速度斜坡(治起步/加速顿挫,见 kart_control.h 病因链注释)。
     * 只限"幅值增大"方向:减速/停车/反向瞬时跟随(安全取向,同 power_sync 的 slew)。
     * 放在开环支路之后:开环 duty 不经速度环 PID,科目二固定动作行为完全不变。
     * 放在 5ms 中断里逐拍爬(不放 set_target):上层是 10ms 拍调的,
     * 放上层会让斜坡分辨率降一半,且 playback 每拍改目标时爬不动。 */
    if(kart_speed.ramp_step > 0.0f)
    {
        float cmd  = kart_speed.target_cmd;
        float cur  = kart_speed.target;
        float step = kart_speed.ramp_step;

        /* 异号(含往零方向穿越)或幅值减小 → 直接跟随,不限速。 */
        if((cmd * cur) < 0.0f || fabsf(cmd) <= fabsf(cur))
        {
            kart_speed.target = cmd;
        }
        else if(cmd > cur)                  /* 同向正、幅值增大 */
        {
            kart_speed.target = ((cmd - cur) > step) ? (cur + step) : cmd;
        }
        else                                /* 同向负、幅值增大(倒车加速) */
        {
            kart_speed.target = ((cur - cmd) > step) ? (cur - step) : cmd;
        }
    }
    else
    {
        kart_speed.target = kart_speed.target_cmd;   /* 斜坡关闭:恢复旧阶跃行为 */
    }

    /* 5. 单速度环:误差 = 中心目标 - 左右平均实测,只喂一个 PID。
     *    两后轮共轴松耦合,双独立闭环 PID 会互顶发散(悬空撞 ±10000)。
     *    改成唯一积分器控平均速度,得到基准 duty base。 */
    kart_pid_update(&kart_speed.pid, kart_speed.target - kart_speed.meas);
    int16 base = (int16)kart_speed.pid.output;

    /* 6. 电子差速改为前馈 duty 偏置(开环,不再是第二个闭环):
     *    左 = base×(1-r)、右 = base×(1+r),r 由转角算,有界 ±MAX_RATIO。
     *    两轮 duty 同号、偏置有限,物理上不会互顶反转;差速效果靠 torque 偏置
     *    + 机械松耦合自然跑出内外轮速差。 */
    kart_speed.output_duty = base;
#if KART_EDIFF_ENABLE
    {
        float r = KART_EDIFF_GAIN * kart_control_get_steer_norm();
        if(r >  KART_EDIFF_MAX_RATIO) r =  KART_EDIFF_MAX_RATIO;
        if(r < -KART_EDIFF_MAX_RATIO) r = -KART_EDIFF_MAX_RATIO;
        kart_speed.output_left  = (int16)((float)base * (1.0f - r));
        kart_speed.output_right = (int16)((float)base * (1.0f + r));
    }
#else
    kart_speed.output_left  = base;
    kart_speed.output_right = base;
#endif
    power_set_rear_duty(kart_speed.output_left, kart_speed.output_right);
}

/* =========================== 在线设置接口 =========================== */
void kart_control_set_enable(uint8 en)
{
    kart_speed.enable = en ? 1 : 0;
    if(!kart_speed.enable)
    {
        kart_pid_reset(&kart_speed.pid);    // 关的时候清记忆,下次开不带旧账
        kart_pid_reset(&kart_speed.pid_right);
        /* 一并退出开环支路:失能是所有急停路径(遥控挡位/失联/deadman/切模式)的
         * 汇聚点,在这里清掉,开环状态就绝不会漏到科目一/四或遥控模式里去。 */
        kart_speed.open_loop = 0;
        kart_speed.open_duty = 0;
        /* 斜坡状态一并归零:失能=车已停,下次使能必须从 0 重新爬。
         * 不清的话 target 会留着上次的大值,再使能时第一拍就是满误差 → 又是阶跃。 */
        kart_speed.target     = 0.0f;
        kart_speed.target_cmd = 0.0f;
    }
}

/* 开环 duty:先写值再置标志。
 * 顺序有意义 —— 本函数在主循环上下文调,speed_update 在 5ms 中断里读;
 * 若先置 open_loop 再写 open_duty,中间被中断打断就会用上一次的旧 duty 跑一拍。
 * int16 写入在 TriCore 上是单指令原子的,不需要额外临界区。 */
void kart_control_set_open_duty(int16 duty)
{
    kart_speed.open_duty = duty;
    kart_speed.open_loop = 1;
}

void kart_control_clear_open_loop(void)
{
    kart_speed.open_loop = 0;
    kart_speed.open_duty = 0;
    kart_pid_reset(&kart_speed.pid);
    kart_pid_reset(&kart_speed.pid_right);
}

/* 实测转角归一化到 [-1,+1]:用绝对编码器 center_delta 除以对应方向软限位。
 * center_delta:左转为正(软限 +KART_STEER_DELTA_LIMIT_L)、右转为负(软限 KART_STEER_DELTA_LIMIT_R<0)。
 * 用实测(非目标):转向机构有延迟/旷量/回中误差,后轮要配合"现在实际转到哪",不是"想转到哪"。 */
static float kart_control_get_steer_norm(void)
{
    float delta = (float)kart_steer_abs_get_center_delta();
    float norm;

    if(delta >= 0.0f)
    {
        norm = delta / (float)KART_STEER_DELTA_LIMIT_L;         // 左转:除左软限(正)
    }
    else
    {
        norm = delta / (float)(-KART_STEER_DELTA_LIMIT_R);      // 右转:除右软限绝对值(R 为负,取反)
    }

    if(norm > 1.0f)  norm = 1.0f;
    if(norm < -1.0f) norm = -1.0f;
    return norm;
}

/* 设中心目标速度,并按实测转角分配左右轮目标(电子差速)。
 * steer_norm>0(左转):左轮内侧应慢(1-r)、右轮外侧应快(1+r)。
 * 倒车(target<0)第一版关闭差速(方向符号未验证),左右直接同目标。 */
void kart_control_set_target(float target)
{
    /* 写"请求目标";实际喂 PID 的 kart_speed.target 由 5ms 中断按 ramp_step 逐拍靠拢。
     * 幅值减小/反向时中断那边直接跟随,故急停(set_target(0))依旧当拍生效。 */
    kart_speed.target_cmd = target;

#if KART_EDIFF_ENABLE
    if(target < 0.0f)
    {
        kart_speed.left_target  = target;      // 倒车:关差速,左右同目标
        kart_speed.right_target = target;
        return;
    }

    {
        float steer_norm = kart_control_get_steer_norm();
        float diff_ratio = KART_EDIFF_GAIN * steer_norm;

        if(diff_ratio >  KART_EDIFF_MAX_RATIO) diff_ratio =  KART_EDIFF_MAX_RATIO;
        if(diff_ratio < -KART_EDIFF_MAX_RATIO) diff_ratio = -KART_EDIFF_MAX_RATIO;

        kart_speed.left_target  = target * (1.0f - diff_ratio);
        kart_speed.right_target = target * (1.0f + diff_ratio);
    }
#else
    kart_speed.left_target  = target;
    kart_speed.right_target = target;
#endif
}

/* 斜坡步长在线设置(菜单/串口)。step<=0 视为关闭斜坡,退回旧的阶跃行为。
 * 只写一个 float,5ms 中断读它,单字写入原子,不需要临界区。 */
void kart_control_set_ramp_step(float step)
{
    if(step < 0.0f) step = 0.0f;
    kart_speed.ramp_step = step;
}

float kart_control_get_ramp_step(void) { return kart_speed.ramp_step; }
float kart_control_get_target_cmd(void) { return kart_speed.target_cmd; }

/* 速度环积分限幅在线调(菜单 Spd Imax)。
 * 为什么单独给接口:i_max 是提速的第一道墙 —— 稳态 duty = Kp*e + i_max,
 * 实测 200*19.4+3000 = 6876,10000 的量程有 27% 永远拿不到。
 * 【不清 PID 记忆】故意的:行驶中调它必须无级平滑,reset 会让积分掉到 0 → 掉速一拍。 */
void kart_control_set_speed_imax(float imax)
{
    kart_speed.pid.i_max       = imax;
    kart_speed.pid_right.i_max = imax;
}

/* 速度环 Kp 在线调(菜单 Spd Kp)。同样不清记忆,理由同上。 */
void kart_control_set_speed_kp(float kp)
{
    kart_speed.pid.Kp       = kp;
    kart_speed.pid_right.Kp = kp;
}

void kart_control_set_pid(float kp, float ki, float kd)
{
    /* 只改三个系数,限幅沿用初始化时的值;改完清一次记忆防跳变 */
    kart_speed.pid.Kp = kp;
    kart_speed.pid.Ki = ki;
    kart_speed.pid.Kd = kd;
    kart_speed.pid_right.Kp = kp;
    kart_speed.pid_right.Ki = ki;
    kart_speed.pid_right.Kd = kd;
    kart_pid_reset(&kart_speed.pid);
    kart_pid_reset(&kart_speed.pid_right);
}

/* =========================== 取值接口(给 VOFA/调试)=========================== */
float kart_control_get_target(void) { return kart_speed.target; }
float kart_control_get_meas(void)   { return kart_speed.meas; }
int16 kart_control_get_output(void) { return kart_speed.output_duty; }
float kart_control_get_left_meas(void) { return kart_speed.meas_left; }
float kart_control_get_right_meas(void) { return kart_speed.meas_right; }
int16 kart_control_get_left_output(void) { return kart_speed.output_left; }
int16 kart_control_get_right_output(void) { return kart_speed.output_right; }
uint8 kart_control_is_enabled(void) { return kart_speed.enable; }
