#include "kart_control.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_steer_ctrl.h"

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

/* 速度环默认 PID 参数 —— 2026-07-17 上车调定并冻结。
 * 量纲是"脉冲/5ms"。串口在线调参最终定为 Kp=200 / Ki=0.8 / Kd=0,
 * 实测 I12≈I13 左右脉冲率拉平、稳态误差可接受,底层速度环到此冻结。
 * (左右后轮机械差异由前轮转向的航向角度环处理,不在速度环加电子差速。)
 * i_max/out_max 的 out_max 给 KART_POWER_MAX_DUTY(满量程),让 PID 能用满输出范围。 */
#define KART_SPEED_KP_DEFAULT          (200.0f)
#define KART_SPEED_KI_DEFAULT          (0.8f)
#define KART_SPEED_KD_DEFAULT          (0.0f)
#define KART_SPEED_IMAX_DEFAULT        (3000.0f)
#define KART_SPEED_OUTMAX_DEFAULT      ((float)KART_POWER_MAX_DUTY)

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
    kart_speed.left_target = 0.0f;
    kart_speed.right_target = 0.0f;
    kart_speed.meas_raw    = 0.0f;
    kart_speed.meas        = 0.0f;
    kart_speed.meas_left   = 0.0f;
    kart_speed.meas_right  = 0.0f;
    kart_speed.output_duty = 0;
    kart_speed.output_left = 0;
    kart_speed.output_right = 0;
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
    }
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
    kart_speed.target = target;

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
