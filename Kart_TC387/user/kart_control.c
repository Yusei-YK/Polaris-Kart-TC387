#include "kart_control.h"
#include "kart_encoder.h"
#include "kart_power.h"

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

/* =========================== 滑动平均滤波 =========================== */
/* 移植自 TopSpeed Speed_encoder 的 10 点均值滤波:
 * 新值写进环形缓冲,返回整个窗口的平均。作用是压掉编码器 delta 的抖动。 */
static float kart_speed_lpf(float new_val)
{
    kart_speed.lpf_buf[kart_speed.lpf_idx] = new_val;
    kart_speed.lpf_idx++;
    if(kart_speed.lpf_idx >= KART_SPEED_LPF_LEN)
    {
        kart_speed.lpf_idx = 0;
    }

    float sum = 0.0f;
    for(int i = 0; i < KART_SPEED_LPF_LEN; i++)
    {
        sum += kart_speed.lpf_buf[i];
    }
    return sum / (float)KART_SPEED_LPF_LEN;
}

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

    /* 5. 误差 = 目标 - 实测,喂 PID */
    kart_pid_update(&kart_speed.pid, kart_speed.target - kart_speed.meas_left);
    kart_pid_update(&kart_speed.pid_right, kart_speed.target - kart_speed.meas_right);

    /* 6. 直接赋值(不累加),下发两后轮 */
    kart_speed.output_left = (int16)kart_speed.pid.output;
    kart_speed.output_right = (int16)kart_speed.pid_right.output;
    kart_speed.output_duty = (int16)(((int32)kart_speed.output_left +
                                      (int32)kart_speed.output_right) / 2);
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

void kart_control_set_target(float target)
{
    kart_speed.target = target;
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
