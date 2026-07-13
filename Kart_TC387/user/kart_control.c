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

/* 速度环默认 PID 参数 —— 悬空调参的起点,不是最终值。
 * 注意:这里的量纲是"脉冲/5ms",和 TopSpeed 的 m/s 不同,所以不能照抄它的
 *      Kp=200/Kd=4000,那套是给 m/s 量纲的。这里先给保守小值,上车用串口慢慢往上调。
 * i_max/out_max 的 out_max 给 KART_POWER_MAX_DUTY(满量程),让 PID 能用满输出范围。 */
#define KART_SPEED_KP_DEFAULT          (10.0f)
#define KART_SPEED_KI_DEFAULT          (0.0f)
#define KART_SPEED_KD_DEFAULT          (0.0f)
#define KART_SPEED_IMAX_DEFAULT        (3000.0f)
#define KART_SPEED_OUTMAX_DEFAULT      ((float)KART_POWER_MAX_DUTY)

static kart_speed_ctrl_t kart_speed = {0};

/* =========================== 初始化 =========================== */
void kart_control_init(void)
{
    /* 整块清零:目标、滤波缓冲、指针、输出全归零 */
    for(int i = 0; i < KART_SPEED_LPF_LEN; i++)
    {
        kart_speed.lpf_buf[i] = 0.0f;
    }
    kart_speed.lpf_idx     = 0;
    kart_speed.target      = 0.0f;
    kart_speed.meas_raw    = 0.0f;
    kart_speed.meas        = 0.0f;
    kart_speed.output_duty = 0;
    kart_speed.enable      = 0;         // 默认不输出,等串口/上层显式使能,防一上电就冲

    kart_pid_init(&kart_speed.pid,
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
    kart_speed.meas = kart_speed_lpf(kart_speed.meas_raw);

    /* 4. 没使能就输出 0、清 PID 记忆,直接返回(悬空/急停态) */
    if(!kart_speed.enable)
    {
        kart_speed.output_duty = 0;
        kart_pid_reset(&kart_speed.pid);
        power_set_rear_duty(0, 0);
        return;
    }

    /* 5. 误差 = 目标 - 实测,喂 PID */
    float err = kart_speed.target - kart_speed.meas;
    kart_pid_update(&kart_speed.pid, err);

    /* 6. 直接赋值(不累加),下发两后轮 */
    kart_speed.output_duty = (int16)kart_speed.pid.output;
    power_set_rear_duty(kart_speed.output_duty, kart_speed.output_duty);
}

/* =========================== 在线设置接口 =========================== */
void kart_control_set_enable(uint8 en)
{
    uint32 primask = interrupt_global_disable();

    kart_speed.enable = en ? 1 : 0;
    if(!kart_speed.enable)
    {
        kart_pid_reset(&kart_speed.pid);    // 关的时候清记忆,下次开不带旧账
        kart_speed.output_duty = 0;
        power_set_rear_duty(0, 0);          // e0/故障关闭后立即撤销上一拍的电机请求
    }

    interrupt_global_enable(primask);
}

void kart_control_set_target(float target)
{
    uint32 primask = interrupt_global_disable();
    kart_speed.target = target;
    interrupt_global_enable(primask);
}

void kart_control_set_pid(float kp, float ki, float kd)
{
    uint32 primask = interrupt_global_disable();

    /* 只改三个系数,限幅沿用初始化时的值;改完清一次记忆防跳变 */
    kart_speed.pid.Kp = kp;
    kart_speed.pid.Ki = ki;
    kart_speed.pid.Kd = kd;
    kart_pid_reset(&kart_speed.pid);

    interrupt_global_enable(primask);
}

/* =========================== 取值接口(给 VOFA/调试)=========================== */
float kart_control_get_target(void)
{
    uint32 primask = interrupt_global_disable();
    float value = kart_speed.target;
    interrupt_global_enable(primask);
    return value;
}

float kart_control_get_meas(void)
{
    uint32 primask = interrupt_global_disable();
    float value = kart_speed.meas;
    interrupt_global_enable(primask);
    return value;
}

int16 kart_control_get_output(void)
{
    uint32 primask = interrupt_global_disable();
    int16 value = kart_speed.output_duty;
    interrupt_global_enable(primask);
    return value;
}

uint8 kart_control_is_enabled(void)
{
    uint32 primask = interrupt_global_disable();
    uint8 value = kart_speed.enable;
    interrupt_global_enable(primask);
    return value;
}

void kart_control_get_pid(float *kp, float *ki, float *kd)
{
    uint32 primask = interrupt_global_disable();

    if(kp != NULL) { *kp = kart_speed.pid.Kp; }
    if(ki != NULL) { *ki = kart_speed.pid.Ki; }
    if(kd != NULL) { *kd = kart_speed.pid.Kd; }

    interrupt_global_enable(primask);
}
