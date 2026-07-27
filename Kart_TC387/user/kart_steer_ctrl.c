#include "kart_steer_ctrl.h"
#include "kart_steer_abs.h"
#include "kart_imu.h"
#include "kart_power.h"

/*
 * 转向串级控制实现 —— 详见 kart_steer_ctrl.h 顶部架构说明。
 * 数据流(主循环一拍,紧跟 kart_steer_abs_update 之后):
 *   [外环开] 目标航向 − IMU yaw(wrap±180)→ head_pid → 目标转角(限幅)
 *   [内环]   目标转角 − center_delta → angle_pid → 转向电机 duty
 */

kart_steer_ctrl_t kart_steer = {0};

/* 航向误差归一到 −180~180,保证走最近的方向纠偏(别绕远路)。 */
static float kart_steer_wrap180(float err)
{
    while(err > 180.0f)  { err -= 360.0f; }
    while(err < -180.0f) { err += 360.0f; }
    return err;
}

/* 目标转角软限幅:夹在左右打死软限位内,别顶机械硬限位。 */
static float kart_steer_clamp_delta(float delta)
{
    if(delta > (float)KART_STEER_DELTA_LIMIT_L) { return (float)KART_STEER_DELTA_LIMIT_L; }
    if(delta < (float)KART_STEER_DELTA_LIMIT_R) { return (float)KART_STEER_DELTA_LIMIT_R; }
    return delta;
}

void kart_steer_ctrl_init(void)
{
    kart_steer.angle_enable = 0;
    kart_steer.head_enable  = 0;
    kart_steer.target_yaw   = 0.0f;
    kart_steer.meas_yaw     = 0.0f;
    kart_steer.target_delta = 0.0f;
    kart_steer.meas_delta   = 0.0f;
    kart_steer.output_duty  = 0;

    kart_pid_init(&kart_steer.angle_pid,
                  KART_STEER_KP_DEFAULT,
                  KART_STEER_KI_DEFAULT,
                  KART_STEER_KD_DEFAULT,
                  KART_STEER_IMAX_DEFAULT,
                  KART_STEER_OUTMAX_DEFAULT);
    kart_pid_init(&kart_steer.head_pid,
                  KART_HEAD_KP_DEFAULT,
                  KART_HEAD_KI_DEFAULT,
                  KART_HEAD_KD_DEFAULT,
                  KART_HEAD_IMAX_DEFAULT,
                  KART_HEAD_OUTMAX_DEFAULT);
}

void kart_steer_ctrl_update(void)
{
    /* 内环没使能:输出 0、清两级 PID 记忆,直接返回(悬空/急停态) */
    if(!kart_steer.angle_enable)
    {
        kart_steer.output_duty = 0;
        kart_pid_reset(&kart_steer.angle_pid);
        kart_pid_reset(&kart_steer.head_pid);
        power_set_steer_duty(0);
        return;
    }

    /* 1. 外环(航向):开了才算,输出覆写 target_delta;没开则沿用串口直给的 target_delta */
    if(kart_steer.head_enable)
    {
        float yaw_err;
        kart_steer.meas_yaw = kart_imu_get_yaw();
        yaw_err = kart_steer_wrap180(kart_steer.target_yaw - kart_steer.meas_yaw);
        kart_pid_update(&kart_steer.head_pid, yaw_err);
        kart_steer.target_delta = kart_steer_clamp_delta(kart_steer.head_pid.output);
    }

    /* 2. 内环(转角):目标转角 − 实测 center_delta → PID → 电机 duty */
    kart_steer.meas_delta = (float)kart_steer_abs_get_center_delta();
    kart_pid_update(&kart_steer.angle_pid,
                    kart_steer.target_delta - kart_steer.meas_delta);

    kart_steer.output_duty = (int16)(kart_steer.angle_pid.output * (float)KART_STEER_LOOP_SIGN);
    power_set_steer_duty(kart_steer.output_duty);
}

/* =========================== 在线设置接口 =========================== */
void kart_steer_set_angle_enable(uint8 en)
{
    if(en && !kart_steer.angle_enable)
    {
        /* 0→1 上升沿:用当前实测转角初始化目标,消除大初始误差防超调。
         * 不在 update() 里做是因为此时 center_delta 已由主循环刷新,读最新值安全。 */
        kart_steer.target_delta = (float)kart_steer_abs_get_center_delta();
        kart_pid_reset(&kart_steer.angle_pid);
    }
    kart_steer.angle_enable = en ? 1 : 0;
    if(!kart_steer.angle_enable)
    {
        kart_steer.head_enable = 0;      // 关内环连带关外环,防外环空转积分
        kart_pid_reset(&kart_steer.angle_pid);
        kart_pid_reset(&kart_steer.head_pid);
    }
}

void kart_steer_set_head_enable(uint8 en)
{
    if(en)
    {
        kart_steer.angle_enable = 1;     // 开外环必须先有内环托底
        kart_steer.target_yaw = kart_imu_get_yaw();  // he1 上升沿:记住当前航向作目标
        kart_steer.head_enable  = 1;
        kart_pid_reset(&kart_steer.head_pid);
    }
    else
    {
        kart_steer.head_enable = 0;
        kart_pid_reset(&kart_steer.head_pid);
    }
}

void kart_steer_set_target_delta(float delta)
{
    kart_steer.target_delta = kart_steer_clamp_delta(delta);
}

void kart_steer_set_target_yaw(float yaw)
{
    kart_steer.target_yaw = yaw;
}

void kart_steer_set_angle_pid(float kp, float ki, float kd)
{
    kart_steer.angle_pid.Kp = kp;
    kart_steer.angle_pid.Ki = ki;
    kart_steer.angle_pid.Kd = kd;
    kart_pid_reset(&kart_steer.angle_pid);
}

void kart_steer_set_head_pid(float kp, float ki, float kd)
{
    kart_steer.head_pid.Kp = kp;
    kart_steer.head_pid.Ki = ki;
    kart_steer.head_pid.Kd = kd;
    kart_pid_reset(&kart_steer.head_pid);
}

/* 内环增益组切换 —— 见 kart_steer_ctrl.h 里 KART_STEER_KP_BACK 的说明。
 * 复用 set_angle_pid(),所以清记忆的行为完全一致,不引入新路径。 */
void kart_steer_use_back_gains(void)
{
    kart_steer_set_angle_pid(KART_STEER_KP_BACK,
                             KART_STEER_KI_BACK,
                             KART_STEER_KD_BACK);
}

void kart_steer_use_fwd_gains(void)
{
    kart_steer_set_angle_pid(KART_STEER_KP_DEFAULT,
                             KART_STEER_KI_DEFAULT,
                             KART_STEER_KD_DEFAULT);
}

/* =========================== 取值接口(给 VOFA/调试)=========================== */
float kart_steer_get_target_delta(void) { return kart_steer.target_delta; }
float kart_steer_get_meas_delta(void)   { return kart_steer.meas_delta; }
int16 kart_steer_get_output(void)       { return kart_steer.output_duty; }
float kart_steer_get_target_yaw(void)   { return kart_steer.target_yaw; }
