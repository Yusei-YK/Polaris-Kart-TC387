#ifndef KART_POWER_H_
#define KART_POWER_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

#define KART_POWER_PWM_FREQ_HZ          (17000)
#define KART_POWER_MAX_DUTY             (PWM_DUTY_MAX)
#define KART_BOOT_MOTOR_DUTY            (0)
#define KART_BOOT_SERVO_DUTY            (0)

/* ---------------- 输出变化率限制(slew-rate limit,防急停急反烧驱动)----------------
 * 场景:高速正转后立刻命令急停反转,电机反电动势+大电流突变,正是烧单驱/驱动板的能量源。
 * 对策(在 power_sync 里对每路输出限速):
 *   1) 禁止穿零跳变:目标与当前输出反号时,先强制到 0(coast 滑行),不允许一拍从大正跳到大负;
 *   2) 零点驻留:到 0 后停 KART_SLEW_ZERO_DWELL_TICKS 拍,让电机靠惯性泄速,再往反向爬;
 *   3) 升幅限速:同向加大 duty 每拍最多加 STEP,杜绝瞬间满载冲击;
 *   4) 降幅/停车不限速:往 0 方向减小是安全的,保持瞬时 —— 急停(→0)依旧即时(coast 安全)。
 * STEP 单位是 duty/拍(主循环 5ms 一拍)。STEP 越小越柔、越不易烧,但控制响应越慢。
 * 关掉(=0)则恢复旧的直通行为,便于对比。*/
#define KART_SLEW_ENABLE                (1)
#define KART_SLEW_REAR_STEP             (400)   /* 后轮:0→满 约 25 拍≈125ms */
#define KART_SLEW_STEER_STEP            (800)   /* 转向:惯量小,可略快 ≈62ms */
#define KART_SLEW_ZERO_DWELL_TICKS      (6)     /* 反向前在 0 点驻留 6 拍≈30ms */

/* 上电自检:开机跑一遍左右轮+转向的动作确认接线。
 * 调参阶段必须关(=0):它会在开机 12.5s 内反复写 Power_now 后轮 duty,和速度环抢控制权。
 * 硬件接线确认完、正式跑之前想重新验证接线时再开回 1。 */
#define KART_POWER_BOOT_CHECK_ENABLE    (0)
#define KART_POWER_CHECK_REAR_DUTY      (3000)
#define KART_POWER_CHECK_STEER_LEFT     (4000)
#define KART_POWER_CHECK_STEER_RIGHT    (-4000)
#define KART_POWER_CHECK_RUN_MS         (2000)
#define KART_POWER_CHECK_STOP_MS        (500)
#define KART_POWER_CHECK_RUN_TICKS      (KART_POWER_CHECK_RUN_MS / KART_MAIN_LOOP_PERIOD_MS)
#define KART_POWER_CHECK_STOP_TICKS     (KART_POWER_CHECK_STOP_MS / KART_MAIN_LOOP_PERIOD_MS)

typedef struct
{
    int16 Motor_Duty;
    int16 Servo_Duty;
    int16 Left_Rear_Duty;
    int16 Right_Rear_Duty;
    uint8 Debug_Stage;
} Power_Output_Struct;

extern volatile Power_Output_Struct Power_now;

void power_init(void);
void power_sync(void);
void power_stop(void);
void power_force_pwm_zero(void);    /* 硬兜底:直接写三路 PWM=0,绕过 Power_now/主循环,给 5ms 中断失能分支调 */
void power_force_rear_pwm_zero(void);   /* 硬兜底(只清后轮):直接写两路后轮 PWM=0,不碰转向,给 5ms 中断失能分支调 */
void power_set_motor_duty(int16 duty);
void power_set_rear_duty(int16 left_duty, int16 right_duty);
void power_set_steer_duty(int16 duty);
/* 后轮升幅步长在线调(菜单 Slew Rear)。宏 KART_SLEW_REAR_STEP 只是上电默认。 */
void power_set_slew_rear_step(int16 step);
void power_check_poll(void);
uint8 power_check_is_done(void);

#endif
