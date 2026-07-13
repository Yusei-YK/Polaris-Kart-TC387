#ifndef KART_POWER_H_
#define KART_POWER_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

#define KART_POWER_PWM_FREQ_HZ          (17000)
#define KART_POWER_MAX_DUTY             (PWM_DUTY_MAX)
#define KART_BOOT_MOTOR_DUTY            (0)
#define KART_BOOT_SERVO_DUTY            (0)

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

void power_init(void);
void power_sync(void);
void power_stop(void);
void power_set_motor_duty(int16 duty);
void power_set_rear_duty(int16 left_duty, int16 right_duty);
void power_set_steer_duty(int16 duty);
void power_check_poll(void);
uint8 power_check_is_done(void);

#endif
