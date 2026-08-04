#include "kart_power.h"

volatile Power_Output_Struct Power_now = {0};

static volatile int16 kart_last_motor_duty = 0;
static uint16 kart_power_check_tick = 0;
static uint8 kart_power_check_done = 0;

static int16 kart_limit_duty(int16 duty)
{
    if(duty > KART_POWER_MAX_DUTY)  { return KART_POWER_MAX_DUTY; }
    if(duty < -KART_POWER_MAX_DUTY) { return -KART_POWER_MAX_DUTY; }
    return duty;
}

static void kart_set_dir_pwm(gpio_pin_enum dir_pin, pwm_channel_enum pwm_pin, int16 duty, int8 sign)
{
    int16 out = kart_limit_duty((int16)(duty * sign));

    if(out >= 0)
    {
        gpio_high(dir_pin);
        pwm_set_duty(pwm_pin, (uint32)out);
    }
    else
    {
        gpio_low(dir_pin);
        pwm_set_duty(pwm_pin, (uint32)(-out));
    }
}

/* 后轮升幅步长:宏(400)只作上电默认,运行时由菜单 Slew Rear 改。
 * 为什么要可调:400 duty/拍 = 0→满约 125ms,起步和出弯加速被它限速;
 * 提速阶段要能放开,又不能删掉限速器(反向穿零保护和防满载冲击还得留着)。 */
static int16 kart_slew_rear_step = KART_SLEW_REAR_STEP;

void power_set_slew_rear_step(int16 step)
{
    if(step < 1) step = 1;              /* 0 会让 duty 永远爬不动,钳成 1 */
    kart_slew_rear_step = step;
}

/* 变化率限制器:把 applied 朝 target 每拍最多挪 step,并禁止穿零跳变。
 * 返回本拍实际应输出的 duty。dwell 指针记录零点驻留剩余拍数(反向前强制停顿)。
 * 规则见 kart_power.h 顶部注释:降幅/停车瞬时(安全),升幅限速,反向先归零+驻留。*/
static int16 kart_slew_step(int16 target, int16 applied, uint16 *dwell, int16 step)
{
#if KART_SLEW_ENABLE
    /* 反向请求:目标与当前输出异号(且都非零),先强制归零,不允许直接反向驱动 */
    if(((int32)target * (int32)applied) < 0)
    {
        *dwell = KART_SLEW_ZERO_DWELL_TICKS;    /* 到零后要驻留,给电机泄速时间 */
        target = 0;
    }

    /* 已在零点且处于驻留期:锁 0 直到驻留结束,期间无视反向目标 */
    if(applied == 0 && *dwell > 0)
    {
        (*dwell)--;
        return 0;
    }

    /* 降幅(朝 0 靠)不限速:减小电流/滑行是安全的,含急停到 0 */
    if((target >= 0 && applied >= 0 && target < applied) ||
       (target <= 0 && applied <= 0 && target > applied))
    {
        return target;
    }

    /* 升幅(远离 0)限速:每拍最多挪 step */
    if(target > applied)
    {
        applied += step;
        if(applied > target) { applied = target; }
    }
    else if(target < applied)
    {
        applied -= step;
        if(applied < target) { applied = target; }
    }
    return applied;
#else
    (void)dwell; (void)step;
    return target;
#endif
}

void power_set_motor_duty(int16 duty)
{
    Power_now.Motor_Duty = kart_limit_duty(duty);
    Power_now.Left_Rear_Duty = Power_now.Motor_Duty;
    Power_now.Right_Rear_Duty = Power_now.Motor_Duty;
    kart_last_motor_duty = Power_now.Motor_Duty;
}

void power_set_rear_duty(int16 left_duty, int16 right_duty)
{
    Power_now.Left_Rear_Duty = kart_limit_duty(left_duty);
    Power_now.Right_Rear_Duty = kart_limit_duty(right_duty);
    Power_now.Motor_Duty = (int16)((Power_now.Left_Rear_Duty + Power_now.Right_Rear_Duty) / 2);
    kart_last_motor_duty = Power_now.Motor_Duty;
}

void power_set_steer_duty(int16 duty)
{
    Power_now.Servo_Duty = kart_limit_duty(duty);
}

void power_init(void)
{
    gpio_init(KART_STEER_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(KART_LEFT_REAR_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(KART_RIGHT_REAR_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(KART_STEER_PWM_PIN, KART_POWER_PWM_FREQ_HZ, 0);
    pwm_init(KART_LEFT_REAR_PWM_PIN, KART_POWER_PWM_FREQ_HZ, 0);
    pwm_init(KART_RIGHT_REAR_PWM_PIN, KART_POWER_PWM_FREQ_HZ, 0);

    Power_now.Debug_Stage = 0;
    power_set_motor_duty(KART_BOOT_MOTOR_DUTY);
    power_set_steer_duty(KART_BOOT_SERVO_DUTY);
}

void power_sync(void)
{
    static int16  steer_applied = 0, left_applied = 0, right_applied = 0;
    static uint16 steer_dwell = 0, left_dwell = 0, right_dwell = 0;
    int16 t_servo, t_left, t_right, t_motor;
    uint32 primask;

    /* [临界区] 成组快照 Power_now 四个 duty，避免读到 5ms 中断写一半的撕裂值。
     * 关中断只包 4 条赋值（几十ns），不包 slew/PWM 硬件调用，不影响中断实时性。 */
    primask = interrupt_global_disable();
    t_servo = Power_now.Servo_Duty;
    t_left  = Power_now.Left_Rear_Duty;
    t_right = Power_now.Right_Rear_Duty;
    t_motor = Power_now.Motor_Duty;
    interrupt_global_enable(primask);

    if(t_motor != kart_last_motor_duty)
    {
        power_set_motor_duty(t_motor);
    }

    /* 每路先过变化率限制器,再写 PWM。target 来自速度环/仲裁,applied 是上拍实际输出。 */
    steer_applied = kart_slew_step(t_servo, steer_applied, &steer_dwell, KART_SLEW_STEER_STEP);
    left_applied  = kart_slew_step(t_left,  left_applied,  &left_dwell,  kart_slew_rear_step);
    right_applied = kart_slew_step(t_right, right_applied, &right_dwell, kart_slew_rear_step);

    kart_set_dir_pwm(KART_STEER_DIR_PIN, KART_STEER_PWM_PIN, steer_applied, KART_STEER_MOTOR_SIGN);
    kart_set_dir_pwm(KART_LEFT_REAR_DIR_PIN, KART_LEFT_REAR_PWM_PIN, left_applied, KART_LEFT_MOTOR_SIGN);
    kart_set_dir_pwm(KART_RIGHT_REAR_DIR_PIN, KART_RIGHT_REAR_PWM_PIN, right_applied, KART_RIGHT_MOTOR_SIGN);
}

void power_stop(void)
{
    power_set_motor_duty(0);
    power_set_steer_duty(0);
    Power_now.Debug_Stage = 0;
    power_sync();
}

/* 硬兜底：直接写三路 PWM=0，绕过 Power_now/主循环/slew。
 * 专给 5ms 中断失能分支调，作最后防线：主循环卡死也能停。
 * 不碰 DIR（duty=0 方向无意义）、不碰 Power_now（不跟主循环抢结构体）。*/
void power_force_pwm_zero(void)
{
    pwm_set_duty(KART_STEER_PWM_PIN, 0);
    pwm_set_duty(KART_LEFT_REAR_PWM_PIN, 0);
    pwm_set_duty(KART_RIGHT_REAR_PWM_PIN, 0);
}

/* 硬兜底(只清后轮版):直接写两路后轮 PWM=0,不碰转向。
 * 专给 5ms 中断失能分支调:速度环未使能时急停后轮,但让转向串级独立驱动
 * (复现/调航向时速度环常关,转向仍需能动)。 */
void power_force_rear_pwm_zero(void)
{
    pwm_set_duty(KART_LEFT_REAR_PWM_PIN, 0);
    pwm_set_duty(KART_RIGHT_REAR_PWM_PIN, 0);
}

uint8 power_check_is_done(void)
{
    return kart_power_check_done;
}

void power_check_poll(void)
{
#if KART_POWER_BOOT_CHECK_ENABLE
    uint16 t = kart_power_check_tick;
    uint16 run = KART_POWER_CHECK_RUN_TICKS;
    uint16 stop = KART_POWER_CHECK_STOP_TICKS;

    if(kart_power_check_done)
    {
        return;
    }

    if(t < run)
    {
        Power_now.Debug_Stage = 1;
        power_set_rear_duty(KART_POWER_CHECK_REAR_DUTY, 0);
        power_set_steer_duty(0);
    }
    else if(t < run + stop)
    {
        Power_now.Debug_Stage = 10;
        power_set_rear_duty(0, 0);
        power_set_steer_duty(0);
    }
    else if(t < run * 2 + stop)
    {
        Power_now.Debug_Stage = 2;
        power_set_rear_duty(0, KART_POWER_CHECK_REAR_DUTY);
        power_set_steer_duty(0);
    }
    else if(t < run * 2 + stop * 2)
    {
        Power_now.Debug_Stage = 20;
        power_set_rear_duty(0, 0);
        power_set_steer_duty(0);
    }
    else if(t < run * 3 + stop * 2)
    {
        Power_now.Debug_Stage = 3;
        power_set_rear_duty(0, 0);
        power_set_steer_duty(KART_POWER_CHECK_STEER_LEFT);
    }
    else if(t < run * 3 + stop * 3)
    {
        Power_now.Debug_Stage = 30;
        power_set_rear_duty(0, 0);
        power_set_steer_duty(0);
    }
    else if(t < run * 4 + stop * 3)
    {
        Power_now.Debug_Stage = 4;
        power_set_rear_duty(0, 0);
        power_set_steer_duty(KART_POWER_CHECK_STEER_RIGHT);
    }
    else if(t < run * 4 + stop * 4)
    {
        Power_now.Debug_Stage = 40;
        power_set_rear_duty(0, 0);
        power_set_steer_duty(0);
    }
    else if(t < run * 5 + stop * 4)
    {
        Power_now.Debug_Stage = 5;
        power_set_rear_duty(KART_POWER_CHECK_REAR_DUTY, KART_POWER_CHECK_REAR_DUTY);
        power_set_steer_duty(0);
    }
    else
    {
        power_stop();
        kart_power_check_done = 1;
    }

    kart_power_check_tick++;
#else
    Power_now.Debug_Stage = 0;
#endif
}
