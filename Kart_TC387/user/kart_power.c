#include "kart_power.h"

Power_Output_Struct Power_now = {0};

static int16 kart_last_motor_duty = 0;
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
    if(Power_now.Motor_Duty != kart_last_motor_duty)
    {
        power_set_motor_duty(Power_now.Motor_Duty);
    }

    kart_set_dir_pwm(KART_STEER_DIR_PIN, KART_STEER_PWM_PIN, Power_now.Servo_Duty, KART_STEER_MOTOR_SIGN);
    kart_set_dir_pwm(KART_LEFT_REAR_DIR_PIN, KART_LEFT_REAR_PWM_PIN, Power_now.Left_Rear_Duty, KART_LEFT_MOTOR_SIGN);
    kart_set_dir_pwm(KART_RIGHT_REAR_DIR_PIN, KART_RIGHT_REAR_PWM_PIN, Power_now.Right_Rear_Duty, KART_RIGHT_MOTOR_SIGN);
}

void power_stop(void)
{
    power_set_motor_duty(0);
    power_set_steer_duty(0);
    Power_now.Debug_Stage = 0;
    power_sync();
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
