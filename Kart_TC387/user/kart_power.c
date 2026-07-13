#include "kart_power.h"
#include "kart_remote.h"

/* 速度环中断会写输出请求，主循环会读取并下发 PWM；访问必须使用临界区。 */
static volatile Power_Output_Struct Power_now = {0};
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
    uint32 primask = interrupt_global_disable();
    int16 limited = kart_limit_duty(duty);

    Power_now.Motor_Duty = limited;
    Power_now.Left_Rear_Duty = limited;
    Power_now.Right_Rear_Duty = limited;

    interrupt_global_enable(primask);
}

void power_set_rear_duty(int16 left_duty, int16 right_duty)
{
    uint32 primask = interrupt_global_disable();
    Power_now.Left_Rear_Duty = kart_limit_duty(left_duty);
    Power_now.Right_Rear_Duty = kart_limit_duty(right_duty);
    Power_now.Motor_Duty = (int16)((Power_now.Left_Rear_Duty + Power_now.Right_Rear_Duty) / 2);
    interrupt_global_enable(primask);
}

void power_set_steer_duty(int16 duty)
{
    uint32 primask = interrupt_global_disable();
    Power_now.Servo_Duty = kart_limit_duty(duty);
    interrupt_global_enable(primask);
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
    int16 steer_duty;
    int16 left_rear_duty;
    int16 right_rear_duty;
    uint32 primask = interrupt_global_disable();

    steer_duty = Power_now.Servo_Duty;
    left_rear_duty = Power_now.Left_Rear_Duty;
    right_rear_duty = Power_now.Right_Rear_Duty;

    interrupt_global_enable(primask);

    kart_set_dir_pwm(KART_STEER_DIR_PIN, KART_STEER_PWM_PIN, steer_duty, KART_STEER_MOTOR_SIGN);
    kart_set_dir_pwm(KART_LEFT_REAR_DIR_PIN, KART_LEFT_REAR_PWM_PIN, left_rear_duty, KART_LEFT_MOTOR_SIGN);
    kart_set_dir_pwm(KART_RIGHT_REAR_DIR_PIN, KART_RIGHT_REAR_PWM_PIN, right_rear_duty, KART_RIGHT_MOTOR_SIGN);
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

volatile kart_remote_t kart_remote = {0};

static uint8 kart_remote_raw[KART_REMOTE_FRAME_LEN] = {0};
static volatile uint16 kart_remote_timeout_ticks = 0;

static int16 kart_remote_limit(int32 value)
{
    if(value > 10000)  { return 10000; }
    if(value < -10000) { return -10000; }
    return (int16)value;
}

static int16 kart_remote_map_channel(uint16 value, int8 reverse)
{
    int32 out = 0;

    if(value > KART_REMOTE_CH_MID - KART_REMOTE_CH_DEAD_ZONE &&
       value < KART_REMOTE_CH_MID + KART_REMOTE_CH_DEAD_ZONE)
    {
        return 0;
    }

    if(value < KART_REMOTE_CH_MID)
    {
        out = -((int32)(KART_REMOTE_CH_MID - KART_REMOTE_CH_DEAD_ZONE - value) * 10000) /
              (KART_REMOTE_CH_MID - KART_REMOTE_CH_DEAD_ZONE - KART_REMOTE_CH_MIN);
    }
    else
    {
        out = ((int32)(value - KART_REMOTE_CH_MID - KART_REMOTE_CH_DEAD_ZONE) * 10000) /
              (KART_REMOTE_CH_MAX - KART_REMOTE_CH_MID - KART_REMOTE_CH_DEAD_ZONE);
    }

    if(reverse)
    {
        out = -out;
    }

    return kart_remote_limit(out);
}

static void kart_remote_parse_frame(uint8 *buffer)
{
    uint8 num = 0;
    uint16 ch4 = 0;

    kart_remote.channel[num++] = (buffer[1] | buffer[2] << 8) & 0x07FF;
    kart_remote.channel[num++] = (buffer[2] >> 3 | buffer[3] << 5) & 0x07FF;
    kart_remote.channel[num++] = (buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF;
    kart_remote.channel[num++] = (buffer[5] >> 1 | buffer[6] << 7) & 0x07FF;
    kart_remote.channel[num++] = (buffer[6] >> 4 | buffer[7] << 4) & 0x07FF;
    kart_remote.channel[num++] = (buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF;

    kart_remote.online = ((buffer[23] & KART_REMOTE_FAILSAFE_FLAG) == 0) ? 1 : 0;
    kart_remote.steering = kart_remote_map_channel(kart_remote.channel[0], 1);
    kart_remote.throttle = kart_remote_map_channel(kart_remote.channel[1], 0);

    ch4 = kart_remote.channel[3];
    if(ch4 >= KART_REMOTE_ENABLE_CH_HIGH)
    {
        kart_remote.switch_stage = 2;
    }
    else if(ch4 >= KART_REMOTE_ENABLE_CH_LOW)
    {
        kart_remote.switch_stage = 1;
    }
    else
    {
        kart_remote.switch_stage = 0;
    }

    kart_remote.frame_ready = 1;
    kart_remote_timeout_ticks = KART_REMOTE_TIMEOUT_TICKS;
}

void kart_remote_init(void)
{
    uart_sbus_init(BOARD_GPS_UART_INDEX,
                   KART_REMOTE_UART_BAUD,
                   BOARD_GPS_UART_TX_PIN,
                   BOARD_GPS_UART_RX_PIN);
    kart_remote_timeout_ticks = 0;
}

void kart_remote_uart_callback(void)
{
    static uint8 length = 0;
    uint8 dat = 0;

    if(uart_query_byte(BOARD_GPS_UART_INDEX, &dat) == 0)
    {
        return;
    }

    if(length == 0 && dat != KART_REMOTE_FRAME_HEAD)
    {
        return;
    }

    kart_remote_raw[length++] = dat;

    if(length >= KART_REMOTE_FRAME_LEN)
    {
        if(kart_remote_raw[0] == KART_REMOTE_FRAME_HEAD &&
           kart_remote_raw[KART_REMOTE_FRAME_LEN - 1] == KART_REMOTE_FRAME_TAIL)
        {
            kart_remote_parse_frame(kart_remote_raw);
        }
        length = 0;
    }
}

void kart_remote_poll(void)
{
    kart_remote_t snapshot;
    uint32 primask = interrupt_global_disable();

    if(kart_remote_timeout_ticks > 0)
    {
        kart_remote_timeout_ticks--;
    }
    else
    {
        kart_remote.online = 0;
        kart_remote.frame_ready = 0;
    }

    snapshot.steering = kart_remote.steering;
    snapshot.throttle = kart_remote.throttle;
    snapshot.switch_stage = kart_remote.switch_stage;
    snapshot.online = kart_remote.online;
    snapshot.frame_ready = kart_remote.frame_ready;

    interrupt_global_enable(primask);

    if(snapshot.online && snapshot.switch_stage == 2)
    {
        power_set_rear_duty(snapshot.throttle, snapshot.throttle);
        power_set_steer_duty(snapshot.steering);
    }
    else
    {
        power_set_rear_duty(0, 0);
        power_set_steer_duty(0);
    }
}
