#include "kart_debug_uart.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"

static kart_debug_sensor_csv_t kart_debug_data = {0};
static uint32 kart_debug_elapsed_ms = 0;

static uint8 kart_debug_get_encoder_ab(gpio_pin_enum a_pin, gpio_pin_enum b_pin)
{
    return (uint8)((gpio_get_level(a_pin) ? 1 : 0) | (gpio_get_level(b_pin) ? 2 : 0));
}

void kart_debug_uart_init(void)
{
    uart_init(BOARD_WIRELESS_UART_INDEX, BOARD_WIRELESS_UART_BAUD, BOARD_WIRELESS_UART_TX_PIN, BOARD_WIRELESS_UART_RX_PIN);
    uart_write_string(BOARD_WIRELESS_UART_INDEX, "t_ms,l_delta,r_delta,l_raw,r_raw,l_sum,r_sum,steer_raw,steer_frame,yaw_deg,power_motor,power_servo,power_stage,power_left,power_right,l_ab,r_ab\r\n");
}

void kart_debug_uart_send_sensor_csv(const kart_debug_sensor_csv_t *data)
{
    char line[192];
    int16 yaw_abs;

    if(data == 0)
    {
        return;
    }

    yaw_abs = (data->yaw_deg_x100 >= 0) ? data->yaw_deg_x100 : (int16)(-data->yaw_deg_x100);

    sprintf(line, "%lu,%d,%d,%d,%d,%ld,%ld,%u,%u,%s%d.%02d,%d,%d,%u,%d,%d,%u,%u\r\n",
            (unsigned long)data->t_ms,
            data->l_delta,
            data->r_delta,
            data->l_raw,
            data->r_raw,
            (long)data->l_sum,
            (long)data->r_sum,
            data->steer_raw,
            data->steer_frame,
            (data->yaw_deg_x100 < 0) ? "-" : "",
            yaw_abs / 100,
            yaw_abs % 100,
            data->power_motor,
            data->power_servo,
            data->power_stage,
            data->power_left,
            data->power_right,
            data->l_ab,
            data->r_ab);

    uart_write_string(BOARD_WIRELESS_UART_INDEX, line);
}

void kart_debug_uart_poll(void)
{
    kart_debug_elapsed_ms += KART_MAIN_LOOP_PERIOD_MS;
    if(kart_debug_elapsed_ms < KART_DEBUG_UART_PERIOD_MS)
    {
        return;
    }
    kart_debug_elapsed_ms = 0;

    kart_encoder_update();
    kart_steer_abs_update();

    kart_debug_data.t_ms += KART_DEBUG_UART_PERIOD_MS;
    kart_debug_data.l_delta = kart_encoder_get_left_delta();
    kart_debug_data.r_delta = kart_encoder_get_right_delta();
    kart_debug_data.l_raw = kart_encoder_get_left_raw();
    kart_debug_data.r_raw = kart_encoder_get_right_raw();
    kart_debug_data.l_sum = kart_encoder_get_left_sum();
    kart_debug_data.r_sum = kart_encoder_get_right_sum();
    kart_debug_data.steer_raw = kart_steer_abs_get_raw();
    kart_debug_data.steer_frame = kart_steer_abs_get_frame();
    kart_debug_data.yaw_deg_x100 = 0;
    kart_debug_data.power_motor = Power_now.Motor_Duty;
    kart_debug_data.power_servo = Power_now.Servo_Duty;
    kart_debug_data.power_stage = Power_now.Debug_Stage;
    kart_debug_data.power_left = Power_now.Left_Rear_Duty;
    kart_debug_data.power_right = Power_now.Right_Rear_Duty;
    kart_debug_data.l_ab = kart_debug_get_encoder_ab(KART_LEFT_ENCODER_A_GPIO, KART_LEFT_ENCODER_B_GPIO);
    kart_debug_data.r_ab = kart_debug_get_encoder_ab(KART_RIGHT_ENCODER_A_GPIO, KART_RIGHT_ENCODER_B_GPIO);

    kart_debug_uart_send_sensor_csv(&kart_debug_data);
}
