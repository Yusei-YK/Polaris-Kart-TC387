#ifndef KART_DEBUG_UART_H_
#define KART_DEBUG_UART_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

#define KART_DEBUG_UART_PERIOD_MS       (200)

typedef struct
{
    uint32 t_ms;
    int16 l_delta;
    int16 r_delta;
    int16 l_raw;
    int16 r_raw;
    int32 l_sum;
    int32 r_sum;
    uint16 steer_raw;
    uint16 steer_frame;
    int16 yaw_deg_x100;
    int16 power_motor;
    int16 power_servo;
    uint8 power_stage;
    int16 power_left;
    int16 power_right;
    uint8 l_ab;
    uint8 r_ab;
} kart_debug_sensor_csv_t;

void kart_debug_uart_init(void);
void kart_debug_uart_send_sensor_csv(const kart_debug_sensor_csv_t *data);
void kart_debug_uart_poll(void);

#endif
