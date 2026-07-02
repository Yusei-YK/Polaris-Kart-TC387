#include "zf_common_headfile.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_debug_uart.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

    power_init();
    kart_encoder_init();
    kart_steer_abs_init();
    kart_debug_uart_init();

    cpu_wait_event_ready();

    while(TRUE)
    {
        power_check_poll();
        power_sync();
        kart_debug_uart_poll();
        system_delay_ms(KART_MAIN_LOOP_PERIOD_MS);
    }
}

#pragma section all restore
