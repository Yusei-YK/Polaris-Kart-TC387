#include "zf_common_headfile.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_debug_uart.h"
#include "kart_imu.h"
#include "kart_remote.h"
#include "kart_control.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

    power_init();
    kart_encoder_init();
    kart_steer_abs_init();
    kart_debug_uart_init();
    kart_remote_init();
    kart_control_init();        // 速度环:填默认 PID、清滤波、默认不使能(等 VOFA 发 e1 才输出)

    /* IMU 初始化 + 上电静止标定零偏。
     * 注意:kart_imu_init() 里有 system_delay_ms(1000) 阻塞标定,
     *       必须在"开 5ms 中断之前"跑完 —— 此时车必须放稳别动。 */
    kart_imu_init();

    /* 开 5ms 周期中断,进 cc60_pit_ch0_isr 调 kart_imu_update()。
     * 放在标定之后:保证进中断时零偏已就绪,解算从第一帧就是准的。 */
    pit_ms_init(CCU60_CH0, KART_MAIN_LOOP_PERIOD_MS);

    cpu_wait_event_ready();

    while(TRUE)
    {
        /* 主循环:自检、后轮控制权仲裁、执行输出、串口调试。
         * 航向解算和速度环 PID 已交给 5ms 中断,这里不再碰。 */
        power_check_poll();

        /* 刷新转向绝对编码器读数(SPI 读,放主循环别放 5ms 中断)。
         * 只有这里周期调 update,steer_raw / center_delta 才会跟着方向盘变;
         * 否则读数停在上电那一帧。调试显示 + 后续角度环都依赖它。 */
        kart_steer_abs_update();

        /* 后轮 duty 的控制权仲裁 —— 同一时刻只能有一个源写后轮,否则互相打架:
         *   速度环使能(VOFA 发了 e1) → 5ms 中断里的速度环独占后轮,主循环不碰;
         *   速度环关闭            → 交给遥控 kart_remote_poll() 接管(遥控/急停)。
         * 悬空调参时遥控没接,速度环一使能就由中断驱动后轮,不会被 remote 的离线清零打断。 */
        if(!kart_control_is_enabled())
        {
            kart_remote_poll();
        }

        power_sync();
        kart_debug_uart_poll();
        system_delay_ms(KART_MAIN_LOOP_PERIOD_MS);
    }
}

#pragma section all restore
