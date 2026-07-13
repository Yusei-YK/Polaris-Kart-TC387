#include "zf_common_headfile.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_debug_uart.h"
#include "kart_imu.h"
#include "kart_remote.h"
#include "kart_control.h"
#include "kart_light.h"
#include "kart_mission.h"
#include "kart_mission_ui.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

    power_init();
    kart_encoder_init();
    if(0U != kart_steer_abs_init())
    {
        zf_log(0, "KART steer absolute encoder init failed");
    }
    kart_debug_uart_init();
    kart_remote_init();
    kart_control_init();        // 速度环:填默认 PID、清滤波、默认不使能(等 VOFA 发 e1 才输出)
    kart_light_init();          // 只初始化 7x15 灯板逻辑，TLD7002 扫描底层仍待接入
    kart_mission_init();        // 比赛状态机默认 IDLE，且所有未验证底层的就绪位均为 0
    kart_mission_ui_init();     // IPS200 菜单与状态页，屏幕刷新只在主循环进行

    /* IMU 初始化 + 上电静止标定零偏。
     * 注意:kart_imu_init() 里有 system_delay_ms(1000) 阻塞标定,
     *       必须在"开 5ms 中断之前"跑完 —— 此时车必须放稳别动。 */
    if(0U != kart_imu_init())
    {
        zf_log(0, "KART IMU963RA init failed");
    }

    /* 开 5ms 周期中断,进 cc60_pit_ch0_isr 调 kart_imu_update()。
     * 放在标定之后:保证进中断时零偏已就绪,解算从第一帧就是准的。 */
    pit_ms_init(CCU60_CH0, KART_MAIN_LOOP_PERIOD_MS);

    cpu_wait_event_ready();

    while(TRUE)
    {
        kart_mission_status_t mission_status;
        kart_mission_request_t mission_request;

        /* 主循环:自检、后轮控制权仲裁、执行输出、串口调试。
         * 航向解算和速度环 PID 已交给 5ms 中断,这里不再碰。 */
        power_check_poll();

        /* Mission 是高层调度，不在中断里操作屏幕或电机。
         * 当前路径/转向底层还没完成，因此这里只消费安全停机和灯板请求；
         * motion 请求保留给后续 kart_path / yaw / steer 控制器接入。 */
        kart_mission_poll();
        kart_mission_get_status(&mission_status);
        kart_mission_get_request(&mission_request);
        kart_light_set_command(mission_request.light);
        kart_light_update(KART_MAIN_LOOP_PERIOD_MS);

        if(kart_mission_stop_is_required() != 0U)
        {
            kart_control_set_enable(0U);
            power_stop();
        }

        /* 刷新转向绝对编码器读数(SPI 读,放主循环别放 5ms 中断)。
         * 只有这里周期调 update,steer_raw / center_delta 才会跟着方向盘变;
         * 否则读数停在上电那一帧。调试显示 + 后续角度环都依赖它。 */
        kart_steer_abs_update();

        /* 后轮 duty 的控制权仲裁 —— 同一时刻只能有一个源写后轮,否则互相打架:
         *   速度环使能(VOFA 发了 e1) → 5ms 中断里的速度环独占后轮,主循环不碰;
         *   速度环关闭            → 交给遥控 kart_remote_poll() 接管(遥控/急停)。
         * 悬空调参时遥控没接,速度环一使能就由中断驱动后轮,不会被 remote 的离线清零打断。 */
        if(mission_status.run_state == KART_MISSION_IDLE && !kart_control_is_enabled())
        {
            kart_remote_poll();
        }

        kart_debug_uart_poll();
        power_sync();
        kart_mission_ui_poll(KART_MAIN_LOOP_PERIOD_MS);
        system_delay_ms(KART_MAIN_LOOP_PERIOD_MS);
    }
}

#pragma section all restore
