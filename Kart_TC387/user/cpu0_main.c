#include "zf_common_headfile.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_debug_uart.h"
#include "kart_imu.h"
#include "kart_control.h"
#include "kart_odom.h"
#include "kart_steer_ctrl.h"
#include "kart_voice.h"
#include "kart_horn.h"
#include "kart_record.h"
#include "kart_playback.h"
#include "kart_mission.h"
#include "kart_remote.h"
#include "kart_hw_test.h"
#include "kart_menu.h"
#include "zf_device_dot_matrix_screen.h"

/* 硬件自测开关:=1 时开机进 kart_hw_test_run() 死循环(验证并口屏/旋钮/按键),
 * 不跑正式主循环。硬件确认完必须改回 0。 */
#define KART_HW_TEST    (0)

/* 菜单系统开关:=1 时启用 IPS200 菜单(取代 VOFA m 命令切科目) */
#define KART_USE_MENU   (1)

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

#if KART_HW_TEST
    kart_hw_test_run();     /* 内部死循环,不返回 */
#endif

    power_init();
    kart_encoder_init();
    kart_steer_abs_init();
    kart_debug_uart_init();
    kart_control_init();        // 速度环:填默认 PID、清滤波、默认不使能(等 VOFA 发 e1 才输出)
    kart_steer_ctrl_init();     // 转向串级:填内/外环默认 PID,默认全不使能(等 VOFA 发 se1 才驱动转向电机)

    /* IMU660RA 初始化 + 上电静止标定零偏。
     * 注意:kart_imu_init() 先静置 1s,再采 300 次(约 1.5s),
     *       必须在"开 5ms 中断之前"跑完 —— 此时车必须放稳别动。 */
    kart_imu_init();

    /* 航位推算基准初始化：必须在 IMU/编码器就绪后、开 5ms 中断前。 */
    kart_odom_init();

    /* 开 5ms 周期中断,进 cc60_pit_ch0_isr 调 kart_imu_update()。
     * 放在标定之后:保证进中断时零偏已就绪,解算从第一帧就是准的。 */
    pit_ms_init(CCU60_CH0, KART_MAIN_LOOP_PERIOD_MS);

    dot_matrix_screen_init();
    dot_matrix_screen_set_brightness(10000);

    /* 语音/鸣笛底层 init 在启动时做一次(UART_2 常驻,避免反复 init);
     * 收帧/分发/节拍推进已交给科目二 loop,只在 SUBJECT_2 模式下轮询。 */
    kart_voice_init();
    kart_horn_init();
    kart_record_init();
    kart_playback_init();

    /* 科目状态机:置 IDLE 并执行统一停机,保证上电无残留输出。 */
    kart_mission_init();

    /* SBUS 枪式遥控接收(UART3,P15.7 RX)。第一版只解析+失联计数,只上 VOFA 观测,
     * 不接管电机/转向(见 kart_remote.h 安全红线)。 */
    kart_remote_init();

#if KART_USE_MENU
    /* IPS200 菜单系统:科目选择 + 路线录制/复现管理。
     * 启用后取代 VOFA m 命令,通过屏幕菜单 + 五向按键操作。 */
    kart_menu_init();
#endif

    cpu_wait_event_ready();

    while(TRUE)
    {
        /* 主循环:自检、后轮控制权仲裁、执行输出、串口调试。
         * 航向解算和速度环 PID 已交给 5ms 中断,这里不再碰。 */
        /* 点阵屏显当前模式号:000=待机 111=科目一 222=科目二 333=遥控 FFF=故障 */
        switch(kart_mission_get_mode())
        {
            case MISSION_SUBJECT_1: dot_matrix_screen_show_string("111"); break;
            case MISSION_SUBJECT_2: dot_matrix_screen_show_string("222"); break;
            case MISSION_REMOTE:    dot_matrix_screen_show_string("333"); break;
            case MISSION_FAULT:     dot_matrix_screen_show_string("FFF"); break;
            case MISSION_IDLE:
            default:                dot_matrix_screen_show_string("000"); break;
        }
        power_check_poll();

        /* 刷新转向绝对编码器读数(SPI 读,放主循环别放 5ms 中断)。
         * 只有这里周期调 update,steer_raw / center_delta 才会跟着方向盘变;
         * 否则读数停在上电那一帧。调试显示 + 后续角度环都依赖它。 */
        kart_steer_abs_update();

        /* 路径复现一拍:Pure Pursuit 找前视点 → 算方位角 → 覆写航向外环目标。
         * 必须在 steer_abs_update 之后(依赖当前位置)、steer_ctrl_update 之前
         * (好让航向外环这一拍吃到新目标)。b0 时空操作。 */
        kart_playback_poll();

        /* 转向串级一拍:必须紧跟 steer_abs_update,吃到本拍最新 center_delta。
         * 内环未使能时它自己会输出 0 并清 PID,不会和归零分支打架。 */
        kart_steer_ctrl_update();

        /* 速度环未使能时强制后轮归零;转向串级独立运行,不受速度环门控。 */
        if(!kart_control_is_enabled())
        {
            power_set_rear_duty(0, 0);
        }

        power_sync();
        dot_matrix_screen_scan();

        /* SBUS 遥控失联计时更新(只更新在线态,不接管控制)。放 debug_poll 前,
         * 让 VOFA 读到本拍最新在线态。 */
        kart_remote_poll(KART_MAIN_LOOP_PERIOD_MS);

        // kart_debug_uart_poll();   // VOFA 已停用(改用屏幕调试,省算力),需要时恢复

        /* 科目状态机一拍:按当前模式分发。科目二的语音/分发/鸣笛已归入其 loop,
         * 只在 SUBJECT_2 模式下轮询,IDLE/科目一不响应语音。 */
        kart_mission_poll();
        kart_record_poll();         // 录制自适应采样(r0 状态下空操作)

#if KART_USE_MENU
        /* 菜单系统一拍:按键扫描 + 状态机更新 + IPS200 屏幕刷新。 */
        kart_menu_poll();
#endif

        system_delay_ms(KART_MAIN_LOOP_PERIOD_MS);
    }
}

#pragma section all restore
