/*********************************************************************************************************************
* Kart_TC387 Opensourec Library 即（Kart_TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 Kart_TC387 开源库的一部分
*
* Kart_TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/
#include "isr_config.h"
#include "isr.h"
#include "kart_imu.h"
#include "kart_control.h"
#include "kart_odom.h"
#include "kart_power.h"
#include "kart_horn.h"
#include "kart_multicore.h"
#include "kart_remote.h"
#include "kart_debug_uart.h"
#include "kart_person_link.h"
#include "kart_pedal.h"           /* uart2_rx_isr 里的踏板盒收字节回调 */
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"
/* 2026-08-10 已删除 #include "kart_camera.h": UART1 不再需要动态分派，
 * uart1_rx_isr 直接调 tld7002_callback() 即可。摄像头 UART 配置在 init 期完成，
 * 之后 UART1 静态归灯板，见 kart_camera.h 文件头说明。 */

/* 5ms PIT 节拍计数器:主循环协作式调度的时基,每个 5ms 中断 +1。 */
volatile uint32 g_tick_5ms = 0;

/* 点阵屏 SYNC(P15.8)下降沿累计:诊断用。因扫描为 14 边沿/帧,若每秒读一次
 * 并清零,帧率 = 读数 / 14;计数低/忽高忽低 → SYNC 整形链或 EXTI 丢中断。
 * 出厂固件里这个计数只涨不读:全仓库唯一读它的是
 * zf_device_dot_matrix_screen.c 里的 dot_matrix_screen_test_all_on_sync(),
 * 而那个自检函数在 code/ 和 user/ 里零调用点(只有头文件的声明),VOFA 的
 * 通道表里也没有 SYNC 这一路。留着是零成本的(EXTI 里一条 ++),SYNC 整形链
 * 哪天修好了,把那个自检手动接进 main 就能直接用。 */
volatile uint32 g_dot_sync_edges = 0;

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。

// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    kart_debug_uart_tick_5ms();

    kart_multicore_imu_update();
    kart_control_speed_update();
    kart_multicore_odom_update();

    if(!kart_control_is_enabled())
    {
        power_force_rear_pwm_zero();
    }

    /* 末尾递增 5ms 节拍:主循环协作式调度的时基。放最后,保证本拍传感器/控制
     * 已更新完再放行主循环任务。 */
    g_tick_5ms++;

}


IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);

    kart_horn_isr();

}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // \xbf\xaa\xc6\xf0\xd6\xd0\xb6\xcf\xc7\xb6\xcc\xd7
    pit_clear_flag(CCU61_CH0);

#if DOT_MATRIX_SCREEN_USE_PIT_SCAN
    /* 点阵屏 1ms 软扫时基(2026-07-26)。
     * 原方案靠 TLD7002 SYNC(P15.8)下降沿驱动,但实测 SYNC 一整秒零边沿
     * (灯板 OUT15→LMV321→飞线 这条整形链不出方波),故改用本 PIT 驱动。
     * 每拖一个 entry,2 entry = 1 行,14 entry = 1 帧 → 帧率 1000/14 ≈ 71Hz,肉眼常亮。
     * 本中断优先级已降到 9(< UART1_RX=14),保证 scan() 里 set_duty 的
     * 半双工回环字节能被 uart1_rx_isr 及时搬走(RX 软 FIFO 只 1 字节深)。 */
    dot_matrix_screen_scan();
#endif
}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);





}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);



    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断:空闲
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);

        /* 2026-07-26 实车确认 SYNC 在 P15.8(ERU_CH5,见 exti_ch1_ch5_isr),
         * P15.5 无设备接入,此处仅清标志占位。 */
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, EXTI_CH1_CH5_INT_VECTAB_NUM, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断:TLD7002 点阵屏 SYNC 下降沿
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

        g_dot_sync_edges++;                         /* 诊断:累计 SYNC 边沿,主循环每秒读一次算 SYNC_Hz */

        /* 点阵屏 SYNC(P15.8)每个 PWM 周期触发,逐 entry 推进扫描。
         * 直接在此调 scan():其内 tld7002_set_duty 回读会因本 ISR(pri 61)>UART1_RX(pri 14)
         * 无法被抢占而拿不到应答(返回 COMM_ERROR,被 set_duty 忽略),但 TX+DC_SYNC 照发,
         * 占空比仍下发生效 → 显示正常,无死锁(回读为单次非阻塞 fifo_read)。
         * 出厂固件走的是 1ms PIT 软扫(本文件上面 cc61_pit_ch0_isr 那处):下面的
         * #if 取反,而 DOT_MATRIX_SCREEN_USE_PIT_SCAN = 1,所以这里的 scan() 一次都
         * 不会被调。上面那套抢占分析是当年 SYNC 方案的实测记录,切回 SYNC 时仍然有效。 */
#if !DOT_MATRIX_SCREEN_USE_PIT_SCAN
        dot_matrix_screen_scan();
#endif
    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, EXTI_CH2_CH6_INT_VECTAB_NUM, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);

    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    /* 2026-07-24 TLD7002 已飞线到 UART1(见 zf_device_tld7002.h),此处回调挪到 uart1_rx_isr。
     * UART0(P14.0/P14.1)现已空出,飞线后无设备接入,一般不再触发本 ISR;
     * 若有噪声触发,轮询读掉一个字节清 FIFO/中断标志,防悬挂(uart_query_byte 非阻塞)。 */
    {
        uint8 discard;
        (void)uart_query_byte(UART_0, &discard);
    }
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DOT_MATRIX_MUTED
    /* UART1 上真正在跑的是下面 #else 的 tld7002_callback() —— 灯板是插着的:
     * DOT_MATRIX_MUTED 展开是 (PERSON_LINK_ENABLE && PORT == LIGHT),而
     * board_pins.h 里 PERSON_LINK_ENABLE = 0,所以本 #if 整块不编译。
     * 把 UART1 改归 TC4D7 人体视觉链路(灯板拔了、4D7 插那个座子)那一档才
     * 走这里。必须二选一、不能两个都调:两边都从同一个 1 字节深的 RX FIFO
     * 取字节,谁先取走另一方就永远收不到;而且 tld7002_callback() 用的是
     * 阻塞式 uart_read_byte(),没字节时会在中断里死自旋。 */
    kart_person_link_rx_callback();
#else
    /* 2026-07-24 TLD7002 飞线到 UART1(P11.12/P11.10)。喂 TLD7002 回调:把芯片响应/
     * 半双工回环字节写入 fifo,否则 init/setDuty 诊断永不完成,列输出停在高阻态。
     */
    tld7002_callback();
#endif
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    /* CH32 油门/刹车踏板盒收字节(UART_2 = ASCLIN2,RX = P02.1)。
     * 原先这里挂的是逐飞例程的 wireless_module_uart_handler():本车的无线
     * 模块走 SPI(见 kart_wifi.c),那个回调没有对应硬件,是模板残留。 */
    kart_pedal_rx_callback();



}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    kart_remote_rx_callback();                      // SBUS 遥控接收:逐字节攒帧解析(UART3,P15.7 RX)
}


IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

//IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // 开启中断嵌套
//
//
//
//}

IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    /* 语音模块不在这里：kart_voice_poll() 用 uart_query_byte() 直读硬件 RX FIFO，
     * 它的帧很稀疏（人说一句才来一帧），轮询足够。
     * 人体视觉链路不同：115200 下背靠背连发，25 字节一帧只需 ~2.2ms，
     * 放到 10ms 任务里轮询必丢字节，所以这里必须用中断。
     * 【出厂档这个 ISR 是空的】下面那个 #if 两个条件都不成立:board_pins.h 里
     * PERSON_LINK_ENABLE = 0,而且 PERSON_LINK_PORT 选的是 PORT_LIGHT 不是
     * PORT_VOFA。所以 UART10 的 RX 中断进来什么也不做(只有一句
     * interrupt_global_enable)。UART10 在出厂档归语音,而语音是轮询的。上面那套
     * "必须用中断"的论证只在 PLINK 走 VOFA 口那一档成立,留着备用。 */
#if (PERSON_LINK_ENABLE && (PERSON_LINK_PORT == PERSON_LINK_PORT_VOFA))
    kart_person_link_rx_callback();
#endif



}

IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart4_handle);
}
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart5_handle);
}
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart6_handle);
}
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart8_handle);
}
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart9_handle);
}
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart10_handle);
}
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart11_handle);
}
// **************************** 串口中断函数 ****************************
