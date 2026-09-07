/*********************************************************************************************************************
* Kart_TC387 Opensourec Library ����Kart_TC387 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� Kart_TC387 ��Դ���һ����
*
* Kart_TC387 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
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

/* 点阵屏 SYNC(P15.8)下降沿累计:诊断用,VOFA/主循环每秒读一次清零 → SYNC_Hz。
 * 因扫描为 14 边沿/帧,帧率 = SYNC_Hz / 14。计数低/忽高忽低 → SYNC 整形链或 EXTI 丢中断。
 * 【2026-09-07 那个"主循环每秒读一次"的读者不存在】全仓库唯一读它的地方是
 * zf_device_dot_matrix_screen.c 里的 dot_matrix_screen_test_all_on_sync(),
 * 而那个自检函数在 code/ 和 user/ 里零调用点(只有头文件的声明)。VOFA 的通道表
 * 里也没有 SYNC 这一路。也就是出厂固件里这个计数只涨不读。
 * 【别删】它是零成本的(EXTI 里一条 ++),而 SYNC 整形链哪天修好了,把那个自检
 * 手动接进 main 就能直接用。要看它,现在得自己去调那个自检函数。 */
volatile uint32 g_dot_sync_edges = 0;

// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// **************************** PIT�жϺ��� ****************************
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
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);





}
// **************************** PIT�жϺ��� ****************************


// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // ͨ��0�ж�
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
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF ģ�� INT �����ж�

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断:TLD7002 点阵屏 SYNC 下降沿
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

        g_dot_sync_edges++;                         /* 诊断:累计 SYNC 边沿,主循环每秒读一次算 SYNC_Hz */

        /* 点阵屏 SYNC(P15.8)每个 PWM 周期触发,逐 entry 推进扫描。
         * 直接在此调 scan():其内 tld7002_set_duty 回读会因本 ISR(pri 61)>UART1_RX(pri 14)
         * 无法被抢占而拿不到应答(返回 COMM_ERROR,被 set_duty 忽略),但 TX+DC_SYNC 照发,
         * 占空比仍下发生效 → 显示正常,无死锁(回读为单次非阻塞 fifo_read)。
         * 【2026-09-07 这段讲的是没编译进去的那条路】下面的 #if 取反,而
         * DOT_MATRIX_SCREEN_USE_PIT_SCAN = 1,所以出厂固件走的是 1ms PIT 软扫
         * (本文件上面 cc61_pit_ch0_isr 那处),这里的 scan() 一次都不会被调。
         * 上面那套抢占分析是当年 SYNC 方案的实测记录,SYNC 修好切回来时仍然有效。 */
#if !DOT_MATRIX_SCREEN_USE_PIT_SCAN
        dot_matrix_screen_scan();
#endif
    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, EXTI_CH2_CH6_INT_VECTAB_NUM, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);

    }
}
// **************************** �ⲿ�жϺ��� ****************************


// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_dma_handler();                           // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



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


// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DOT_MATRIX_MUTED
    /* 2026-08-12 UART1 已改归 TC4D7 人体视觉链路（灯板拔了，4D7 插在那个坐子）。
     * 必须二选一、不能两个都调：两边都从同一个 1 字节深的 RX FIFO 取字节，
     * 谁先取走另一方就永远收不到；而且 tld7002_callback() 用的是阻塞式
     * uart_read_byte()，没字节时会在中断里死自旋。
     * 【2026-09-07 出厂档跑的不是这一支】DOT_MATRIX_MUTED 展开是
     * (PERSON_LINK_ENABLE && PORT == LIGHT),而 board_pins.h 里 PERSON_LINK_ENABLE
     * = 0,所以本 #if 整块不编译,UART1 上真正在跑的是下面 #else 的
     * tld7002_callback() —— 灯板是插着的。"改归 4D7"是 2026-08-12 那次实验的状态,
     * 后来插回来了。二选一那条约束本身永远成立,别为了省事把两个都调上。 */
    kart_person_link_rx_callback();
#else
    /* 2026-07-24 TLD7002 飞线到 UART1(P11.12/P11.10)。喂 TLD7002 回调:把芯片响应/
     * 半双工回环字节写入 fifo,否则 init/setDuty 诊断永不完成,列输出停在高阻态。
     */
    tld7002_callback();
#endif
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    /* CH32 油门/刹车踏板盒收字节(UART_2 = ASCLIN2,RX = P02.1)。
     * 原先这里挂的是逐飞例程的 wireless_module_uart_handler():本车的无线
     * 模块走 SPI(见 kart_wifi.c),那个回调没有对应硬件,是模板残留。 */
    kart_pedal_rx_callback();



}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    kart_remote_rx_callback();                      // SBUS 遥控接收:逐字节攒帧解析(UART3,P15.7 RX)
}


IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

//IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // �����ж�Ƕ��
//
//
//
//}

IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    /* 语音模块不在这里：kart_voice_poll() 用 uart_query_byte() 直读硬件 RX FIFO，
     * 它的帧很稀疏（人说一句才来一帧），轮询足够。
     * 人体视觉链路不同：115200 下背靠背连发，25 字节一帧只需 ~2.2ms，
     * 放到 10ms 任务里轮询必丢字节，所以这里必须用中断。 */
#if (PERSON_LINK_ENABLE && (PERSON_LINK_PORT == PERSON_LINK_PORT_VOFA))
    kart_person_link_rx_callback();
#endif



}

IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}
// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart4_handle);
}
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart5_handle);
}
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart6_handle);
}
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart8_handle);
}
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart9_handle);
}
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart10_handle);
}
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart11_handle);
}
// **************************** �����жϺ��� ****************************
