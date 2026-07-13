/*********************************************************************************************************************
* TC387 Opensourec Library ����TC387 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC387 ��Դ���һ����
*
* TC387 ��Դ�� ���������
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
#include "board_pins.h"                                 // KART_MAIN_LOOP_PERIOD_MS 的板级定义
#include "kart_remote.h"
#include "kart_imu.h"                                   // 5ms 中断里要调 kart_imu_update()
#include "kart_control.h"                               // 5ms 中断里要调 kart_control_speed_update()
#include "kart_mission.h"                               // 5ms 中断只推进 Mission 计时，不改状态或操作屏幕

// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// **************************** PIT�жϺ��� ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);

    kart_imu_update();                              // 5ms 周期:读 IMU + Madgwick 解算航向
    kart_control_speed_update();                    // 5ms 周期:读编码器→滤波→PID→算 duty(内部按 enable 决定是否下发)
    kart_mission_tick(KART_MAIN_LOOP_PERIOD_MS);    // 固定节拍累计比赛时间，状态转换仍全部留在主循环

}


IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH1);




}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);




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

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




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

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

         

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
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // ͨ��7�ж�
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
    interrupt_global_enable(0);                     // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
        debug_interrupr_handler();                  // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}


// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_uart_handler();                          // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    wireless_module_uart_handler();                 // ����ģ��ͳһ�ص�����



}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    kart_remote_uart_callback();                     // UART3 borrowed from GPS for SBUS receiver



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
