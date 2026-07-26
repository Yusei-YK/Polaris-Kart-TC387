/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ���������
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
* �ļ�����          zf_device_tld7002
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.12
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-22       Seekfree            first version
********************************************************************************************************************/
/********************************************************************************************************************
* ���߶��壺
*                  ------------------------------------
*                  TLD7002����ģ��      ��Ƭ���ܽ�
*                  RX                  �鿴 zf_device_tld7002.h �� TLD7002_UART_RX �궨��
*                  HSLIL               �鿴 zf_device_tld7002.h �� TLD7002_UART_HLSIL �궨��
*                  GPIN0               �鿴 zf_device_tld7002.h �� TLD7002_GPIN0_PIN �궨��
*                  VCC                 6-10V��Դ
*                  GND                 ��Դ��
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_tld7002_h_
#define _zf_device_tld7002_h_


#include "zf_common_typedef.h"


/* 2026-07-24 从 UART0(P14.0/P14.1)飞线改到 UART1(P11.12/P11.10):
 * UART0 实测收发不通(ERR=1/RX=0/回环0),疑引脚复用问题,改用 UART1 定位。
 * 注意:UART1 与摄像头(SCC8660 P02)、无线模块(P33.12/13)、语音模块共用同一硬件外设,
 *       此改动前提=摄像头/无线/语音当前不用。VOFA 遥测在 UART10(P13.0/13.1),不冲突。 */
#define TLD7002_UART_INDEX      (UART_1)            // 串口号(飞线到 UART1)
#define TLD7002_UART_BAUD       (2000000)           // 波特率
#define TLD7002_UART_RX         (UART1_TX_P11_12)  // 接模块RX=MCU TX(P11.12)
#define TLD7002_UART_HLSIL      (UART1_RX_P11_10)  // 接模块HSLI_L=MCU RX(P11.10)

#define TLD7002_GPIN0_PIN       (P00_8)             // 最新网表:GPIN0=核心板U1.107(P00.8);P20.7是START



extern uint16 tld7002_duty[16];

extern volatile int    tld7002_init_err;   // 诊断:initDevice 返回码,0=应答正常,非0=通信失败
extern volatile uint32 tld7002_rx_count;   // 诊断:init 阶段 UART 收到字节数(含半双工回环)
extern volatile uint32 tld7002_tx_count;      // 诊断:累计发出字节数(半双工必回环)
extern volatile uint32 tld7002_rx_after_init; // 诊断:init 返回瞬间 rx_count 快照
extern volatile uint32 tld7002_tx_after_init; // 诊断:init 返回瞬间 tx_count 快照;芯片应答=rx_after-tx_after

void    tld7002_set_duty        (uint8 tld7002_id);
void    tld7002_callback        (void);
void    tld7002_init            (void);

/* 诊断用:只重跑芯片 init(不重配 UART/GPIO/fifo),返回 initDevice 返回码。
 * 0=NO_ERR(芯片正确应答) 1=COMM_ERROR(无有效应答)。供点阵屏自检每秒重试一次。 */



#endif
