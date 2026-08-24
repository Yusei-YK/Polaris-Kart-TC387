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
* �ļ�����          zf_device_dot_matrix_screen
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
*                  SR0                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR0_PIN�궨��
*                  SR1                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR1_PIN�궨��
*                  SR2                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR2_PIN�궨��
*                  SR3                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR3_PIN�궨��
*                  SR4                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR4_PIN�궨��
*                  SR5                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR5_PIN�궨��
*                  SR6                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR6_PIN�궨��
*                  SYNC                �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SYNC_PIN�궨��
*                  GND                 ��Դ��
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_dot_matrix_screen_h_
#define _zf_device_dot_matrix_screen_h_
#include "zf_common_typedef.h"
#include "zf_driver_exti.h"
#include "zf_driver_pit.h"


#define DOT_MATRIX_SCREEN_ROW_A0_PIN    (P20_8)
#define DOT_MATRIX_SCREEN_ROW_A1_PIN    (P20_9)
#define DOT_MATRIX_SCREEN_ROW_A2_PIN    (P20_10)
#define DOT_MATRIX_SCREEN_ROW_EN_PIN    (P33_8)

/* SYNC 飞线:TLD7002 每个 PWM 周期在此脚吐一个下降沿,EXTI 触发逐 entry 扫描。
 * 板载 SYNC 走 P32.4,无 ERU 映射(见 zf_driver_exti.h),必须飞线到有 ERU 的脚。
 * 2026-07-26 实车确认:SYNC 飞在 P15.8 = ERU_CH5(走 exti_ch1_ch5_isr),与本定义一致。
 * P15.8 无人占用:无线模块 SPI 排针上的脚,SPI 未使用;IPS200 走软件 SPI(P02.8/P20.3)。 */
#define DOT_MATRIX_SCREEN_SYNC_PIN      (ERU_CH5_REQ1_P15_8)

/* 扫描时基选择(2026-07-26 实车定案):
 *   =1  CCU61_CH0 的 1ms PIT 软扫驱动(当前方案)
 *   =0  TLD7002 SYNC(P15.8)下降沿驱动(官方方案)
 * 为何改软扫:实测屏能亮、UART 回环 2300 字节/秒全健康,但 SYNC 一整秒
 * 零边沿(ch0=0,累计只 6 个且全在开机 init 阶段)—— 灯板的 OUT15→LMV321
 * 比较器→P15.8 飞线这条整形链不出方波。SYNC 降级为纯诊断信号。 */
#define DOT_MATRIX_SCREEN_USE_PIT_SCAN  (1)

/* 1ms 软扫的 PIT 通道:CCU61_CH0(原本空置)。优先级已在 isr_config.h
 * 降到 9(低于 UART1_RX=14),否则 set_duty 的半双工回环字节会被丢。 */
#define DOT_MATRIX_SCREEN_PIT_CH        (CCU61_CH0)

#define DOT_MATRIX_SCREEN_ROW_NUM       (7)

void dot_matrix_screen_scan             (void);
void dot_matrix_screen_show_string      (const char *str);
void dot_matrix_screen_show_frame       (const uint16 *rows);   /* 下发任意 7x15 图案 */
void dot_matrix_screen_clear            (void);
void dot_matrix_screen_set_brightness   (uint16 brightness);
void dot_matrix_screen_set_all_on       (uint8 on);
void dot_matrix_screen_test_row0_static (uint16 hold_ms);
void dot_matrix_screen_test_rows_static (void);
void dot_matrix_screen_test_all_on_sync (void);
void dot_matrix_screen_init             (void);


#endif

