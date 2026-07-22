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

#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_exti.h"
#include "zf_device_tld7002.h"

#include "zf_device_dot_matrix_screen.h"


// 卡丁车灯板使用74HC138译码器，用A0/A1/A2三位二进制选择7行（000~110），EN使能

int8    dot_matrix_screen_data[3];
int8    dot_matrix_screen_data_backup[3];
uint16  dot_matrix_screen_brightness = 10000;


// ������Ļȡģ����
const uint8 tld7002_ascii_font_5x7[][7] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  //    0
    { 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00},  // !  1
    { 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00},  // "  2
    { 0x14, 0x14, 0x3E, 0x14, 0x3E, 0x14, 0x14},  // #  3
    { 0x08, 0x3C, 0x0A, 0x1C, 0x28, 0x1E, 0x08},  // $  4
    { 0x30, 0x32, 0x04, 0x08, 0x10, 0x26, 0x06},  // %  5
    { 0x0C, 0x12, 0x0A, 0x04, 0x2A, 0x12, 0x2C},  // &  6
    { 0x0C, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00},  // '  7
    { 0x10, 0x08, 0x04, 0x04, 0x04, 0x08, 0x10},  // (  8
    { 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, 0x04},  // )  9
    { 0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00},  // * 10
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00},  // + 11
    { 0x00, 0x00, 0x00, 0x00, 0x18, 0x10, 0x08},  // , 12
    { 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00},  // - 13
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},  // . 14
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01},  // / 15
    { 0x1C, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x1C},  // 0 16
    { 0x08, 0x0C, 0x08, 0x08, 0x08, 0x08, 0x1C},  // 1 17
    { 0x1C, 0x22, 0x20, 0x10, 0x08, 0x04, 0x3E},  // 2 18
    { 0x3E, 0x10, 0x08, 0x10, 0x20, 0x22, 0x1C},  // 3 19
    { 0x10, 0x18, 0x14, 0x12, 0x3E, 0x10, 0x10},  // 4 20
    { 0x3E, 0x02, 0x1E, 0x20, 0x20, 0x22, 0x1C},  // 5 21
    { 0x18, 0x04, 0x02, 0x1E, 0x22, 0x22, 0x1C},  // 6 22
    { 0x3E, 0x20, 0x10, 0x08, 0x04, 0x04, 0x04},  // 7 23
    { 0x1C, 0x22, 0x22, 0x1C, 0x22, 0x22, 0x1C},  // 8 24
    { 0x1C, 0x22, 0x22, 0x3C, 0x20, 0x10, 0x0C},  // 9 25
    { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},  // : 26
    { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x08, 0x04},  // ; 27
    { 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10},  // < 28
    { 0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00},  // = 29
    { 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04},  // > 30
    { 0x1C, 0x22, 0x20, 0x10, 0x08, 0x00, 0x08},  // ? 31
    { 0x1C, 0x22, 0x20, 0x2C, 0x3A, 0x22, 0x1C},  // @ 32
    { 0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22},  // A 33
    { 0x1E, 0x22, 0x22, 0x1E, 0x22, 0x22, 0x1E},  // B 34
    { 0x1C, 0x22, 0x02, 0x02, 0x02, 0x22, 0x1C},  // C 35
    { 0x0E, 0x12, 0x22, 0x22, 0x22, 0x12, 0x0E},  // D 36
    { 0x3E, 0x02, 0x02, 0x1E, 0x02, 0x02, 0x3E},  // E 37
    { 0x3E, 0x02, 0x02, 0x1E, 0x02, 0x02, 0x02},  // F 38
    { 0x1C, 0x22, 0x02, 0x3A, 0x22, 0x22, 0x3C},  // G 39
    { 0x22, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22},  // H 40
    { 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C},  // I 41
    { 0x38, 0x10, 0x10, 0x10, 0x10, 0x12, 0x0C},  // J 42
    { 0x22, 0x12, 0x0A, 0x06, 0x0A, 0x12, 0x22},  // K 43
    { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x3E},  // L 44
    { 0x22, 0x36, 0x2A, 0x2A, 0x22, 0x22, 0x22},  // M 45
    { 0x22, 0x22, 0x26, 0x2A, 0x32, 0x22, 0x22},  // N 46
    { 0x1C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C},  // O 47
    { 0x1E, 0x22, 0x22, 0x1E, 0x02, 0x02, 0x02},  // P 48
    { 0x1C, 0x22, 0x22, 0x22, 0x2A, 0x12, 0x2C},  // Q 49
    { 0x1E, 0x22, 0x22, 0x1E, 0x0A, 0x12, 0x22},  // R 50
    { 0x3C, 0x02, 0x02, 0x1C, 0x20, 0x20, 0x1E},  // S 51
    { 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},  // T 52
    { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C},  // U 53
    { 0x22, 0x22, 0x22, 0x22, 0x22, 0x14, 0x08},  // V 54
    { 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x2A, 0x14},  // W 55
    { 0x22, 0x22, 0x14, 0x08, 0x14, 0x22, 0x22},  // X 56
    { 0x22, 0x22, 0x22, 0x14, 0x08, 0x08, 0x08},  // Y 57
    { 0x3E, 0x20, 0x10, 0x08, 0x04, 0x02, 0x3E},  // Z 58
    { 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1C},  // [ 59
    { 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20},  // \ 60
    { 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C},  // ] 61
    { 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00},  // ^ 62
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E},  // _ 63
    { 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00},  // ` 64
    { 0x00, 0x00, 0x1C, 0x20, 0x3C, 0x22, 0x3C},  // a 65
    { 0x02, 0x02, 0x1A, 0x26, 0x22, 0x22, 0x1E},  // b 66
    { 0x00, 0x00, 0x1C, 0x02, 0x02, 0x22, 0x1C},  // c 67
    { 0x20, 0x20, 0x2C, 0x32, 0x22, 0x22, 0x3C},  // d 68
    { 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x02, 0x1C},  // e 69
    { 0x18, 0x24, 0x04, 0x0E, 0x04, 0x04, 0x04},  // f 70
    { 0x00, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x1C},  // g 71
    { 0x02, 0x02, 0x1A, 0x26, 0x22, 0x22, 0x22},  // h 72
    { 0x08, 0x00, 0x0C, 0x08, 0x08, 0x08, 0x1C},  // i 73
    { 0x10, 0x00, 0x18, 0x10, 0x10, 0x12, 0x0C},  // j 74
    { 0x02, 0x02, 0x12, 0x0A, 0x06, 0x0A, 0x12},  // k 75
    { 0x0C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C},  // l 76
    { 0x00, 0x00, 0x16, 0x2A, 0x2A, 0x22, 0x22},  // m 77
    { 0x00, 0x00, 0x1A, 0x26, 0x22, 0x22, 0x22},  // n 78
    { 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C},  // o 79
    { 0x00, 0x1E, 0x22, 0x22, 0x1E, 0x02, 0x02},  // p 80
    { 0x00, 0x2C, 0x32, 0x32, 0x2C, 0x20, 0x20},  // q 81
    { 0x00, 0x00, 0x1A, 0x26, 0x02, 0x02, 0x02},  // r 82
    { 0x00, 0x00, 0x1C, 0x02, 0x1C, 0x20, 0x1E},  // s 83
    { 0x04, 0x04, 0x0E, 0x04, 0x04, 0x24, 0x18},  // t 84
    { 0x00, 0x00, 0x22, 0x22, 0x22, 0x32, 0x2C},  // u 85
    { 0x00, 0x00, 0x22, 0x22, 0x22, 0x14, 0x08},  // v 86
    { 0x00, 0x00, 0x22, 0x22, 0x2A, 0x2A, 0x14},  // w 87
    { 0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22},  // x 88
    { 0x00, 0x22, 0x22, 0x22, 0x3C, 0x20, 0x1C},  // y 89
    { 0x00, 0x00, 0x3E, 0x10, 0x08, 0x04, 0x3E},  // z 90
    { 0x0C, 0x04, 0x06, 0x02, 0x06, 0x04, 0x0C},  // { 91
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},  // | 92
    { 0x0C, 0x08, 0x18, 0x10, 0x18, 0x08, 0x0C},  // } 93
};

//-------------------------------------------------------------------------------------------------------------------
//  �������      ������ɨ�躯��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      �˺�����Ҫ��exti_ch3_ch7_isr�жϵ�ERU_CH7_REQ16_P15_1��֧�е���
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_scan(void)
{
    uint8 i;
    static uint8 entry_num;
    uint8 display_row_now;

    display_row_now = entry_num / 2;
    if(0 == (entry_num % 2))
    {
        gpio_high(DOT_MATRIX_SCREEN_ROW_EN_PIN);

        if(7 > display_row_now)
        {
            for(i = 0; 5 > i; i++)
            {
                if((tld7002_ascii_font_5x7[dot_matrix_screen_data_backup[0] - ' '][display_row_now]) & ((uint16)1 << (i + 1)))
                {
                    tld7002_duty[i] = dot_matrix_screen_brightness;
                }
                else
                {
                    tld7002_duty[i] = 0;
                }
            }

            for(; 10 > i; i++)
            {
                if((tld7002_ascii_font_5x7[dot_matrix_screen_data_backup[1] - ' '][display_row_now]) & ((uint16)1 << (i - 5 + 1)))
                {
                    tld7002_duty[i] = dot_matrix_screen_brightness;
                }
                else
                {
                    tld7002_duty[i] = 0;
                }
            }

            for(; 15 > i; i++)
            {
                if((tld7002_ascii_font_5x7[dot_matrix_screen_data_backup[2] - ' '][display_row_now]) & ((uint16)1 << (i - 10 + 1)))
                {
                    tld7002_duty[i] = dot_matrix_screen_brightness;
                }
                else
                {
                    tld7002_duty[i] = 0;
                }
            }
        }
        tld7002_set_duty(1);
    }
    else
    {
        if(display_row_now & 0x01)
        {
            gpio_high(DOT_MATRIX_SCREEN_ROW_A0_PIN);
        }
        else
        {
            gpio_low(DOT_MATRIX_SCREEN_ROW_A0_PIN);
        }

        if(display_row_now & 0x02)
        {
            gpio_high(DOT_MATRIX_SCREEN_ROW_A1_PIN);
        }
        else
        {
            gpio_low(DOT_MATRIX_SCREEN_ROW_A1_PIN);
        }

        if(display_row_now & 0x04)
        {
            gpio_high(DOT_MATRIX_SCREEN_ROW_A2_PIN);
        }
        else
        {
            gpio_low(DOT_MATRIX_SCREEN_ROW_A2_PIN);
        }

        gpio_low(DOT_MATRIX_SCREEN_ROW_EN_PIN);
    }

    entry_num++;
    if((DOT_MATRIX_SCREEN_ROW_NUM * 2) <= entry_num)
    {
        entry_num = 0;
        memcpy((uint8 *)dot_matrix_screen_data_backup, (uint8 *)dot_matrix_screen_data, sizeof(dot_matrix_screen_data_backup));
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      �������ַ�����ʾ����
//  ����˵��      *str          ��Ҫ��ʾ���ַ��������֧����ʾ3���ַ�
//  ���ز���      void
//  ʹ��ʾ��      dot_matrix_screen_show_string("OK");
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_show_string(const char *str)
{
    uint8 i;
    for(i = 0; i < 3; i++)
    {
        if(0 == str[i])
        {
            break;
        }
        dot_matrix_screen_data[i] = str[i];
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ���������ȵ���
//  ����˵��      brightness    ���õ���Ļ���� 0-10000
//  ���ز���      void
//  ʹ��ʾ��      dot_matrix_screen_show_string("OK");
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_set_brightness(uint16 brightness)
{
    dot_matrix_screen_brightness = brightness;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ��������ʼ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      ��Ҫ�ڵ���
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_init(void)
{
    tld7002_init();

    gpio_init(DOT_MATRIX_SCREEN_ROW_A0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DOT_MATRIX_SCREEN_ROW_A1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DOT_MATRIX_SCREEN_ROW_A2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DOT_MATRIX_SCREEN_ROW_EN_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    tld7002_duty[15] = 5000;
    tld7002_set_duty(1);
    system_delay_ms(10);

    dot_matrix_screen_set_brightness(dot_matrix_screen_brightness);
    dot_matrix_screen_show_string("   ");
}
