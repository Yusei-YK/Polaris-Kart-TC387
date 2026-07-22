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
* �ļ�����          zf_device_ips200
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-11-04       pudding            first version
* 2023-04-28       pudding            ��������ע��˵��
* 2023-05-20       pudding            �޸��޷���������������������
* 2025-02-08       pudding            �޸Ĳ���ע�� �Ƴ����������롱��˫�����롱�����淶
********************************************************************************************************************/
/********************************************************************************************************************
* ���߶��壺
*                  ------------------------------------
*                  ģ��ܽ�             ��Ƭ���ܽ�
*                  // ��λ���� ���������� Ӳ������
*                  RD                 �鿴 zf_device_ips200.h �� IPS200_RD_PIN_PARALLEL8     �궨��
*                  WR                 �鿴 zf_device_ips200.h �� IPS200_WR_PIN_PARALLEL8     �궨��
*                  RS                 �鿴 zf_device_ips200.h �� IPS200_RS_PIN_PARALLEL8     �궨��
*                  RST                �鿴 zf_device_ips200.h �� IPS200_RST_PIN_PARALLEL8    �궨��
*                  CS                 �鿴 zf_device_ips200.h �� IPS200_CS_PIN_PARALLEL8     �궨��
*                  BL                 �鿴 zf_device_ips200.h �� IPS200_BL_PIN_PARALLEL8     �궨��
*                  D0-D7              �鿴 zf_device_ips200.h �� IPS200_Dx_PIN_PARALLEL8     �궨��
*                  VCC                3.3V��Դ
*                  GND                ��Դ��
*
*                  // SPI ���� SPI ������ Ӳ������
*                  SCL                �鿴 zf_device_ips200.h �� IPS200_SCL_PIN_SPI  �궨��
*                  SDA                �鿴 zf_device_ips200.h �� IPS200_SDA_PIN_SPI  �궨��
*                  RST                �鿴 zf_device_ips200.h �� IPS200_RST_PIN_SPI  �궨��
*                  DC                 �鿴 zf_device_ips200.h �� IPS200_DC_PIN_SPI   �궨��
*                  CS                 �鿴 zf_device_ips200.h �� IPS200_CS_PIN_SPI   �궨��
*                  BLk                �鿴 zf_device_ips200.h �� IPS200_BLk_PIN_SPI  �궨��
*                  VCC                3.3V��Դ
*                  GND                ��Դ��
*                  ���ֱ��� 320 * 240
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_ips200_h_
#define _zf_device_ips200_h_

#include "zf_common_typedef.h"

//==================================================���� IPS200 ��������================================================
#define IPS200_USE_SOFT_SPI             (1 )                                    // 改用软件 SPI (硬件引脚不匹配)
#if IPS200_USE_SOFT_SPI                                                         // ������ ��ɫ�����Ĳ�����ȷ�� ��ɫ�ҵľ���û���õ�
//====================================================���� SPI ����====================================================
// ���ʹ�õ���SPI ���ڵ�������Ļ SPI ������������ �����޸�
#define IPS200_SOFT_SPI_DELAY           (0 )                                    // ���� SPI ��ʱ����ʱ���� ��ֵԽС SPI ͨ������Խ��
#define IPS200_SCL_PIN                  (P02_8)                                 // 软件 SPI SCK 引脚 (对应硬件 02.8)
#define IPS200_SDA_PIN                  (P20_3)                                 // 软件 SPI MOSI 引脚 (对应硬件 20.3)
//====================================================���� SPI ����====================================================
#else
//====================================================Ӳ�� SPI ����====================================================
// ���ʹ�õ���SPI ���ڵ�������Ļ SPI ������������ �����޸�
#define IPS200_SPI_SPEED                (60*1000*1000)                          // Ӳ�� SPI ����
#define IPS200_SPI                      (SPI_2           )                      // Ӳ�� SPI ��
#define IPS200_SCL_PIN_SPI              (SPI2_SCLK_P15_3 )                      // Ӳ�� SPI SCK ����
#define IPS200_SDA_PIN_SPI              (SPI2_MOSI_P15_5 )                      // Ӳ�� SPI MOSI ����
#define IPS200_SDA_IN_PIN_SPI           (SPI2_MISO_P15_4 )                      // Ӳ�� SPI MISO ����  IPSû��MISO���ţ�����������Ȼ��Ҫ���壬��spi�ĳ�ʼ��ʱ��Ҫʹ��
//====================================================Ӳ�� SPI ����====================================================
#endif
// ���ʹ�õ���SPI ���ڵ�������Ļ SPI ������������ �����޸�
#define IPS200_RST_PIN_SPI              (P15_1)                                 // (SPI����)Һ����λ���Ŷ��� (对应硬件)
#define IPS200_DC_PIN_SPI               (P15_0)                                 // (SPI����)Һ������λ���Ŷ��� (对应硬件)
#define IPS200_CS_PIN_SPI               (P00_12)                                // (SPI����)Һ��Ƭѡ���Ŷ��� (对应硬件 00.12)
#define IPS200_BLk_PIN_SPI              (P33_9)                                 // (SPI����)Һ���������Ŷ��� (对应硬件 33.9)

// ���ʹ�õ��ǰ�λ���ڵ�������Ļ ���������������� �����޸�
#define IPS200_RD_PIN_PARALLEL8         (P15_3)                                 // (��λ����)Һ����ȡλ���Ŷ���
#define IPS200_WR_PIN_PARALLEL8         (P15_5)                                 // (��λ����)Һ��д��λ���Ŷ���
#define IPS200_RST_PIN_PARALLEL8        (P15_0)                                 // (��λ����)Һ����λ���Ŷ���
#define IPS200_RS_PIN_PARALLEL8         (P15_1)                                 // (��λ����)Һ������λ���Ŷ���
#define IPS200_CS_PIN_PARALLEL8         (P15_2)                                 // (��λ����)Һ��Ƭѡ���Ŷ���
#define IPS200_BL_PIN_PARALLEL8         (P15_4)                                 // (��λ����)Һ���������Ŷ���
//���������������� �����޸� ��������Ļ�ǰ�λ���� ����������õõ�
//D0-D3�ĸ��������ű������� ����C0-C3,C1-C4�ȵȣ�
//D4-D7�ĸ��������ű������� ����B0-B3,B1-B4�ȵȡ�
//�������ӵ���ͬ�˿ڵ���˼������Ļ��D0-D3��C1-C4���ӣ�D4-D7��B2-B5���ӡ�
//�л����ź�ע���޸�IPS200_DATA_PORT1��IPS200_DATA_PORT2�궨��
#define IPS200_D0_PIN_PARALLEL8         (P11_9 )                                // (��λ����)Һ����������D0
#define IPS200_D1_PIN_PARALLEL8         (P11_10)                                // (��λ����)Һ����������D1
#define IPS200_D2_PIN_PARALLEL8         (P11_11)                                // (��λ����)Һ����������D2
#define IPS200_D3_PIN_PARALLEL8         (P11_12)                                // (��λ����)Һ����������D3
#define IPS200_D4_PIN_PARALLEL8         (P13_0 )                                // (��λ����)Һ����������D4
#define IPS200_D5_PIN_PARALLEL8         (P13_1 )                                // (��λ����)Һ����������D5
#define IPS200_D6_PIN_PARALLEL8         (P13_2 )                                // (��λ����)Һ����������D6
#define IPS200_D7_PIN_PARALLEL8         (P13_3 )                                // (��λ����)Һ����������D7

#define IPS200_DEFAULT_DISPLAY_DIR      (IPS200_PORTAIT)                        // Ĭ�ϵ���ʾ����
#define IPS200_DEFAULT_PENCOLOR         (RGB565_RED    )                        // Ĭ�ϵĻ�����ɫ
#define IPS200_DEFAULT_BGCOLOR          (RGB565_WHITE  )                        // Ĭ�ϵı�����ɫ
#define IPS200_DEFAULT_DISPLAY_FONT     (IPS200_8X16_FONT)                      // Ĭ�ϵ�����ģʽ

//�������ݶ˿�����PORT���л����ź���ظ�����������PORT���и���   ����ʹ��������˿ڽ������  ��˶���������������ʼ���
#define IPS200_DATA_PORT1               (4)       //0:P00�˿�  1��P01�˿�  2��P02�˿�  3��P10�˿�  4��P11�˿�  5��P12�˿�  6��P13�˿�  7��P14�˿�  8��P15�˿�  9��P20�˿�  10��P21�˿�  11��P22�˿�  12��P23�˿�  13��P32�˿�  14��P33�˿�
#define IPS200_DATAPORT1                (get_port_out_addr(IPS200_DATA_PORT1))
#define DATA_START_NUM1                 (IPS200_D0_PIN_PARALLEL8&0x1f)          // �����������ŵ���ʼ���
#define IPS200_DATA_PORT2               (6)       //0:P00�˿�  1��P01�˿�  2��P02�˿�  3��P10�˿�  4��P11�˿�  5��P12�˿�  6��P13�˿�  7��P14�˿�  8��P15�˿�  9��P20�˿�  10��P21�˿�  11��P22�˿�  12��P23�˿�  13��P32�˿�  14��P33�˿�
#define IPS200_DATAPORT2                (get_port_out_addr(IPS200_DATA_PORT2))
#define DATA_START_NUM2                 (IPS200_D4_PIN_PARALLEL8&0x1f)          // �����������ŵ���ʼ���

// �������
#define IPS200_RD(x)                    ((x) ? (gpio_high(IPS200_RD_PIN_PARALLEL8)) : (gpio_low(IPS200_RD_PIN_PARALLEL8)))
#define IPS200_WR(x)                    ((x) ? (gpio_high(IPS200_WR_PIN_PARALLEL8)) : (gpio_low(IPS200_WR_PIN_PARALLEL8)))
#define IPS200_RST(x)                   ((x) ? (gpio_high(ips_rst_pin))             : (gpio_low(ips_rst_pin)))
#define IPS200_BL(x)                    ((x) ? (gpio_high(ips_bl_pin))              : (gpio_low(ips_bl_pin)))
#define IPS200_RS(x)                    ((x) ? (gpio_high(IPS200_RS_PIN_PARALLEL8)) : (gpio_low(IPS200_RS_PIN_PARALLEL8)))
#define IPS200_DC(x)                    ((x) ? (gpio_high(IPS200_DC_PIN_SPI))       : (gpio_low(IPS200_DC_PIN_SPI)))
#define IPS200_CS(x)                    ((x) ? (gpio_high(IPS200_CS_PIN_SPI))       : (gpio_low(IPS200_CS_PIN_SPI)))
//==================================================���� IPS200 ��������================================================


//==================================================���� IPS200 �����ṹ��===============================================
typedef enum
{
    IPS200_TYPE_SPI,                                                            // SPI ����
    IPS200_TYPE_PARALLEL8,                                                      // ��������
}ips200_type_enum;

typedef enum
{
    IPS200_PORTAIT                      = 0,                                    // ����ģʽ
    IPS200_PORTAIT_180                  = 1,                                    // ����ģʽ  ��ת180
    IPS200_CROSSWISE                    = 2,                                    // ����ģʽ
    IPS200_CROSSWISE_180                = 3,                                    // ����ģʽ  ��ת180
}ips200_dir_enum;

typedef enum
{
    IPS200_6X8_FONT                     = 0,                                    // 6x8      ����
    IPS200_8X16_FONT                    = 1,                                    // 8x16     ����
    IPS200_16X16_FONT                   = 2,                                    // 16x16    ���� Ŀǰ��֧��
}ips200_font_size_enum;
extern  uint16  ips200_width_max;
extern  uint16  ips200_height_max;
//==================================================���� IPS200 �����ṹ��===============================================


//==================================================���� IPS200 ��������================================================
void    ips200_clear                    (void);                                                                                // IPS200 ��������
void    ips200_full                     (const uint16 color);                                                                  // IPS200 ��Ļ��亯��
void    ips200_set_dir                  (ips200_dir_enum dir);                                                                 // IPS200 ������ʾ����
void    ips200_set_font                 (ips200_font_size_enum font);                                                          // IPS200 ������ʾ����
void    ips200_set_color                (const uint16 pen, const uint16 bgcolor);                                              // IPS200 ������ʾ��ɫ
void    ips200_draw_point               (uint16 x, uint16 y, const uint16 color);                                              // IPS200 ���㺯��
void    ips200_draw_line                (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);      // IPS200 ���ߺ���

void    ips200_show_char                (uint16 x, uint16 y, const char dat);
void    ips200_show_string              (uint16 x, uint16 y, const char dat[]);
void    ips200_show_int                 (uint16 x, uint16 y, const int32 dat, uint8 num);
void    ips200_show_uint                (uint16 x, uint16 y, const uint32 dat, uint8 num);
void    ips200_show_float               (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);

void    ips200_show_binary_image        (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);                      // IPS200 ��ʾ��ֵͼ�� ����ÿ�˸������һ���ֽ�����
void    ips200_show_gray_image          (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold);     // IPS200 ��ʾ 8bit �Ҷ�ͼ�� ����ֵ����ֵ
void    ips200_show_rgb565_image        (uint16 x, uint16 y, const uint16 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 color_mode);   // IPS200 ��ʾ RGB565 ��ɫͼ��

void    ips200_show_wave                (uint16 x, uint16 y, const uint16 *wave, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max);                // IPS200 ��ʾ����
void    ips200_show_chinese             (uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number, const uint16 color);                                 // IPS200 ������ʾ

void    ips200_init                     (ips200_type_enum type_select);                                                         // 2�� IPSҺ����ʼ��
//==================================================���� IPS200 ��������================================================


//==================================================���� IPS200 ��չ����================================================
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾС���ͼ��
// ����˵��     p               ͼ������ָ��
// ����˵��     width           ͼ��ʵ�ʿ���
// ����˵��     height          ͼ��ʵ�ʸ߶�
// ���ز���     void
// ʹ��ʾ��     ips200_displayimage7725(ov7725_image_binary[0], OV7725_W, OV7725_H);
// ��ע��Ϣ     ��չ��һ����ʾ������Ĭ�������ţ�����Ļ������ʼ�㿪ʼ��ʾ
//-------------------------------------------------------------------------------------------------------------------
#define ips200_displayimage7725(p, width, height)       (ips200_show_binary_image(0, 0, (p), OV7725_W, OV7725_H, (width), (height)))
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ�����ͼ��
// ����˵��     p               ͼ������ָ��
// ����˵��     width           ͼ��ʵ�ʿ���
// ����˵��     height          ͼ��ʵ�ʸ߶�
// ���ز���     void
// ʹ��ʾ��     ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
// ��ע��Ϣ     ��չ��һ����ʾ������Ĭ�������ţ�����Ļ������ʼ�㿪ʼ��ʾ
//-------------------------------------------------------------------------------------------------------------------
#define ips200_displayimage03x(p, width, height)        (ips200_show_gray_image(0, 0, (p), MT9V03X_W, MT9V03X_H, (width), (height), 0))
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ��ͫͼ��
// ����˵��     p               ͼ������ָ��
// ����˵��     width           ͼ��ʵ�ʿ���
// ����˵��     height          ͼ��ʵ�ʸ߶�
// ���ز���     void
// ʹ��ʾ��     ips200_displayimage8660(scc8660_image[0], SCC8660_W, SCC8660_H);
// ��ע��Ϣ     ��չ��һ����ʾ������Ĭ�������ţ�����Ļ������ʼ�㿪ʼ��ʾ
//-------------------------------------------------------------------------------------------------------------------
#define ips200_displayimage8660(p, width, height)       (ips200_show_rgb565_image(0, 0, (p), SCC8660_W, SCC8660_H, (width), (height), 1))
//==================================================���� IPS200 ��չ����================================================


#endif

