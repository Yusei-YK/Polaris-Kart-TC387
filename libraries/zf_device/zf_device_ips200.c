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
* 2023-10-07       pudding            SPI������ �޸����ݴ��䷽ʽ����������һ������
********************************************************************************************************************/
/********************************************************************************************************************
* ���߶��壺
*                  ------------------------------------
*                  ģ��ܽ�             ��Ƭ���ܽ�
*                  // ˫������ ���������� Ӳ������
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
*                  // �������� SPI ������ Ӳ������
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

#include "zf_common_debug.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_spi.h"
#include "zf_driver_spi.h"
#include "zf_device_ips200.h"

       uint16                   ips200_width_max    = 240;
       uint16                   ips200_height_max   = 320;
static uint16                   ips200_pencolor     = IPS200_DEFAULT_PENCOLOR;          // ������ɫ(����ɫ)
static uint16                   ips200_bgcolor      = IPS200_DEFAULT_BGCOLOR;           // ������ɫ
static ips200_type_enum         ips200_display_type     = IPS200_TYPE_SPI;
static ips200_dir_enum          ips200_display_dir  = IPS200_DEFAULT_DISPLAY_DIR;       // ��ʾ����
static ips200_font_size_enum    ips200_display_font = IPS200_DEFAULT_DISPLAY_FONT;      // ��ʾ��������

static gpio_pin_enum            ips_rst_pin         = IPS200_RST_PIN_SPI;               // ���帴λ��������
static gpio_pin_enum            ips_bl_pin          = IPS200_BLk_PIN_SPI;               // ���屳����������
static gpio_pin_enum            ips_cs_pin          = IPS200_CS_PIN_SPI;                // ����Ƭѡ��������

#if IPS200_USE_SOFT_SPI
static soft_spi_info_struct                 ips200_spi;
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ���� SPI д 8bit ����
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_8bit_data_spi(command);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_8bit_data_spi(data)                (soft_spi_write_8bit(&ips200_spi, (data)))

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ���� SPI д 8bit ��������
// ����˵��     *data           ����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     ips200_write_8bit_data_spi_array(data, len);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_8bit_data_spi_array(data, len)     (soft_spi_write_8bit_array(&ips200_spi, (data), (len)))

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ���� SPI д 16bit ����
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_16bit_data_spi(dat);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_16bit_data_spi(data)               (soft_spi_write_16bit(&ips200_spi, (data)))

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ���� SPI д 16bit ��������
// ����˵��     *data           ����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     ips200_write_16bit_data_spi_array(data, len);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_16bit_data_spi_array(data, len)    (soft_spi_write_16bit_array(&ips200_spi, (data), (len)))
#else
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 SPI д 8bit ����
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_8bit_data_spi(command);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_8bit_data_spi(data)                (spi_write_8bit(IPS200_SPI, (data)))

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 SPI д 8bit ��������
// ����˵��     *data           ����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     ips200_write_8bit_data_spi_array(data, len);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_8bit_data_spi_array(data, len)     (spi_write_8bit_array(IPS200_SPI, (data), (len)))

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 SPI д 16bit ����
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_16bit_data_spi(dat);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_16bit_data_spi(data)               (spi_write_16bit(IPS200_SPI, (data)))
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 SPI д 16bit ��������
// ����˵��     *data           ����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     ips200_write_16bit_data_spi_array(data, len);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define ips200_write_16bit_data_spi_array(data, len)    (spi_write_16bit_array(IPS200_SPI, (data), (len)))
#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ����д����
// ����˵��     data           ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_data(data);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void ips200_write_data(uint8 dat)
{
    IPS200_DATAPORT1 = (dat << DATA_START_NUM1) | (IPS200_DATAPORT1 & ~((uint32)(0x0F << DATA_START_NUM1)) );
    IPS200_DATAPORT2 = ((dat>>4) << DATA_START_NUM2) | (IPS200_DATAPORT2 & ~((uint32)(0x0F << DATA_START_NUM2)) );
}


//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 д����
// ����˵��     command         ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_command(0x2a);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void ips200_write_command (const uint8 command)
{
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_DC(0);
        ips200_write_8bit_data_spi(command);
        IPS200_DC(1);
    }
    else
    {
        IPS200_CS(0);
        IPS200_RS(0);
        IPS200_RD(1);
        IPS200_WR(0);
        ips200_write_data(command);
        IPS200_WR(1);
        IPS200_CS(1);
        IPS200_RS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��Һ����д 8bit ����
// ����˵��     dat             ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_8bit_data(0x0C);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void ips200_write_8bit_data (const uint8 dat)
{
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        ips200_write_8bit_data_spi(dat);
    }
    else
    {
        IPS200_CS(0);
        IPS200_RD(1);
        IPS200_WR(0);
        ips200_write_data(dat);
        IPS200_WR(1);
        IPS200_CS(1);
    }
}
//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��Һ����д 8bit ����
// ����˵��     dat             ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_8bit_data(0x0C);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void ips200_write_8bit_data_array (const uint8 *dat, uint32 len)
{
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        ips200_write_8bit_data_spi_array(dat, len);
    }
    else
    {
        IPS200_CS(0);
        IPS200_RD(1);
        while(len --)
        {
            IPS200_WR(0);
            ips200_write_data((uint8)*dat);
            IPS200_WR(1);
            dat ++;
        }
        IPS200_CS(1);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��Һ����д 16bit ����
// ����˵��     dat             ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_16bit_data(x1);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
void ips200_write_16bit_data (const uint16 dat)
{
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        ips200_write_16bit_data_spi(dat);
    }
    else
    {
        IPS200_CS(0);
        IPS200_RD(1);
        IPS200_WR(0);
        ips200_write_data((uint8)(dat >> 8));
        IPS200_WR(1);
        IPS200_WR(0);
        ips200_write_data((uint8)(dat & 0x00FF));
        IPS200_WR(1);
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��Һ����д 16bit ����
// ����˵��     dat             ����
// ���ز���     void
// ʹ��ʾ��     ips200_write_16bit_data(x1);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void ips200_write_16bit_data_array (const uint16 *dat, uint32 len)
{
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        ips200_write_16bit_data_spi_array(dat, len);
    }
    else
    {
        IPS200_CS(0);
        IPS200_RD(1);
        while(len --)
        {
            IPS200_WR(0);
            ips200_write_data((uint8)(*dat >> 8));
            IPS200_WR(1);
            IPS200_WR(0);
            ips200_write_data((uint8)(*dat & 0xFF));
            IPS200_WR(1);
            dat ++;
        }
        IPS200_CS(1);
    }
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ����
// ����˵��     x1              ��ʼx������
// ����˵��     y1              ��ʼy������
// ����˵��     x2              ����x������
// ����˵��     y2              ����y������
// ���ز���     void
// ʹ��ʾ��     ips200_set_region(0, 0, ips200_width_max - 1, ips200_height_max - 1);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void ips200_set_region (uint16 x1, uint16 y1, uint16 x2, uint16 y2)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
    zf_assert(x1 < ips200_width_max);
    zf_assert(y1 < ips200_height_max);
    zf_assert(x2 < ips200_width_max);
    zf_assert(y2 < ips200_height_max);

    ips200_write_command(0x2a);
    ips200_write_16bit_data(x1);
    ips200_write_16bit_data(x2);
    
    ips200_write_command(0x2b);
    ips200_write_16bit_data(y1);
    ips200_write_16bit_data(y2);
    
    ips200_write_command(0x2c);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾDEBUG��Ϣ��ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     ips200_debug_init();
// ��ע��Ϣ     �ڲ�ʹ�� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void ips200_debug_init (void)
{
    debug_output_struct info;
    debug_output_struct_init(&info);

    info.type_index = 1;
    info.display_x_max = ips200_width_max;
    info.display_y_max = ips200_height_max;

    switch(ips200_display_font)
    {
        case IPS200_6X8_FONT:
        {
            info.font_x_size = 6;
            info.font_y_size = 8;
        }break;
        case IPS200_8X16_FONT:
        {
            info.font_x_size = 8;
            info.font_y_size = 16;
        }break;
        case IPS200_16X16_FONT:
        {
            // �ݲ�֧��
        }break;
    }
    info.output_screen = ips200_show_string;
    info.output_screen_clear = ips200_clear;
        
    debug_output_init(&info);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     ips200_clear();
// ��ע��Ϣ     ����Ļ��ճɱ�����ɫ
//-------------------------------------------------------------------------------------------------------------------
void ips200_clear (void)
{
    uint16 color_buffer[ips200_width_max];
    uint16 i = 0, j = 0;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(0, 0, ips200_width_max - 1, ips200_height_max - 1);
    for(i = 0; i < ips200_width_max; i ++)
    {
        color_buffer[i] = ips200_bgcolor;
    }
    for (j = 0; j < ips200_height_max; j ++)
    {
        ips200_write_16bit_data_array(color_buffer, ips200_width_max);
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��Ļ��亯��
// ����˵��     color           ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ���ز���     void
// ʹ��ʾ��     ips200_full(RGB565_BLACK);
// ��ע��Ϣ     ����Ļ����ָ����ɫ
//-------------------------------------------------------------------------------------------------------------------
void ips200_full (const uint16 color)
{
    uint16 color_buffer[ips200_width_max];
    uint16 i = 0, j = 0;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(0, 0, ips200_width_max - 1, ips200_height_max - 1);
    for(i = 0; i < ips200_width_max; i ++)
    {
        color_buffer[i] = color;
    }
    for (j = 0; j < ips200_height_max; j ++)
    {
        ips200_write_16bit_data_array(color_buffer, ips200_width_max);
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ����
// ����˵��     dir             ��ʾ����  ���� zf_device_ips200.h �� ips200_dir_enum ö���嶨��
// ���ز���     void
// ʹ��ʾ��     ips200_set_dir(IPS200_PORTAIT);
// ��ע��Ϣ     �������ֻ���ڳ�ʼ����Ļ֮ǰ���ò���Ч
//-------------------------------------------------------------------------------------------------------------------
void ips200_set_dir (ips200_dir_enum dir)
{
    ips200_display_dir = dir;
    switch(ips200_display_dir)
    {
        case IPS200_PORTAIT:
        case IPS200_PORTAIT_180:
        {
            ips200_width_max = 240;
            ips200_height_max = 320;
        }break;
        case IPS200_CROSSWISE:
        case IPS200_CROSSWISE_180:
        {
            ips200_width_max = 320;
            ips200_height_max = 240;
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ����
// ����˵��     dir             ��ʾ����  ���� zf_device_ips200.h �� ips200_font_size_enum ö���嶨��
// ���ز���     void
// ʹ��ʾ��     ips200_set_font(IPS200_8x16_FONT);
// ��ע��Ϣ     ���������ʱ�������� ���ú���Ч ������ʾ�����µ������С
//-------------------------------------------------------------------------------------------------------------------
void ips200_set_font (ips200_font_size_enum font)
{
    ips200_display_font = font;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ��ɫ
// ����˵��     pen             ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ����˵��     bgcolor         ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ���ز���     void
// ʹ��ʾ��     ips200_set_color(RGB565_RED, RGB565_GRAY);
// ��ע��Ϣ     ������ɫ�ͱ�����ɫҲ������ʱ�������� ���ú���Ч
//-------------------------------------------------------------------------------------------------------------------
void ips200_set_color (const uint16 pen, const uint16 bgcolor)
{
    ips200_pencolor = pen;
    ips200_bgcolor = bgcolor;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ����
// ����˵��     x               ����x�������� [0, ips200_width_max-1]
// ����˵��     y               ����y�������� [0, ips200_height_max-1]
// ����˵��     color           ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ���ز���     void
// ʹ��ʾ��     ips200_draw_point(0, 0, RGB565_RED);            //����0,0��һ����ɫ�ĵ�
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_draw_point (uint16 x, uint16 y, const uint16 color)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(x, y, x, y);
    ips200_write_16bit_data(color);
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ����
// ����˵��     x_start         ����x�������� [0, ips200_width_max-1]
// ����˵��     y_start         ����y�������� [0, ips200_height_max-1]
// ����˵��     x_end           ����x������յ� [0, ips200_width_max-1]
// ����˵��     y_end           ����y������յ� [0, ips200_height_max-1]
// ����˵��     color           ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ���ز���     void
// ʹ��ʾ��     ips200_draw_line(0, 0, 10, 10, RGB565_RED);     // ���� 0,0 �� 10,10 ��һ����ɫ����
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_draw_line (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x_start < ips200_width_max);
    zf_assert(y_start < ips200_height_max);
    zf_assert(x_end < ips200_width_max);
    zf_assert(y_end < ips200_height_max);

    int16 x_dir = (x_start < x_end ? 1 : -1);
    int16 y_dir = (y_start < y_end ? 1 : -1);
    float temp_rate = 0;
    float temp_b = 0;

    do
    {
        if(x_start != x_end)
        {
            temp_rate = (float)(y_start - y_end) / (float)(x_start - x_end);
            temp_b = (float)y_start - (float)x_start * temp_rate;
        }
        else
        {
            while(y_start != y_end)
            {
                ips200_draw_point(x_start, y_start, color);
                y_start += y_dir;
            }
            ips200_draw_point(x_start, y_start, color);
            break;
        }
        if(func_abs(y_start - y_end) > func_abs(x_start - x_end))
        {
            while(y_start != y_end)
            {
                ips200_draw_point(x_start, y_start, color);
                y_start += y_dir;
                x_start = (int16)(((float)y_start - temp_b) / temp_rate);
            }
            ips200_draw_point(x_start, y_start, color);
        }
        else
        {
            while(x_start != x_end)
            {
                ips200_draw_point(x_start, y_start, color);
                x_start += x_dir;
                y_start = (int16)((float)x_start * temp_rate + temp_b);
            }
            ips200_draw_point(x_start, y_start, color);
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ�ַ�
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     dat             ��Ҫ��ʾ���ַ�
// ���ز���     void
// ʹ��ʾ��     ips200_show_char(0, 0, 'x');                     // ����0,0дһ���ַ�x
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_char (uint16 x, uint16 y, const char dat)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);

    uint8 i = 0, j = 0;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    switch(ips200_display_font)
    {
        case IPS200_6X8_FONT:
        {
            uint16 display_buffer[6*8];
            ips200_set_region(x, y, x + 5, y + 7);
            for(i = 0; 6 > i; i ++)
            {
                // �� 32 ��Ϊ��ȡģ�Ǵӿո�ʼȡ�� �ո��� ascii ������� 32
                uint8 temp_top = ascii_font_6x8[dat - 32][i];
                for(j = 0; 8 > j; j ++)
                {
                    if(temp_top & 0x01)
                    {
                        display_buffer[i + j * 6] = (ips200_pencolor);
                    }
                    else
                    {
                        display_buffer[i + j * 6] = (ips200_bgcolor);
                    }
                    temp_top >>= 1;
                }
            }
            ips200_write_16bit_data_array(display_buffer, 6*8);
        }break;
        case IPS200_8X16_FONT:
        {
            uint16 display_buffer[8*16];
            ips200_set_region(x, y, x + 7, y + 15);
            for(i = 0; 8 > i; i ++)
            {
                uint8 temp_top = ascii_font_8x16[dat - 32][i];
                uint8 temp_bottom = ascii_font_8x16[dat - 32][i + 8];
                for(j = 0; 8 > j; j ++)
                {
                    if(temp_top & 0x01)
                    {
                        display_buffer[i + j * 8] = (ips200_pencolor);
                    }
                    else
                    {
                        display_buffer[i + j * 8] = (ips200_bgcolor);
                    }
                    temp_top >>= 1;
                }
                for(j = 0; 8 > j; j ++)
                {
                    if(temp_bottom & 0x01)
                    {
                        display_buffer[i + j * 8 + 4 * 16] = (ips200_pencolor);
                    }
                    else
                    {
                        display_buffer[i + j * 8 + 4 * 16] = (ips200_bgcolor);
                    }
                    temp_bottom >>= 1;
                }
            }
            ips200_write_16bit_data_array(display_buffer, 8 * 16);
        }break;
        case IPS200_16X16_FONT:
        {
            // �ݲ�֧��
        }break;
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ�ַ���
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     dat             ��Ҫ��ʾ���ַ���
// ���ز���     void
// ʹ��ʾ��     ips200_show_string(0, 0, "seekfree");
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_string (uint16 x, uint16 y, const char dat[])
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    
    uint16 j = 0;
    while('\0' != dat[j])
    {
        switch(ips200_display_font)
        {
            case IPS200_6X8_FONT:   ips200_show_char(x + 6 * j, y, dat[j]); break;
            case IPS200_8X16_FONT:  ips200_show_char(x + 8 * j, y, dat[j]); break;
            case IPS200_16X16_FONT: break;                                      // �ݲ�֧��
        }
        j ++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ32λ�з��� (ȥ������������Ч��0)
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     dat             ��Ҫ��ʾ�ı��� �������� int32
// ����˵��     num             ��Ҫ��ʾ��λ�� ���10λ  ������������
// ���ز���     void
// ʹ��ʾ��     ips200_show_int(0, 0, x, 3);                    // x ����Ϊ int32 int16 int8 ����
// ��ע��Ϣ     ��������ʾһ�� ��-����
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_int (uint16 x, uint16 y, const int32 dat, uint8 num)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(0 < num);
    zf_assert(10 >= num);

    int32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num+1);

    // ��������������ʾ 123 ��ʾ 2 λ��Ӧ����ʾ 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_int_to_str(data_buffer, dat_temp);
    ips200_show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ32λ�޷��� (ȥ������������Ч��0)
// ����˵��     x               ����x�������� ������Χ [0, ips114_x_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips114_y_max-1]
// ����˵��     dat             ��Ҫ��ʾ�ı��� �������� uint32
// ����˵��     num             ��Ҫ��ʾ��λ�� ���10λ  ������������
// ���ز���     void
// ʹ��ʾ��     ips200_show_uint(0, 0, x, 3);                   // x ����Ϊ uint32 uint16 uint8 ����
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_uint (uint16 x, uint16 y, const uint32 dat, uint8 num)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(0 < num);
    zf_assert(10 >= num);

    uint32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12];
    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num);

    // ��������������ʾ 123 ��ʾ 2 λ��Ӧ����ʾ 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_uint_to_str(data_buffer, dat_temp);
    ips200_show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ������(ȥ������������Ч��0)
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     dat             ��Ҫ��ʾ�ı��� �������� double
// ����˵��     num             ����λ��ʾ����   ���8λ  
// ����˵��     pointnum        С��λ��ʾ����   ���6λ
// ���ز���     void
// ʹ��ʾ��     ips200_show_float(0, 0, x, 2, 3);               // ��ʾ������   ������ʾ2λ   С����ʾ��λ
// ��ע��Ϣ     �ر�ע�⵱����С��������ʾ��ֵ����д���ֵ��һ����ʱ��
//              ���������ڸ��������ȶ�ʧ���⵼�µģ��Ⲣ������ʾ���������⣬
//              �й���������飬�����аٶ�ѧϰ   ���������ȶ�ʧ���⡣
//              ��������ʾһ�� ��-����
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_float (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(0 < num);
    zf_assert(8 >= num);
    zf_assert(0 < pointnum);
    zf_assert(6 >= pointnum);

    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17];
    memset(data_buffer, 0, 17);
    memset(data_buffer, ' ', num+pointnum+2);

    // ��������������ʾ 123 ��ʾ 2 λ��Ӧ����ʾ 23
    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    ips200_show_string(x, y, data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ��ֵͼ�� ����ÿ�˸������һ���ֽ�����
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     *image          ͼ������ָ��
// ����˵��     width           ͼ��ʵ�ʿ���
// ����˵��     height          ͼ��ʵ�ʸ߶�
// ����˵��     dis_width       ͼ����ʾ���� ������Χ [0, ips200_width_max]
// ����˵��     dis_height      ͼ����ʾ�߶� ������Χ [0, ips200_height_max]
// ���ز���     void
// ʹ��ʾ��     ips200_show_binary_image(0, 0, ov7725_image_binary[0], OV7725_W, OV7725_H, OV7725_W, OV7725_H);
// ��ע��Ϣ     ������ʾС����δ��ѹ��ѹ����ֵ��ͼ��
//              �����������������ֱ����ʾ������δѹ���Ķ�ֵ��ͼ��
//              �����������������ֱ����ʾ������δѹ���Ķ�ֵ��ͼ��
//              �����������������ֱ����ʾ������δѹ���Ķ�ֵ��ͼ��
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_binary_image (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(NULL != image);

    uint32 i = 0, j = 0;
    uint8 temp = 0;
    uint32 width_index = 0;
    uint16 data_buffer[dis_width];
    const uint8 *image_temp;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(x, y, x + dis_width - 1, y + dis_height - 1);             // ������ʾ����

    for(j = 0; j < dis_height; j ++)
    {
        image_temp = image + j * height / dis_height * width / 8;               // ֱ�Ӷ� image ������ Hardfault ��ʱ��֪��Ϊʲô
        for(i = 0; i < dis_width; i ++)
        {
            width_index = i * width / dis_width;
            temp = *(image_temp + width_index / 8);                             // ��ȡ���ص�
            if(0x80 & (temp << (width_index % 8)))
            {
                data_buffer[i] = (RGB565_WHITE);
            }
            else
            {
                data_buffer[i] = (RGB565_BLACK);
            }
        }
        ips200_write_16bit_data_array(data_buffer, dis_width);
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ 8bit �Ҷ�ͼ�� ����ֵ����ֵ
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     *image          ͼ������ָ��
// ����˵��     width           ͼ��ʵ�ʿ���
// ����˵��     height          ͼ��ʵ�ʸ߶�
// ����˵��     dis_width       ͼ����ʾ���� ������Χ [0, ips200_width_max]
// ����˵��     dis_height      ͼ����ʾ�߶� ������Χ [0, ips200_height_max]
// ����˵��     threshold       ��ֵ����ʾ��ֵ 0-��������ֵ��
// ���ز���     void
// ʹ��ʾ��     ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
// ��ע��Ϣ     ������ʾ������ͼ��
//              ���Ҫ��ʾ��ֵ��ͼ�� ֱ���޸����һ������Ϊ��Ҫ�Ķ�ֵ����ֵ����
//              ���Ҫ��ʾ��ֵ��ͼ�� ֱ���޸����һ������Ϊ��Ҫ�Ķ�ֵ����ֵ����
//              ���Ҫ��ʾ��ֵ��ͼ�� ֱ���޸����һ������Ϊ��Ҫ�Ķ�ֵ����ֵ����
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_gray_image (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(NULL != image);

    uint32 i = 0, j = 0;
    uint16 color = 0,temp = 0;
    uint16 data_buffer[dis_width];
    const uint8 *image_temp;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(x, y, x + dis_width - 1, y + dis_height - 1);             // ������ʾ����

    for(j = 0; j < dis_height; j ++)
    {
        image_temp = image + j * height / dis_height * width;                   // ֱ�Ӷ� image ������ Hardfault ��ʱ��֪��Ϊʲô
        for(i = 0; i < dis_width; i ++)
        {
            temp = *(image_temp + i * width / dis_width);                       // ��ȡ���ص�
            if(threshold == 0)
            {
                color = (0x001f & ((temp) >> 3)) << 11;
                color = color | (((0x003f) & ((temp) >> 2)) << 5);
                color = color | (0x001f & ((temp) >> 3));
                data_buffer[i] = (color);
            }
            else if(temp < threshold)
            {
                data_buffer[i] = (RGB565_BLACK);
            }
            else
            {
                data_buffer[i] = (RGB565_WHITE);
            }
        }
        ips200_write_16bit_data_array(data_buffer, dis_width);
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ RGB565 ��ɫͼ��
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     *image          ͼ������ָ��
// ����˵��     width           ͼ��ʵ�ʿ���
// ����˵��     height          ͼ��ʵ�ʸ߶�
// ����˵��     dis_width       ͼ����ʾ���� ������Χ [0, ips200_width_max]
// ����˵��     dis_height      ͼ����ʾ�߶� ������Χ [0, ips200_height_max]
// ����˵��     color_mode      ɫ��ģʽ 0-��λ��ǰ 1-��λ��ǰ
// ���ز���     void
// ʹ��ʾ��     ips200_show_rgb565_image(0, 0, scc8660_image[0], SCC8660_W, SCC8660_H, SCC8660_W, SCC8660_H, 1);
// ��ע��Ϣ     ������ʾ��ͫ�� RGB565 ��ͼ��
//              ���Ҫ��ʾ��λ��ǰ������ RGB565 ͼ�� �޸����һ����������
//              ���Ҫ��ʾ��λ��ǰ������ RGB565 ͼ�� �޸����һ����������
//              ���Ҫ��ʾ��λ��ǰ������ RGB565 ͼ�� �޸����һ����������
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_rgb565_image (uint16 x, uint16 y, const uint16 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 color_mode)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(NULL != image);

    uint32 i = 0, j = 0;
    uint16 data_buffer[dis_width];
    const uint16 *image_temp;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(x, y, x + dis_width - 1, y + dis_height - 1);             // ������ʾ����

    for(j = 0; j < dis_height; j ++)
    {
        image_temp = image + j * height / dis_height * width;                   // ֱ�Ӷ� image ������ Hardfault ��ʱ��֪��Ϊʲô
        for(i = 0; i < dis_width; i ++)
        {
            data_buffer[i] = *(image_temp + i * width / dis_width);             // ��ȡ���ص�
        }
        if(color_mode)
        {
            ips200_write_8bit_data_array((uint8 *)data_buffer, dis_width * 2);
        }
        else
        {
            ips200_write_16bit_data_array(data_buffer, dis_width);
        }
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IPS200 ��ʾ����
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     *wave           ��������ָ��
// ����˵��     width           ����ʵ�ʿ���
// ����˵��     value_max       ����ʵ�����ֵ
// ����˵��     dis_width       ������ʾ���� ������Χ [0, ips200_width_max]
// ����˵��     dis_value_max   ������ʾ���ֵ ������Χ [0, ips200_height_max]
// ���ز���     void
// ʹ��ʾ��     ips200_show_wave(0, 0, data, 128, 64, 64, 32);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_wave (uint16 x, uint16 y, const uint16 *wave, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(NULL != wave);

    uint32 i = 0, j = 0;
    uint32 width_index = 0, value_max_index = 0;
    uint16 data_buffer[dis_width];

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(x, y, x + dis_width - 1, y + dis_value_max - 1);          // ������ʾ����
    for(j = 0; j < dis_value_max; j ++)
    {
        for(i = 0; i < dis_width; i ++)
        {
            data_buffer[i] = (ips200_bgcolor); 
        }
        ips200_write_16bit_data_array(data_buffer, dis_width);
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }

    for(i = 0; i < dis_width; i ++)
    {
        width_index = i * width / dis_width;
        value_max_index = *(wave + width_index) * (dis_value_max - 1) / value_max;
        ips200_draw_point((uint16)(i + x), (uint16)((dis_value_max - 1) - value_max_index + y), ips200_pencolor);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ
// ����˵��     x               ����x�������� ������Χ [0, ips200_width_max-1]
// ����˵��     y               ����y�������� ������Χ [0, ips200_height_max-1]
// ����˵��     size            ȡģ��ʱ�����õĺ��������С Ҳ����һ������ռ�õĵ��󳤿�Ϊ���ٸ��� ȡģ��ʱ����Ҫ������һ����
// ����˵��     *chinese_buffer ��Ҫ��ʾ�ĺ�������
// ����˵��     number          ��Ҫ��ʾ����λ
// ����˵��     color           ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ���ز���     void
// ʹ��ʾ��     ips200_show_chinese(0, 0, 16, chinese_test[0], 4, RGB565_RED);//��ʾfont�ļ������ ʾ��
// ��ע��Ϣ     ʹ��PCtoLCD2002����ȡģ           ���롢����ʽ��˳��   16*16
//-------------------------------------------------------------------------------------------------------------------
void ips200_show_chinese (uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number, const uint16 color)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    zf_assert(x < ips200_width_max);
    zf_assert(y < ips200_height_max);
    zf_assert(NULL != chinese_buffer);

    int i = 0, j = 0, k = 0; 
    uint8 temp = 0, temp1 = 0, temp2 = 0;
    const uint8 *p_data = chinese_buffer;
    
    temp2 = size / 8;
    
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(x, y, number * size - 1 + x, y + size - 1);
    
    for(i = 0; i < size; i ++)
    {
        temp1 = number;
        p_data = chinese_buffer + i * temp2;
        while(temp1 --)
        {
            for(k = 0; k < temp2; k ++)
            {
                for(j = 8; 0 < j; j --)
                {
                    temp = (*p_data >> (j - 1)) & 0x01;
                    if(temp)
                    {
                        ips200_write_16bit_data(color);
                    }
                    else
                    {
                        ips200_write_16bit_data(ips200_bgcolor);
                    }
                }
                p_data ++;
            }
            p_data = p_data - temp2 + temp2 * size;
        }   
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     2�� IPSҺ����ʼ��
// ����˵��     type_select     �������ӿ����� IPS200_TYPE_SPI Ϊ SPI �ӿڴ��������� IPS200_TYPE_PARALLEL8 Ϊ 8080 Э���λ����������
// ���ز���     void
// ʹ��ʾ��     ips200_init(IPS200_TYPE_PARALLEL8);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void ips200_init (ips200_type_enum type_select)
{
    if(IPS200_TYPE_SPI == type_select)
    {
        ips200_display_type = IPS200_TYPE_SPI;
        ips_rst_pin = IPS200_RST_PIN_SPI;
        ips_bl_pin =  IPS200_BLk_PIN_SPI;
        ips_cs_pin =  IPS200_CS_PIN_SPI;
#if IPS200_USE_SOFT_SPI
        soft_spi_init(&ips200_spi, 0, IPS200_SOFT_SPI_DELAY, IPS200_SCL_PIN, IPS200_SDA_PIN, SOFT_SPI_PIN_NULL, SOFT_SPI_PIN_NULL);
#else
        spi_init(IPS200_SPI, SPI_MODE0, IPS200_SPI_SPEED, IPS200_SCL_PIN_SPI, IPS200_SDA_PIN_SPI, IPS200_SDA_IN_PIN_SPI, SPI_CS_NULL);
#endif

        gpio_init(IPS200_DC_PIN_SPI, GPO, GPIO_LOW, GPO_PUSH_PULL);
        gpio_init(ips_rst_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
        gpio_init(ips_cs_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
        gpio_init(ips_bl_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    }
    else
    {
        ips200_display_type = IPS200_TYPE_PARALLEL8;
        ips_rst_pin = IPS200_RST_PIN_PARALLEL8;
        ips_bl_pin = IPS200_BL_PIN_PARALLEL8;
        ips_cs_pin =  IPS200_CS_PIN_PARALLEL8;

        gpio_init(ips_rst_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);                   // RTS
        gpio_init(ips_bl_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);                    // BL
        gpio_init(ips_cs_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);                   // LCD_CS

        gpio_init(IPS200_RD_PIN_PARALLEL8, GPO, GPIO_LOW, GPO_PUSH_PULL);       // LCD_RD
        gpio_init(IPS200_WR_PIN_PARALLEL8, GPO, GPIO_LOW, GPO_PUSH_PULL);       // LCD_WR
        gpio_init(IPS200_RS_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_RS

        gpio_init(IPS200_D0_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D0
        gpio_init(IPS200_D1_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D1

        gpio_init(IPS200_D2_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D2
        gpio_init(IPS200_D3_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D3

        gpio_init(IPS200_D4_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D4
        gpio_init(IPS200_D5_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D5
        gpio_init(IPS200_D6_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D6
        gpio_init(IPS200_D7_PIN_PARALLEL8, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // LCD_D7
    }

    ips200_set_dir(ips200_display_dir);
    ips200_set_color(ips200_pencolor, ips200_bgcolor);
    
    IPS200_BL(1);
    IPS200_RST(0);  
    system_delay_ms(5);
    IPS200_RST(1);      
    system_delay_ms(120);
    
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_write_command(0x11);
    system_delay_ms(120);

    ips200_write_command(0x36);
    switch(ips200_display_dir)
    {
        case IPS200_PORTAIT:        ips200_write_8bit_data(0x00);   break;
        case IPS200_PORTAIT_180:    ips200_write_8bit_data(0xC0);   break;
        case IPS200_CROSSWISE:      ips200_write_8bit_data(0x70);   break;
        case IPS200_CROSSWISE_180:  ips200_write_8bit_data(0xA0);   break;
    }

    ips200_write_command(0x3A);
    ips200_write_8bit_data(0x05);
    
    ips200_write_command(0xB2);
    ips200_write_8bit_data(0x0C);
    ips200_write_8bit_data(0x0C);
    ips200_write_8bit_data(0x00);
    ips200_write_8bit_data(0x33);
    ips200_write_8bit_data(0x33);

    ips200_write_command(0xB7);
    ips200_write_8bit_data(0x35);

    ips200_write_command(0xBB);
    ips200_write_8bit_data(0x29);                                               // 32 Vcom=1.35V

    ips200_write_command(0xC2);
    ips200_write_8bit_data(0x01);

    ips200_write_command(0xC3);
    ips200_write_8bit_data(0x19);                                               // GVDD=4.8V 

    ips200_write_command(0xC4);
    ips200_write_8bit_data(0x20);                                               // VDV, 0x20:0v

    ips200_write_command(0xC5);
    ips200_write_8bit_data(0x1A);                                               // VCOM Offset Set

    ips200_write_command(0xC6);
    ips200_write_8bit_data(0x01F);                                              // 0x0F:60Hz

    ips200_write_command(0xD0);
    ips200_write_8bit_data(0xA4);
    ips200_write_8bit_data(0xA1);
                
    ips200_write_command(0xE0);
    ips200_write_8bit_data(0xD0);
    ips200_write_8bit_data(0x08);
    ips200_write_8bit_data(0x0E);
    ips200_write_8bit_data(0x09);
    ips200_write_8bit_data(0x09);
    ips200_write_8bit_data(0x05);
    ips200_write_8bit_data(0x31);
    ips200_write_8bit_data(0x33);
    ips200_write_8bit_data(0x48);
    ips200_write_8bit_data(0x17);
    ips200_write_8bit_data(0x14);
    ips200_write_8bit_data(0x15);
    ips200_write_8bit_data(0x31);
    ips200_write_8bit_data(0x34);

    ips200_write_command(0xE1);  
    ips200_write_8bit_data(0xD0);
    ips200_write_8bit_data(0x08);
    ips200_write_8bit_data(0x0E);
    ips200_write_8bit_data(0x09);
    ips200_write_8bit_data(0x09);
    ips200_write_8bit_data(0x15);
    ips200_write_8bit_data(0x31);
    ips200_write_8bit_data(0x33);
    ips200_write_8bit_data(0x48);
    ips200_write_8bit_data(0x17);
    ips200_write_8bit_data(0x14);
    ips200_write_8bit_data(0x15);
    ips200_write_8bit_data(0x31);
    ips200_write_8bit_data(0x34);

    ips200_write_command(0x21);
    
    ips200_write_command(0x29);
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }

    ips200_clear();                                                             // ��ʼ��Ϊ����
    ips200_debug_init();
}

