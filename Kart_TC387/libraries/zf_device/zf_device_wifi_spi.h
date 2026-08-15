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
* �ļ�����          zf_device_wifi_spi
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-18        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   RST                 �鿴 zf_device_wifi_spi.h �� WIFI_SPI_RST_PIN �궨��
*                   INT                 �鿴 zf_device_wifi_spi.h �� WIFI_SPI_INT_PIN �궨��
*                   CS                  �鿴 zf_device_wifi_spi.h �� WIFI_SPI_CS_PIN �궨��
*                   MISO                �鿴 zf_device_wifi_spi.h �� WIFI_SPI_MISO_PIN �궨��
*                   SCK                 �鿴 zf_device_wifi_spi.h �� WIFI_SPI_SCK_PIN �궨��
*                   MOSI                �鿴 zf_device_wifi_spi.h �� WIFI_SPI_MOSI_PIN �궨��
*                   5V                  5V ��Դ
*                   GND                 ��Դ��
*                   ������������
*                   ------------------------------------
*********************************************************************************************************************/

#ifndef _zf_device_wifi_spi_h
#define _zf_device_wifi_spi_h

#include "zf_common_typedef.h"

          
/* ============ 2026-08-12 Kart 工程改：按本车无线排针网表覆盖驱动默认引脚 ============
 * 【为什么必须改库文件】下面这些是硬编码宏，wifi_spi_init() 没有引脚形参，
 * 不改库就没有任何办法换脚。
 *
 * 【出厂默认值(已注释保留，换板回退用)】
 *     WIFI_SPI_INDEX     SPI_4
 *     WIFI_SPI_SPEED     30MHz
 *     WIFI_SPI_SCK_PIN   SPI4_SCLK_P22_3
 *     WIFI_SPI_MOSI_PIN  SPI4_MOSI_P22_0
 *     WIFI_SPI_MISO_PIN  SPI4_MISO_P22_1
 *     WIFI_SPI_CS_PIN    P22_2
 *     WIFI_SPI_INT_PIN   P15_8
 *     WIFI_SPI_RST_PIN   P23_1
 *
 * 【为什么不能用出厂默认值：会打死转向绝对值编码器】
 *   本车 KART_STEER_ABS_* 占用 SPI_4 + P22.3(SCK)/P22.0(MOSI)/P22.1(MISO)，
 *   片选是 P23.1 —— 与上面默认值逐条重合，连 RST 都正好压在编码器片选上。
 *   照默认初始化的后果不是"图传不通"，是转向角读数当场失效：
 *   转向软限位和回中都只有这一个数据源，丢了就是失控。
 *   本车无线排针实际走线是 SPI_2 + P15.x。IPS200 已改为 P02.8/P20.3 软件 SPI，
 *   两者不冲突；见 board_pins.h 的"无线模块(SPI 版，图传用)"一节。
 *
 * 【改法：为什么在这里就地写死，而不是 include "board_pins.h" 转发】
 *   zf_common_headfile.h:111 已经 include 了本文件，而 board_pins.h 又要
 *   include zf_common_headfile.h —— 在这里再反向 include board_pins.h 就构成
 *   头文件环。带 include guard 的环不会无限递归，但会让 board_pins.h 在
 *   "SPI_2 / SPI2_SCLK_P15_3 这些枚举还没被 zf_driver_spi.h 定义"的时机被展开，
 *   报一堆莫名其妙的未定义符号，且报错位置和真正的原因完全对不上。
 *   所以这里直接写死引脚，board_pins.h 那一节只保留文字说明与取舍依据；
 *   两处数值一致性由 kart_wifi.c 里的编译期断言强制(那是 .c，可以安全比较枚举)。
 *
 * 【与转向编码器的一致性检查放哪】放在 kart_wifi.c：
 *   预处理器无法比较 SPI_2 / SPI_4 这类枚举常量(它把标识符当 0)，
 *   写 #if (WIFI_SPI_INDEX == KART_STEER_ABS_SPI_INDEX) 会恒真、永远误报。
 *   所以改用 C 层的 typedef 数组断言，见 kart_wifi.c 顶部。 */

/* 本车实际走线：无线模块独占硬件 SPI_2 + P15.x；IPS200 使用另一组软件 SPI。 */
#define WIFI_SPI_INDEX              (SPI_2             )        // 本车无线排针(出厂默认 SPI_4，与转向编码器冲突)
#define WIFI_SPI_SPEED              (1 * 1000 * 1000   )        // 联调降到 1MHz，先排除排针/走线信号完整性(通后再提速)
#define WIFI_SPI_SCK_PIN            (SPI2_SCLK_P15_3   )        // 排针 3   (出厂默认 SPI4_SCLK_P22_3)
#define WIFI_SPI_MOSI_PIN           (SPI2_MOSI_P15_5   )        // 排针 5   (出厂默认 SPI4_MOSI_P22_0)

/* 网表 + WIFI6B21-SPI V2.0 手册逐项确认：1=MISO、3=SCK、4=CS、5=MOSI、
 * 6=INT、7/8=GND、9=5V、10=RST。当前主板必须保持 SWAP=0；SWAP 只为旧 PCB
 * 的历史 A/B 排查保留，当前板读不到版本时应测 CS/SCK/MOSI/RST 波形，不再对调。 */
#ifndef KART_WIFI_SWAP_CS_MISO
/* 2026-08-12 网表已确认 1=MISO、4=CS，固定用 0。 */
#define KART_WIFI_SWAP_CS_MISO      (0)
#endif
#if (KART_WIFI_SWAP_CS_MISO == 0)
#define WIFI_SPI_MISO_PIN           (SPI2_MISO_P15_4   )        // 排针 1   (出厂默认 SPI4_MISO_P22_1)
#define WIFI_SPI_CS_PIN             (P15_2             )        // 排针 4，软片选(出厂默认 P22_2)
#else
#define WIFI_SPI_MISO_PIN           (SPI2_MISO_P15_2   )        // 排针 4
#define WIFI_SPI_CS_PIN             (P15_4             )        // 排针 1，软片选
#endif

#define WIFI_SPI_INT_PIN            (P15_8             )        // 排针 6；驱动按浮空输入读取，避免内部下拉加载模块 INT
#define WIFI_SPI_RST_PIN            (P33_5             )        // 排针 10；排针 7/8 均为 GND(出厂默认 P23_1，绝不可用)


#define WIFI_SPI_RECVIVE_FIFO_SIZE  (1024)                      // ����FIFO��С
#define WIFI_SPI_READ_TRANSFER      (1)                         // �ڵ���wifi_spi_read_buffer �Ƿ��Է���SPIͨѶ�����ģ�����Ƿ���������Ҫ��ȡ 1������SPIͨѶ 0��������SPIͨѶ������ȡFIFO
                                                                // ���Ӧ�ó�����û���κεĵط����÷��ͺ�������WIFI_SPI_READ_TRANSFER��������Ϊ1
                                                                
#define WIFI_SPI_AUTO_CONNECT       (0)                         // �����Ƿ��ʼ��ʱ����TCP����UDP����    0-���Զ�����  1-�Զ�����TCP������  2-�Զ�����UDP

#if     (WIFI_SPI_AUTO_CONNECT > 2)    
#error "WIFI_SPI_AUTO_CONNECT ��ֵֻ��Ϊ [0,1,2]"
#else   
#define WIFI_SPI_TARGET_IP          "192.168.137.1"              // ����Ŀ��� IP
#define WIFI_SPI_TARGET_PORT        "8086"                      // ����Ŀ��Ķ˿�
#define WIFI_SPI_LOCAL_PORT         "6666"                      // �����Ķ˿� 0�����  �����÷�Χ2048-65535  Ĭ�� 6666
#endif


#define WIFI_SPI_RECVIVE_SIZE       (32)                        // ÿ��SPI������յ��ֽ��� �������޸�
#define WIFI_SPI_TRANSFER_SIZE      (4088)                      // ���SPI������յ��ֽ��� �������޸�



typedef enum
{
    // �������͵�����
    WIFI_SPI_INVALID1               = 0x00,                     // ��Ч���ݰ�
    WIFI_SPI_RESET                  = 0x01,                     // ��λ����
    WIFI_SPI_DATA                   = 0x02,                     // ͸�����ݰ�
    WIFI_SPI_UDP_SEND               = 0x03,                     // UDP��������������,Ĭ��SPI�������ݺ�2MSδ�յ������Զ���������
    WIFI_SPI_CLOSE_SOCKET           = 0x04,                     // �Ͽ�����
                
    WIFI_SPI_SET_WIFI_INFORMATION   = 0x10,                     // ����WIFI��Ϣ����
    WIFI_SPI_SET_SOCKET_INFORMATION = 0x11,                     // ����SOCKET��Ϣ����
    WIFI_SPI_SET_WIFI_SCAN          = 0x12,                     // ��ʼɨ��WIFI
    WIFI_SPI_SET_READ_LENGTH        = 0x13,                     // ��������ȡ����
                    
    WIFI_SPI_GET_VERSION            = 0x20,                     // ��ȡģ��汾
    WIFI_SPI_GET_MAC_ADDR           = 0x21,                     // ��ȡģ��MAC��ַ
    WIFI_SPI_GET_IP_ADDR            = 0x22,                     // ��ȡģ��IP��ַ
    WIFI_SPI_GET_TIME1              = 0x23,                     // ��ȡʱ�� ��ʽ1
    WIFI_SPI_GET_TIME2              = 0x24,                     // ��ȡʱ�� ��ʽ2
    WIFI_SPI_GET_TIME3              = 0x25,                     // ��ȡʱ�� ��ʽ3
                    
    // �ӻ��ش�������
    WIFI_SPI_REPLY_OK               = 0x80,                     // �ӻ�Ӧ�����ȷ����
    WIFI_SPI_REPLY_ERROR            = 0x81,                     // �ӻ�Ӧ��Ĵ�������
                    
    WIFI_SPI_REPLY_DATA_START       = 0x90,                     // �ӻ��ش������ݰ������һ���������Ҫ������ȡ
    WIFI_SPI_REPLY_DATA_END         = 0x91,                     // �ӻ��ش������ݰ��������Ѷ�ȡ���
                    
    WIFI_SPI_REPLY_VERSION          = 0xA0,                     // �ӻ��ظ��̼��汾
    WIFI_SPI_REPLY_MAC_ADDR         = 0xA1,                     // �ӻ��ظ�����MAC��ַ����Ϣ
    WIFI_SPI_REPLY_IP_ADDR          = 0xA2,                     // �ӻ��ظ�����IP��ַ���˿ں�
    WIFI_SPI_REPLY_TIME1            = 0xA3,                     // �ӻ��ظ�ʱ��
    WIFI_SPI_REPLY_TIME2            = 0xA4,                     // �ӻ��ظ�ʱ��
    WIFI_SPI_REPLY_TIME3            = 0xA5,                     // �ӻ��ظ�ʱ��
    WIFI_SPI_INVALID2               = 0xFF                      // ��Ч���ݰ�
}wifi_spi_packets_command_enum;             
                
typedef enum                
{               
    WIFI_SPI_IDLE,                                              // ģ����У����Խ���SPIͨѶ
    WIFI_SPI_BUSY,                                              // ģ����æ�����ɽ���SPIͨѶ
}wifi_spi_state_enum;               
                
                
typedef struct              
{               
    uint8   command;                                            // ������
    uint8   reserve;                                            // ����
    uint16  length;                                             // ����Ч����
}wifi_spi_head_struct;              
                
                
typedef struct              
{               
    wifi_spi_head_struct  head;                                 // ֡ͷ
    uint8 buffer[WIFI_SPI_RECVIVE_SIZE];                        // ������
}wifi_spi_packets_struct;               
                
typedef enum
{
    WIFI_SPI_UTC_0 = 1,                                         // ����ʱ��
    WIFI_SPI_GMT,                                               // ����ʱ�� ת��ΪGMT��ʽ����׺����GMT һ�����ڶԽ��ƶ˴�ģ��ʹ��
    WIFI_SPI_UTC_8,                                             // ����ʱ��
}wifi_spi_time_enum;

extern char wifi_spi_version[12];                               // �̼��汾         �ַ���
extern char wifi_spi_mac_addr[20];                              // ģ��MAC��ַ      �ַ���
extern char wifi_spi_ip_addr_port[25];                          // IP��ַ��˿ں�   �ַ���

uint8   wifi_spi_get_time           (wifi_spi_time_enum time_format, char *buffer, uint8 buffer_size);
uint8   wifi_spi_wifi_scan          (char *buffer, uint16 buffer_size);
uint8   wifi_spi_wifi_connect       (char *wifi_ssid, char *pass_word);
uint8   wifi_spi_socket_connect     (char *transport_type, char *ip_addr, char *port, char *local_port);
uint8   wifi_spi_socket_disconnect  (void);
uint8   wifi_spi_udp_send_now       (void);
uint32  wifi_spi_send_buffer        (const uint8 *buff, uint32 length);
void    wifi_spi_send_string        (const char *string);
uint32  wifi_spi_read_buffer        (uint8 *buffer, uint32 length);

uint8   wifi_spi_init               (char *wifi_ssid, char *pass_word);

#endif
