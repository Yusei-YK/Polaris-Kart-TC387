#ifndef TEST_ZF_COMMON_HEADFILE_H_
#define TEST_ZF_COMMON_HEADFILE_H_

#include <stdint.h>

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int16_t  int16;
typedef int32_t  int32;

/* 仅供 kart_mission_ui.c 做主机语法检查的 IPS200 最小接口桩。 */
#define RGB565_WHITE (0xFFFFU)
#define RGB565_BLACK (0x0000U)

typedef enum
{
    IPS200_TYPE_SPI = 0
} ips200_type_enum;

typedef enum
{
    IPS200_CROSSWISE = 2
} ips200_dir_enum;

typedef enum
{
    IPS200_8X16_FONT = 1
} ips200_font_size_enum;

void ips200_init(ips200_type_enum type_select);
void ips200_set_dir(ips200_dir_enum dir);
void ips200_set_font(ips200_font_size_enum font);
void ips200_set_color(uint16 pen, uint16 bgcolor);
void ips200_full(uint16 color);
void ips200_show_string(uint16 x, uint16 y, const char dat[]);

#endif
