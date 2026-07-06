#ifndef KART_DEBUG_UART_H_
#define KART_DEBUG_UART_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

/*
 * 卡丁车调参台 —— VOFA+ 波形输出 + 在线改参
 * ------------------------------------------------------------------
 * 走无线转串口(UART_2,双向)。两条数据流:
 *   下行(单片机→上位机):VOFA+ justfloat 二进制协议,多通道浮点波形。
 *   上行(上位机→单片机):ASCII 单字符命令 + 数值,在线改 PID / 目标 / 使能。
 *
 * 为什么换掉原来的 CSV:
 *   1. CSV 是文本,一个 float 要十几个字节,还得 sprintf,5ms 拍里扛不住;
 *      VOFA justfloat 一个通道固定 4 字节,直接内存拷贝,快且省。
 *   2. VOFA+ 能实时画波形,调 PID 看阶跃响应比读数字直观太多。
 *
 * justfloat 帧格式(VOFA+ 官方定义):
 *   [ch0(4B) ch1(4B) ... chN-1(4B)] + 帧尾 [0x00 0x00 0x80 0x7F]
 *   每个通道是小端 float(TC387 本来就是小端,直接拷),帧尾是固定 4 字节。
 *   当前 7 通道:ch0=目标速度 ch1=实测速度(滤波后) ch2=输出duty
 *               ch3=航向yaw ch4=RX累计字节数(诊断:看命令有没有进 MCU)
 *               ch5=转向raw(0~4095) ch6=转向中位偏差(角度环误差源)
 * ------------------------------------------------------------------
 * 在线改参命令(ASCII,以换行/回车/空格结尾三者任一即触发解析):
 *   p<数>   改 Kp,例如 "p12.5"
 *   i<数>   改 Ki,例如 "i0.02"
 *   d<数>   改 Kd,例如 "d300"
 *   t<数>   改目标速度(脉冲/5ms),例如 "t8"
 *   e<0/1>  使能/关闭速度环,例如 "e1" 开、"e0" 停
 * ------------------------------------------------------------------
 */

/* VOFA 波形下发周期(ms)。调 PID 看阶跃要够快,20ms(50Hz)足够顺滑又不刷爆串口。 */
#define KART_DEBUG_UART_PERIOD_MS       (20)

/* 命令接收缓冲区长度:一条命令最长 "p-1234.56" 量级,32 字节绰绰有余 */
#define KART_DEBUG_CMD_BUF_LEN          (32)

void kart_debug_uart_init(void);
void kart_debug_uart_poll(void);        // 放主循环:发 VOFA 波形 + 收命令改参

#endif
