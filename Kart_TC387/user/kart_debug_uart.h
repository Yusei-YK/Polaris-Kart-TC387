#ifndef KART_DEBUG_UART_H_
#define KART_DEBUG_UART_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

/*
 * 科目一串口日志。
 * UART_10/P13.0 TX，460800 baud，VOFA JustFloat(33通道float32+帧尾,共136字节)。
 * 5 ms中断只维护tick，组帧和发送全部放在CPU0主循环。
 */
#define KART_LOG_PERIOD_TICKS           (4U)   /* 4 * 5 ms = 20 ms，50 Hz */

void kart_debug_uart_init(void);

/* 日志总闸。科目二要把 UART_10 让给语音模块(见 board_pins.h),
 * 必须先关闸再切波特率:否则 background_poll 会继续把 JustFloat 字节
 * 以 115200 喷到语音模块的 RX 上。关闸时顺带丢弃环形缓冲里的残留半帧。 */
void kart_debug_uart_set_enabled(uint8 enabled);
uint8 kart_debug_uart_is_enabled(void);
void kart_debug_uart_tick_5ms(void);
void kart_debug_uart_poll(void);              /* 采样组帧入环形缓冲(放调度任务) */
void kart_debug_uart_background_poll(void);   /* 后台分块非阻塞发送(放主循环每 spin) */

#endif
