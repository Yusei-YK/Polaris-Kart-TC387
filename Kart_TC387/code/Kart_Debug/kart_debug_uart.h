#ifndef KART_DEBUG_UART_H_
#define KART_DEBUG_UART_H_
#include "zf_common_headfile.h"
#include "board_pins.h"

/*
 * 科目一串口日志。
 * 串口由 BOARD_AUX_UART_*(board_pins.h 的 LOG_ON_UART0)选:
 *   =0 → UART_10/P13.0 TX(无线模块排针)  ← 当前
 *   =1 → UART_0 /P14.0 TX(USB-TTL 直插)
 * 460800 baud，VOFA JustFloat(51通道float32+帧尾,共208字节)。
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

/* 科目三关键事件：CH45编号、CH46级别、CH47发生时的5ms节拍。 */
#define EVENT_LEVEL_INFO           (0U)
#define EVENT_LEVEL_WARNING        (1U)
#define EVENT_LEVEL_ERROR          (2U)
#define EVENT_S3_ENTER             (301U)
#define EVENT_S3_REVERSE_START     (302U)
#define EVENT_S3_DONE              (303U)
#define EVENT_S3_ABORT             (309U)
#define EVENT_VISION_ACQUIRED      (311U)
#define EVENT_VISION_NO_TARGET     (312U)
#define EVENT_CAMERA_FAULT         (313U)
#define EVENT_VISION_REJ_AREA      (314U)
#define EVENT_VISION_REJ_WIDTH     (315U)
#define EVENT_VISION_REJ_ASPECT    (316U)
#define EVENT_VISION_REJ_FILL      (317U)
#define EVENT_FOLLOW_NEAR_STOP     (318U)
#define EVENT_FOLLOW_STEER_SAT     (319U)
#define EVENT_FOLLOW_LOST_STOP     (320U)
#define EVENT_FOLLOW_TOO_FAR       (321U)
#define EVENT_FOLLOW_RECOVERED     (322U)
#define EVENT_CAMERA_RECOVERED     (323U)
#define EVENT_STEER_RECOVERED      (324U)
void kart_debug_uart_set_event(uint16 event_id, uint8 level);

#endif
