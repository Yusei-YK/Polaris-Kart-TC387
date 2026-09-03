#ifndef KART_DEBUG_UART_H_
#define KART_DEBUG_UART_H_
#include "zf_common_headfile.h"
#include "board_pins.h"

/*
 * 科目一串口日志。
 * 串口由 BOARD_AUX_UART_*(board_pins.h 的 LOG_ON_UART0)选:
 *   =0 → UART_10/P13.0 TX(无线模块排针)  ← 当前
 *   =1 → UART_0 /P14.0 TX(USB-TTL 直插)
 * 460800 baud，VOFA JustFloat(通道数看 kart_debug_uart.c 的 KART_LOG_CHANNELS)。
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

/* 科目三关键事件：CH45编号、CH46级别、CH47发生时的5ms节拍。
 * 【这三路只在全量档发得出去】出厂编译的是 43 路剖面(kart_debug_uart.c 的
 * LOG_PROFILE_S3=1),只到 CH42;ch[45..47] 的赋值在 #else 里。也就是说
 * set_event() 照样把事件写进模块静态量,VOFA 上却一路都看不见。
 * 想在现场看事件:要么把 LOG_PROFILE_S3 改 0(VOFA 通道数同步改 51),
 * 要么拿调试器 watch kart_event_id / level / tick。 */
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

/* CH38 分段打点 +1。日志模块自己会数 START(P20.7) 下降沿,
 * 【本接口全工程没有任何调用点】留着当退路:若现场发现 START 在当前
 * 菜单页上有别的作用,换一个键/菜单项调它即可。
 * 另注意 CH38 是打点只在出厂档(43 路)成立;全量档的 CH38 是
 * 摄像头中断最长耗时。 */
void kart_debug_uart_bump_mark(void);

#endif
