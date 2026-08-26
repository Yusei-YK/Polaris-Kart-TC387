#ifndef KART_ENCODER_H_
#define KART_ENCODER_H_
/*
 * 后轮增量编码器(左 TIM2 / 右 TIM5,正交 512 线 × 4 边沿 = 2048 脉冲/圈)。
 * 引脚在 board_pins.h;极性 KART_LEFT/RIGHT_ENCODER_SIGN 与脉冲当量
 * KART_LEFT/RIGHT_ENC_PULSE_TO_M 在 kart_calib.h 第二、三节 —— 换轮子、换编码器
 * 只改那边,本文件一个标定数都不放。
 *
 * 【节拍】kart_encoder_update() 是速度环专用的,唯一调用点 kart_control.c:73
 *   (5ms 环)。它每次读完就把硬件计数清零,所以多调一次就等于偷走一拍的计数,
 *   速度环立刻少一拍。里程模块为此专门声明绝不调它(kart_odom.h:26),
 *   要读就读累计和。
 *
 * 【量纲】三组量容易混,分清楚:
 *   delta = 本拍脉冲增量,单倍(不是 4 倍边沿),已套极性符号 —— 速度环吃这个。
 *   sum   = 上次 reset 以来的累计,同样已套极性 —— 里程吃这个。
 *   raw   = 未套极性的本拍增量,给排线序/极性排查用,不要拿它进控制。
 *   脉冲 → 米用 kart_calib.h 的 PULSE_TO_M;脉冲/拍 → m/s 用 KART_PULSE_V_TO_MS。
 *
 * 【当前没人调的四个】kart_encoder_reset()、get_left_raw()、get_right_raw()、
 *   get_state():2026-08-26 查全仓库零调用点。留着不删 —— reset 是重开里程累计
 *   的正规入口,raw 与 state 是查接线和极性时的现成抓手。
 */
#include "zf_common_headfile.h"
#include "board_pins.h"

typedef struct
{
    int16 left_delta;
    int16 right_delta;
    int16 left_raw;
    int16 right_raw;
    int32 left_sum;
    int32 right_sum;
} kart_encoder_state_t;

void kart_encoder_init(void);
void kart_encoder_update(void);
void kart_encoder_reset(void);
int16 kart_encoder_get_left_delta(void);
int16 kart_encoder_get_right_delta(void);
int16 kart_encoder_get_left_raw(void);
int16 kart_encoder_get_right_raw(void);
int32 kart_encoder_get_left_sum(void);
int32 kart_encoder_get_right_sum(void);
const kart_encoder_state_t *kart_encoder_get_state(void);

#endif
