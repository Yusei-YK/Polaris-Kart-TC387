#ifndef KART_STEER_ABS_H_
#define KART_STEER_ABS_H_
/*
 * 转向绝对值编码器(SPI_4,12 位)。全车【唯一】的转向角来源:软限位、回中、
 * 录轨里记的打角全靠它,读数丢了就等于失控 —— board_pins.h 里"冲突一"那段
 * 特意警告过无线模块库的出厂默认引脚正好踩这条 SPI,别让它初始化。
 *
 * 【节拍】kart_steer_abs_update() 每拍读一次,唯一调用点 cpu0_main.c:73,
 *   必须排在转向串级前面(kart_steer_ctrl.h:21 从另一头写了同一条),
 *   否则转向内环吃到的是上一拍的角度。
 *
 * 【四个 getter 的量纲,别混】
 *   get_raw()          12 位原始计数 0~4095,没减中位。
 *   get_frame()        SPI 读回的整 16 位帧;raw 是它右移 4 位再取低 12 位。
 *                      查 SPI 通不通看这个:恒 0 或恒 0xFFFF 就是没通。
 *   get_center_delta() 已减 kart_calib.h 的 CENTER_RAW、已解 4096 环绕,左正右负。
 *                      这个才是控制环和录轨真正用的量。
 *   get_deg_x100()     center_delta 换成 0.01 度,只是给人看的。
 *   中位、硬限位、软限位、满舵半径全在 kart_calib.h 第四节,本文件不放标定数。
 *
 * 【当前没人调的两个】get_frame() 与 get_deg_x100():2026-08-26 查零调用点。
 *   留着 —— 一个是查 SPI 死活的第一手数据,一个是出故障时给人看的单位。
 */
#include "zf_common_headfile.h"
#include "board_pins.h"

void kart_steer_abs_init(void);
void kart_steer_abs_update(void);
uint16 kart_steer_abs_get_frame(void);
uint16 kart_steer_abs_get_raw(void);
int16 kart_steer_abs_get_center_delta(void);
int16 kart_steer_abs_get_deg_x100(void);

#endif
