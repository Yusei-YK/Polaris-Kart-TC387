#ifndef KART_PLAYBACK_H_
#define KART_PLAYBACK_H_

#include "zf_common_headfile.h"

/*
 * 路径复现模块（Pure Pursuit 纯跟踪）
 * ------------------------------------------------------------------
 * 读 kart_record 录下的路径点（相对起点坐标），用 Pure Pursuit 算法
 * 跟踪：从当前位置沿路径前视一段距离找目标点 → 目标点方位角作为航向
 * 外环目标 → 串级转向自动打角；速度取录制点速度喂速度环。
 * ------------------------------------------------------------------
 * 坐标系：复现前必须先把车摆回录制起点(位置 + 朝向大致对齐)并发 z 清零，
 *   playback_start 以当前 odom 为原点，路径点为相对起点增量(仅平移，
 *   不做旋转补偿，故起点朝向要对齐)。
 * ------------------------------------------------------------------
 * 调用位置：
 *   kart_playback_start()  —— VOFA 命令 b1(先 z 清零、摆正车)
 *   kart_playback_poll()   —— 主循环(在 steer_abs_update 之后、
 *                             steer_ctrl_update 之前，好覆写目标航向)
 *   kart_playback_stop()   —— b0 或跑完自动停
 */

#define KART_PLAYBACK_LOOKAHEAD     (0.4f)      /* 前视距离(m) */
#define KART_PLAYBACK_FINISH_DIST   (0.15f)     /* 到终点判定距离(m):提前15cm停车避免过冲 */
#define KART_PLAYBACK_SPEED_MAX     (20.0f)     /* 复现速度上限钳位(脉冲/5ms):跟随录制速度,防毛刺冲出 */

void   kart_playback_init(void);
void   kart_playback_start(void);
void   kart_playback_stop(void);
void   kart_playback_poll(void);
uint8  kart_playback_is_running(void);
uint16 kart_playback_get_index(void);
float  kart_playback_get_target_yaw(void);

#endif
