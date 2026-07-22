#ifndef KART_RECORD_H_
#define KART_RECORD_H_

#include "zf_common_headfile.h"

/*
 * 路径录制模块
 * ------------------------------------------------------------------
 * 车辆行驶时按"距离或转角阈值"自适应采样,把航位推算坐标 + 速度打包
 * 存入 RAM 环形缓冲。坐标/yaw 均取相对录制起点的增量,不依赖绝对零点,
 * 规避 IMU 漂移和重启后零点不一致的问题。
 * ------------------------------------------------------------------
 * 调用位置:
 *   kart_record_init()  —— cpu0_main.c 初始化段
 *   kart_record_poll()  —— 主循环(每拍检查阈值)
 *   kart_record_start/stop() —— VOFA 命令 r1/r0 触发
 * ------------------------------------------------------------------
 * 后续扩展:Flash 写入在 kart_record_flush_to_flash() 里做,
 *          IPS200 菜单调它保存/加载路线。
 */

#define KART_RECORD_MAX_WAYPOINTS   (1500)
#define KART_RECORD_DIST_THRESH     (0.05f)
#define KART_RECORD_YAW_THRESH      (2.0f)

typedef struct
{
    float x;
    float y;
    float yaw;
    float v_left;
    float v_right;
} kart_waypoint_t;

void   kart_record_init(void);
void   kart_record_start(void);
void   kart_record_stop(void);
void   kart_record_poll(void);
uint8  kart_record_is_running(void);
uint16 kart_record_get_count(void);
const  kart_waypoint_t* kart_record_get_waypoints(void);

/* -------------------- Flash 持久化便捷接口 -------------------- */
/* 把当前 RAM 里录好的路径存进 Flash 指定槽位(阻塞擦写,只能停车静止时调)。
 * 返回 0=成功,非 0=失败(见 kart_flash_save_path)。录制进行中拒绝存(返回 3)。 */
uint8  kart_record_save_to_flash(uint8 slot);

/* 从 Flash 指定槽位读回路径到 RAM(覆盖当前录制缓冲),供 kart_playback 复现。
 * 返回读回点数;0=槽位空/无效。录制进行中拒绝读(返回 0)。 */
uint16 kart_record_load_from_flash(uint8 slot);

#endif
