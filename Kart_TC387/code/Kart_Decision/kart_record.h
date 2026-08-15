#ifndef KART_RECORD_H_
#define KART_RECORD_H_

#include "zf_common_headfile.h"

/*
 * 路径录制模块
 * ------------------------------------------------------------------
 * 车辆行驶时按"距离或转角阈值"自适应采样,把航位推算坐标 + 速度打包
 * 存入 RAM 缓冲。坐标旋转到录制起点车体系(x向右、y向前)，yaw取相对起点增量，
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

/* 停车后的板上路径修正：把 center 前后 radius 个点平滑平移。
 * 中心移动 dx/dy 米，越靠近范围边缘移动越小；点 0 固定为录制原点。
 * 返回 1=已修改，0=参数无效/正在录制。 */
uint8  kart_record_adjust_segment(uint16 center, uint16 radius, float dx, float dy);

/* 取本次录制起点航向(度,世界系):录制坐标系相对里程计世界系的旋转量。
 * 科目三反向复现用它把世界 odom 投影回录制坐标系,与倒序数组同系。 */
float  kart_record_get_origin_yaw(void);

/* 取本次录制起点的世界坐标(米)。与 origin_yaw 合起来才是完整的坐标系变换:
 * 录制坐标 = R(-origin_yaw) * (世界坐标 - 录制原点)。科目三位置闭环倒车用它
 * 把当前 odom 位置换算到与 wp[].x/y 同系,才能算横向偏差。 */
float  kart_record_get_origin_x(void);
float  kart_record_get_origin_y(void);

/* -------------------- 开环反向复现数据(2026-07-28 起随路径一起进 Flash)-------------------- */
/* 与 record 路径点同索引、同帧采样的两个并行数组:
 *   steer[i] = 第 i 点录制瞬间转向绝对编码器 center_delta(真实物理打角);
 *   dist[i]  = 第 i 点相对录制起点的累计里程(米,单调增)。
 * 开环倒车按里程 s=total-d 索引回放同一打角,不走 Pure Pursuit/航向外环。
 * 科目一录制轨迹里的倒车段也吃 steer[]:playback 见到负速就切开环回放打角。
 * 【原 RAM only 的后果】从槽位载入后这两个数组是残留/全 0,倒车打角只剩纠偏
 * (钳在 ±400 计数)→ 半径 3.7m 倒不进库。现已随路径持久化。 */
const int16* kart_record_get_steer(void);       /* 打角数组首址 */
const float* kart_record_get_dist(void);        /* 累计里程数组首址 */
float  kart_record_get_total_dist(void);        /* 全程总里程(=末点 dist) */

/* -------------------- Flash 持久化便捷接口 -------------------- */
/* 把当前 RAM 里录好的路径存进 Flash 指定槽位(阻塞擦写,只能停车静止时调)。
 * 存的内容:路径点 + steer[] + dist[] + 起点位姿(origin_yaw/x/y)。
 * 返回 0=成功,非 0=失败(见 kart_flash_save_path)。录制进行中拒绝存(返回 3)。 */
uint8  kart_record_save_to_flash(uint8 slot);

/* 从 Flash 指定槽位读回路径到 RAM(覆盖当前录制缓冲),供 kart_playback 复现。
 * 连 steer[]/dist[]/起点位姿一起还原,故载入的路径含倒车段也能正常复现。
 * 返回读回点数;0=槽位空/无效/是 v1 老格式(布局不兼容,须重录)。
 * 录制进行中拒绝读(返回 0)。 */
uint16 kart_record_load_from_flash(uint8 slot);

#endif
