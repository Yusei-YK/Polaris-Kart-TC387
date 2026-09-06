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
 *   kart_record_poll()  —— 【10ms 一拍】cpu0_main.c 的 kart_task_10ms(),
 *       紧跟在 kart_mission_poll() 后面。而且不是直调:中间隔着
 *       kart_multicore_record_poll(),那个包装只在 runtime_enabled 为真时才把
 *       活派给 CPU2 —— 该标志仅在 #if KART_MULTICORE_COMPAT_ENABLE 里被置 1,
 *       宏是 0,所以出厂固件恒走 else 分支,poll 就在 CPU0 上原地跑。
 *       采样阈值是【距离/转角】的,不是时间的,所以换拍率不改路径形状,
 *       只改能跟上 0.05m 间距的最高车速(10ms 一拍、2m/s 时每拍走 2cm)。
 *   kart_record_start/stop() —— 六处,都不是调试口。
 *       start: kart_menu.c   menu_handle_recording_mid_press()
 *              kart_mission.c mission_enter()
 *       stop : kart_menu.c   menu_handle_recording_mid_press()
 *              kart_mission.c mission_stop_all()
 *              kart_mission.c subject3_loop() 两处(正常收尾 + 异常收尾)
 *       【2026-09-06 为什么这里不写行号了】原来这六处写的是
 *       menu 2097/2102、mission 87/174/827/883,现在全是错的,而本文件谁都没动过
 *       —— 是别处插进来的代码把它们顶下去的(改踏板那批往 kart_menu.c 里加了
 *       二十多行)。而且错得不显眼:老的 mission.c:87 现在是 kart_playback_stop(),
 *       看上去还挺像那么回事,真正的 kart_record_stop() 在 88 行。
 *       函数名不会这样漂,所以改成按函数定位。上面 kart_task_10ms 那处同理。
 *       【原来写的"VOFA 命令 r1/r0"已作废】调试口现在一处都不碰录制。
 * ------------------------------------------------------------------
 * Flash 持久化【已经做完了,不是后续扩展】:接口就是本文件下面的
 * kart_record_save_to_flash() / kart_record_load_from_flash(),菜单已在调。
 * 原来这里写的 kart_record_flush_to_flash() 全工程不存在,别去找。
 */

/* 【容量上限值得先算一遍】1500 点 × 0.05m = 直线 75m 封顶;弯道里 2° 转角阈值
 * 会额外花点,所以实际能录的路程只会比 75m 短。跟随段实测已经 65m,离顶不远。
 * 【录满了会静默停录】kart_record_poll() 见 record_count 到顶就把 record_running
 * 清 0 直接返回(kart_record.c 里那个 >= MAX_WAYPOINTS 分支),没有溢出标志、没有
 * 返回码,kart_record_is_running() 只是从 1 变 0 —— 和正常停录一模一样,现场只能
 * 靠"点数恰好停在 1500"这一个迹象判断。要录更长的路先加大本宏(RAM 在 cpu2_dsram,
 * 三个数组按点数线性涨),不要靠放大距离阈值,那会直接牺牲路径精度。 */
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
 * 科目三反向复现用它把世界 kart_odom 投影回录制坐标系,与倒序数组同系。 */
float  kart_record_get_origin_yaw(void);

/* 取本次录制起点的世界坐标(米)。与 origin_yaw 合起来才是完整的坐标系变换:
 * 录制坐标 = R(-origin_yaw) * (世界坐标 - 录制原点)。科目三位置闭环倒车用它
 * 把当前 kart_odom 位置换算到与 wp[].x/y 同系,才能算横向偏差。 */
float  kart_record_get_origin_x(void);
float  kart_record_get_origin_y(void);

/* -------------------- 开环反向复现数据(2026-07-28 起随路径一起进 Flash)-------------------- */
/* 与 kart_record 路径点同索引、同帧采样的两个并行数组:
 *   kart_steer[i] = 第 i 点录制瞬间转向绝对编码器 center_delta(真实物理打角);
 *   dist[i]  = 第 i 点相对录制起点的累计里程(米,单调增)。
 * 开环倒车按里程 s=total-d 索引回放同一打角,不走 Pure Pursuit/航向外环。
 * 科目一录制轨迹里的倒车段也吃 kart_steer[]:kart_playback 见到负速就切开环回放打角。
 * 【原 RAM only 的后果】从槽位载入后这两个数组是残留/全 0,倒车打角只剩纠偏
 * (钳在 ±400 计数)→ 半径 3.7m 倒不进库。现已随路径持久化。 */
const int16* kart_record_get_steer(void);       /* 打角数组首址 */
const float* kart_record_get_dist(void);        /* 累计里程数组首址 */
float  kart_record_get_total_dist(void);        /* 全程总里程(=末点 dist) */

/* -------------------- Flash 持久化便捷接口 -------------------- */
/* 把当前 RAM 里录好的路径存进 Flash 指定槽位(阻塞擦写,只能停车静止时调)。
 * 存的内容:路径点 + kart_steer[] + dist[] + 起点位姿(origin_yaw/x/y)。
 * 返回 0=成功,非 0=失败(见 kart_flash_save_path)。录制进行中拒绝存(返回 3)。 */
uint8  kart_record_save_to_flash(uint8 slot);

/* 从 Flash 指定槽位读回路径到 RAM(覆盖当前录制缓冲),供 kart_playback 复现。
 * 连 kart_steer[]/dist[]/起点位姿一起还原,故载入的路径含倒车段也能正常复现。
 * 返回读回点数;0=槽位空/无效/是 v1 老格式(布局不兼容,须重录)。
 * 录制进行中拒绝读(返回 0)。 */
uint16 kart_record_load_from_flash(uint8 slot);

#endif
