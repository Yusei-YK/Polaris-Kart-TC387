#include "kart_record.h"
#include "kart_flash.h"
#include "kart_odom.h"
#include "kart_control.h"
#include "kart_steer_abs.h"
#include <math.h>

#if defined(__TASKING__)
#pragma section all "cpu2_dsram"
#endif

static kart_waypoint_t record_buf[KART_RECORD_MAX_WAYPOINTS];
static uint16 record_count  = 0;
static uint8  record_running = 0;

/* 科目四开环反向复现专用并行数组(RAM only,不入 Flash,不改 kart_waypoint_t):
 *   steer_buf[i]  —— 第 i 点录制瞬间的转向绝对编码器 center_delta(真实物理打角,
 *                    遥控开车时即车轮实际转角)。开环倒车按里程索引回放同一打角。
 *   dist_buf[i]   —— 第 i 点相对录制起点的累计里程(米,单调增),用于把倒车里程
 *                    d 映射回原路弧长 s=total-d,查该 s 对应的打角。
 * 与 record_buf 同索引、同帧采样,保证打角/里程与坐标严格对齐。 */
static int16 steer_buf[KART_RECORD_MAX_WAYPOINTS];
static float dist_buf[KART_RECORD_MAX_WAYPOINTS];

static float origin_x   = 0.0f;
static float origin_y   = 0.0f;
static float origin_yaw = 0.0f;
static float origin_dist = 0.0f;    /* 录制起点累计里程基准,dist_buf 相对它 */

static float last_x   = 0.0f;
static float last_y   = 0.0f;
static float last_yaw = 0.0f;

static float kart_record_yaw_diff(float a, float b)
{
    float d = a - b;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

void kart_record_init(void)
{
    record_count   = 0;
    record_running = 0;
}

void kart_record_start(void)
{
    kart_odom_snapshot_t odom;

    record_count = 0;
    record_running = 1;

    /* 起点位姿一次性取整帧,x/y/yaw 必须同帧,否则录制坐标系原点就是歪的。 */
    kart_odom_get_snapshot(&odom);
    origin_x   = odom.x;
    origin_y   = odom.y;
    origin_yaw = odom.yaw;
    origin_dist = odom.dist_sum;        /* 里程基准:dist_buf 记相对增量 */

    last_x   = 0.0f;
    last_y   = 0.0f;
    last_yaw = 0.0f;

    kart_waypoint_t *wp = &record_buf[0];
    wp->x       = 0.0f;
    wp->y       = 0.0f;
    wp->yaw     = 0.0f;
    wp->v_left  = kart_control_get_left_meas();
    wp->v_right = kart_control_get_right_meas();
    steer_buf[0] = kart_steer_abs_get_center_delta();   /* 起点打角 */
    dist_buf[0]  = 0.0f;                                 /* 起点里程增量=0 */
    record_count = 1;
}

void kart_record_stop(void)
{
    record_running = 0;
}

void kart_record_poll(void)
{
    float cx, cy, cyaw, dx, dy, dyaw, dist;
    float origin_rad, origin_sin, origin_cos;
    kart_odom_snapshot_t odom;

    if(!record_running) return;
    if(record_count >= KART_RECORD_MAX_WAYPOINTS)
    {
        record_running = 0;
        return;
    }

    /* 一致快照:x/y/yaw 一次性取整帧,避免被 5ms 中断插到半路取到撕裂位姿。 */
    kart_odom_get_snapshot(&odom);
    dx = odom.x - origin_x;
    dy = odom.y - origin_y;

    /*
     * 里程计世界坐标与局部坐标都采用x向右、y向前。
     * 将世界坐标增量投影到录制起点车体系：
     *   right   = ( cos(yaw), sin(yaw))
     *   forward = (-sin(yaw), cos(yaw))
     */
    origin_rad = origin_yaw * 0.01745329252f;
    origin_sin = sinf(origin_rad);
    origin_cos = cosf(origin_rad);
    cx =  origin_cos * dx + origin_sin * dy;
    cy = -origin_sin * dx + origin_cos * dy;
    cyaw = kart_record_yaw_diff(odom.yaw, origin_yaw);

    dx   = cx - last_x;
    dy   = cy - last_y;
    dist = dx * dx + dy * dy;
    dyaw = fabsf(kart_record_yaw_diff(cyaw, last_yaw));

    if(dist < KART_RECORD_DIST_THRESH * KART_RECORD_DIST_THRESH && dyaw < KART_RECORD_YAW_THRESH)
        return;

    kart_waypoint_t *wp = &record_buf[record_count];
    wp->x       = cx;
    wp->y       = cy;
    wp->yaw     = cyaw;
    wp->v_left  = kart_control_get_left_meas();
    wp->v_right = kart_control_get_right_meas();
    /* 科目四开环回放:同帧记打角 + 相对起点累计里程(与该点坐标严格对齐)。 */
    steer_buf[record_count] = kart_steer_abs_get_center_delta();
    dist_buf[record_count]  = odom.dist_sum - origin_dist;
    record_count++;

    last_x   = cx;
    last_y   = cy;
    last_yaw = cyaw;
}

uint8  kart_record_is_running(void)          { return record_running; }
uint16 kart_record_get_count(void)           { return record_count; }
const  kart_waypoint_t* kart_record_get_waypoints(void) { return record_buf; }
float  kart_record_get_origin_yaw(void)      { return origin_yaw; }

/* 科目四开环回放取数据(与 record_buf 同索引):打角数组 / 累计里程数组 / 总里程。 */
const int16* kart_record_get_steer(void)     { return steer_buf; }
const float* kart_record_get_dist(void)      { return dist_buf; }
float  kart_record_get_total_dist(void)
{
    return (record_count > 0) ? dist_buf[record_count - 1] : 0.0f;
}

/* -------------------- Flash 持久化 -------------------- */
uint8 kart_record_save_to_flash(uint8 slot)
{
    if(record_running)   return 3;              /* 录制进行中不存,防写脏 */
    if(record_count == 0) return 2;
    return kart_flash_save_path(slot, record_buf, record_count);
}

uint16 kart_record_load_from_flash(uint8 slot)
{
    if(record_running) return 0;                /* 录制进行中不覆盖缓冲 */

    uint16 n = kart_flash_load_path(slot, record_buf, KART_RECORD_MAX_WAYPOINTS);
    record_count = n;                           /* 覆盖当前缓冲,供 playback 读 */
    return n;
}

#if defined(__TASKING__)
#pragma section all restore
#endif
