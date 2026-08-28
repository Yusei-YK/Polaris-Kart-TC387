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

/* 开环反向复现专用并行数组(2026-07-28 起【随路径一起进 Flash】,不改 kart_waypoint_t):
 *   steer_buf[i]  —— 第 i 点录制瞬间的转向绝对编码器 center_delta(真实物理打角,
 *                    遥控开车时即车轮实际转角)。开环倒车按里程索引回放同一打角。
 *   dist_buf[i]   —— 第 i 点相对录制起点的累计里程(米,单调增),用于把倒车里程
 *                    d 映射回原路弧长 s=total-d,查该 s 对应的打角。
 * 与 record_buf 同索引、同帧采样,保证打角/里程与坐标严格对齐。
 * 【为什么后来必须入 Flash】科目一从槽位载入后倒车半径明显大于手动、倒不进库:
 * kart_playback 倒车段的打角是开环回放 steer_buf[nearest] 的,过去这数组不进 Flash,
 * 载入路径后它还是上次录制的残留(冷启动即全 0)→ 打角只剩航向纠偏一项,而纠偏被
 * REV_CORR_MAX=400 计数钳死 → R=1480/400≈3.7m,手动满锁只要 1.39m
 * (1480/1064,见 kart_calib.h 的 KART_STEER_MIN_RADIUS_M;原注释写的 1.32m 是
 * 重标中位之前的值,已订正 —— 差距比原来还大一点,结论不变)。 */
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
    kart_odom_snapshot_t kart_odom;

    record_count = 0;
    record_running = 1;

    /* 起点位姿一次性取整帧,x/y/yaw 必须同帧,否则录制坐标系原点就是歪的。 */
    kart_odom_get_snapshot(&kart_odom);
    origin_x   = kart_odom.x;
    origin_y   = kart_odom.y;
    origin_yaw = kart_odom.yaw;
    origin_dist = kart_odom.dist_sum;        /* 里程基准:dist_buf 记相对增量 */

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
    kart_odom_snapshot_t kart_odom;
    float dx, dy, cx, cy, cyaw;
    float origin_rad, origin_sin, origin_cos;
    uint16 i;

    if(!record_running)
    {
        return;
    }

    /* 按 START 收尾时强制记住当前终点。尤其要把停车后的最终回正打角
     * 写进 steer_buf；常规 poll 按里程/航向阈值采样，车不动时不会自动记这一次。 */
    kart_odom_get_snapshot(&kart_odom);
    dx = kart_odom.x - origin_x;
    dy = kart_odom.y - origin_y;
    origin_rad = origin_yaw * 0.01745329252f;
    origin_sin = sinf(origin_rad);
    origin_cos = cosf(origin_rad);
    cx   =  origin_cos * dx + origin_sin * dy;
    cy   = -origin_sin * dx + origin_cos * dy;
    cyaw = kart_record_yaw_diff(kart_odom.yaw, origin_yaw);

    /* 与末点几乎重合就覆盖末点；否则追加一点。缓冲已满时也覆盖最后一点，
     * 保证真实终点优先于倒数第二个采样点。 */
    i = (record_count > 0U) ? (uint16)(record_count - 1U) : 0U;
    if(record_count > 0U)
    {
        float ex = cx - record_buf[i].x;
        float ey = cy - record_buf[i].y;
        float eyaw = fabsf(kart_record_yaw_diff(cyaw, record_buf[i].yaw));
        if((ex * ex + ey * ey) >= 0.000001f || eyaw >= 0.01f)
        {
            if(record_count < KART_RECORD_MAX_WAYPOINTS)
            {
                i = record_count++;
            }
        }
    }

    record_buf[i].x       = cx;
    record_buf[i].y       = cy;
    record_buf[i].yaw     = cyaw;
    record_buf[i].v_left  = kart_control_get_left_meas();
    record_buf[i].v_right = kart_control_get_right_meas();
    steer_buf[i]          = kart_steer_abs_get_center_delta();
    dist_buf[i]           = kart_odom.dist_sum - origin_dist;

    last_x = cx;
    last_y = cy;
    last_yaw = cyaw;
    record_running = 0;
}

void kart_record_poll(void)
{
    float cx, cy, cyaw, dx, dy, dyaw, dist;
    float origin_rad, origin_sin, origin_cos;
    kart_odom_snapshot_t kart_odom;

    if(!record_running) return;
    /* 【录满即静默停录】没有溢出标志也没有返回码:调用者只能看到
     * kart_record_is_running() 从 1 变 0,与正常按键停录无法区分,现场唯一迹象是
     * 点数恰好卡在 KART_RECORD_MAX_WAYPOINTS。1500 点 × 0.05m = 直线 75m 封顶,
     * 弯道更短,而跟随段实测 65m —— 是够用但不宽裕,长路线要先加大那个宏。
     * 【为什么不在这儿补个标志】赛后不动运行时行为;真要报,该在菜单那层显示
     * 点数,而点数已经显示了。 */
    if(record_count >= KART_RECORD_MAX_WAYPOINTS)
    {
        record_running = 0;
        return;
    }

    /* 一致快照:x/y/yaw 一次性取整帧,避免被 5ms 中断插到半路取到撕裂位姿。 */
    kart_odom_get_snapshot(&kart_odom);
    dx = kart_odom.x - origin_x;
    dy = kart_odom.y - origin_y;

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
    cyaw = kart_record_yaw_diff(kart_odom.yaw, origin_yaw);

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
    /* 科目三开环回放:同帧记打角 + 相对起点累计里程(与该点坐标严格对齐)。 */
    steer_buf[record_count] = kart_steer_abs_get_center_delta();
    dist_buf[record_count]  = kart_odom.dist_sum - origin_dist;
    record_count++;

    last_x   = cx;
    last_y   = cy;
    last_yaw = cyaw;
}

uint8  kart_record_is_running(void)          { return record_running; }
uint16 kart_record_get_count(void)           { return record_count; }
const  kart_waypoint_t* kart_record_get_waypoints(void) { return record_buf; }

uint8 kart_record_adjust_segment(uint16 center, uint16 radius, float dx, float dy)
{
    uint16 first, last, i;
    uint8 changed = 0;

    if(record_running || record_count < 2U || center >= record_count || radius == 0U)
        return 0;

    first = (center > radius) ? (uint16)(center - radius) : 0U;
    last  = ((uint32)center + radius < record_count)
          ? (uint16)(center + radius) : (uint16)(record_count - 1U);

    for(i = first; i <= last; i++)
    {
        uint16 d;
        float weight;

        if(i == 0U) continue;                 /* 录制坐标原点必须保持 (0,0) */
        /* 倒车段实际按 kart_steer/dist/yaw 开环回放，不按 x/y 追踪。改它的坐标只会
         * 让屏幕看起来变了、车辆动作却不变，因此红色倒车点明确锁住。
         * 【这里的门限比 kart_playback 判倒车的门限松 100 倍】本处 -0.02 脉冲/5ms,
         * 而 kart_playback 要低于 KART_PLAYBACK_REV_SPEED_EPS = 2.0 才切开环回放。
         * 于是 -2.0 ~ -0.02 这一段里的点:编辑器锁住不让拖,复现时却仍走 Pure
         * Pursuit 当前进点。方向是安全的那一边(锁多了,不会把该锁的放过去),
         * 所以只记不改。真要对齐就把本处改成引用那个宏,别再写字面量。 */
        if(0.5f * (record_buf[i].v_left + record_buf[i].v_right) < -0.02f)
            continue;
        d = (i > center) ? (uint16)(i - center) : (uint16)(center - i);
        weight = (float)(radius + 1U - d) / (float)(radius + 1U);
        record_buf[i].x += dx * weight;
        record_buf[i].y += dy * weight;
        changed = 1;
    }
    return changed;
}

float  kart_record_get_origin_yaw(void)      { return origin_yaw; }
/* 录制起点的世界坐标。科目三位置闭环倒车要把当前 kart_odom 位置投影回录制坐标系,
 * 光有 origin_yaw 不够(那只给旋转),还要这两个做平移基准。 */
float  kart_record_get_origin_x(void)        { return origin_x; }
float  kart_record_get_origin_y(void)        { return origin_y; }

/* 科目三开环回放取数据(与 record_buf 同索引):打角数组 / 累计里程数组 / 总里程。 */
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

    /* 起点位姿一起存:它是倒车段航向纠偏的参考系原点,不存等于载入后参考系是别人的。 */
    kart_flash_meta_t meta;
    meta.origin_yaw = origin_yaw;
    meta.origin_x   = origin_x;
    meta.origin_y   = origin_y;

    return kart_flash_save_path(slot, record_buf, steer_buf, dist_buf,
                                &meta, record_count);
}

uint16 kart_record_load_from_flash(uint8 slot)
{
    if(record_running) return 0;                /* 录制进行中不覆盖缓冲 */

    kart_flash_meta_t meta;
    uint16 n = kart_flash_load_path(slot, record_buf, steer_buf, dist_buf,
                                    &meta, KART_RECORD_MAX_WAYPOINTS);
    record_count = n;                           /* 覆盖当前缓冲,供 kart_playback 读 */

    if(n > 0)
    {
        /* 起点位姿也一并还原:kart_playback 倒车段用 origin_yaw 当参考系原点,
         * 不还原就还是上次录制的值,整段参考航向偏多少车就跟着偏多少。
         * 【坐标系前提】kart_odom 的 yaw 零点由上电时 IMU 姿态决定,重启后世界系会变。
         * 因此"载入槽位再复现"要求发车朝向与录制那次一致(与录制后直接复现同理),
         * 这一条没变,本改动只是不再额外叠加"参考系拿错"这个二次误差。 */
        origin_yaw = meta.origin_yaw;
        origin_x   = meta.origin_x;
        origin_y   = meta.origin_y;

        /* last_* 是录制增量采样的游标。载入不是录制,但把它对到末点,
         * 万一现场在载入后又按了录制续录,不会从 (0,0) 突跳出一条假直线。 */
        last_x   = record_buf[n - 1].x;
        last_y   = record_buf[n - 1].y;
        last_yaw = record_buf[n - 1].yaw;
    }
    return n;
}

#if defined(__TASKING__)
#pragma section all restore
#endif
