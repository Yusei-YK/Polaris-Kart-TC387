#include "kart_record.h"
#include "kart_flash.h"
#include "kart_odom.h"
#include "kart_control.h"
#include <math.h>

static kart_waypoint_t record_buf[KART_RECORD_MAX_WAYPOINTS];
static uint16 record_count  = 0;
static uint8  record_running = 0;

static float origin_x   = 0.0f;
static float origin_y   = 0.0f;
static float origin_yaw = 0.0f;

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
    record_count = 0;
    record_running = 1;

    origin_x   = kart_odom_get_x();
    origin_y   = kart_odom_get_y();
    origin_yaw = kart_odom_get_yaw();

    last_x   = 0.0f;
    last_y   = 0.0f;
    last_yaw = 0.0f;

    kart_waypoint_t *wp = &record_buf[0];
    wp->x       = 0.0f;
    wp->y       = 0.0f;
    wp->yaw     = 0.0f;
    wp->v_left  = kart_control_get_left_meas();
    wp->v_right = kart_control_get_right_meas();
    record_count = 1;
}

void kart_record_stop(void)
{
    record_running = 0;
}

void kart_record_poll(void)
{
    float cx, cy, cyaw, dx, dy, dyaw, dist;

    if(!record_running) return;
    if(record_count >= KART_RECORD_MAX_WAYPOINTS)
    {
        record_running = 0;
        return;
    }

    cx   = kart_odom_get_x() - origin_x;
    cy   = kart_odom_get_y() - origin_y;
    cyaw = kart_record_yaw_diff(kart_odom_get_yaw(), origin_yaw);

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
    record_count++;

    last_x   = cx;
    last_y   = cy;
    last_yaw = cyaw;
}

uint8  kart_record_is_running(void)          { return record_running; }
uint16 kart_record_get_count(void)           { return record_count; }
const  kart_waypoint_t* kart_record_get_waypoints(void) { return record_buf; }

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
