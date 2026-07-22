#include "kart_playback.h"
#include "kart_record.h"
#include "kart_odom.h"
#include "kart_calc.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"
#include "kart_remote.h"        /* 复现期间遥控急停(deadman:失联或低挡即停) */
#include <math.h>

static uint8  playback_running = 0;
static uint16 playback_index   = 0;      /* 当前跟踪到的最近路径点索引 */
static float  playback_target_yaw = 0.0f;

static float origin_x = 0.0f;
static float origin_y = 0.0f;

void kart_playback_init(void)
{
    playback_running   = 0;
    playback_index     = 0;
    playback_target_yaw = 0.0f;
}

void kart_playback_start(void)
{
    if(kart_record_get_count() < 2) return;     /* 没录到有效路径,不启动 */

    origin_x = kart_odom_get_x();
    origin_y = kart_odom_get_y();

    playback_index     = 0;
    playback_running   = 1;

    kart_steer_set_head_enable(1);              /* 开航向外环(连带开内环) */
    kart_control_set_enable(1);                 /* 开速度环 */
}

void kart_playback_stop(void)
{
    playback_running = 0;
    kart_control_set_enable(0);                 /* 停后轮 */
    kart_steer_set_head_enable(0);              /* 关转向串级 */
    kart_control_set_target(0.0f);
}

/* Pure Pursuit 一拍:找前视目标点 → 方位角作航向目标 → 速度喂速度环 */
void kart_playback_poll(void)
{
    Point_2D cur;
    uint16 n, i;
    const kart_waypoint_t *wp;
    float lookahead_yaw;
    float target_v;

    if(!playback_running) return;

    /* 遥控急停(deadman):遥控失联或三段拨到低挡 → 立即收车。
     * 只读在线态/挡位,不读油门/方向,故不干扰复现的转向和速度。
     * 放在这里而非科目状态机:playback_poll 在主循环无条件每拍调,
     * 无论 b1 直发(mission=IDLE)还是科目一绕桩都统一生效。 */
    if(!kart_remote_is_online() || kart_remote_get_sw3() == KART_REMOTE_SW3_L)
    {
        kart_playback_stop();
        return;
    }

    n = kart_record_get_count();
    wp = kart_record_get_waypoints();

    /* 当前位置换成"相对复现起点"坐标,和录制点同一坐标系 */
    cur.x = kart_odom_get_x() - origin_x;
    cur.y = kart_odom_get_y() - origin_y;

    /* 到终点判定:离最后一个点足够近就收车 */
    {
        Point_2D last;
        last.x = wp[n - 1].x;
        last.y = wp[n - 1].y;
        if(get_distance(cur, last) < KART_PLAYBACK_FINISH_DIST)
        {
            kart_playback_stop();
            return;
        }
    }

    /* 从当前索引往后找第一个"距离 >= 前视距离"的点作为目标点。
     * playback_index 单调前进,避免抄近路跳回已过路段。 */
    for(i = playback_index; i < n; i++)
    {
        Point_2D p;
        p.x = wp[i].x;
        p.y = wp[i].y;
        if(get_distance(cur, p) >= KART_PLAYBACK_LOOKAHEAD)
        {
            playback_index = i;
            break;
        }
    }
    if(i >= n)
    {
        playback_index = n - 1;         /* 前视点越界,盯住终点 */
    }

    {
        Point_2D aim;
        aim.x = wp[playback_index].x;
        aim.y = wp[playback_index].y;
        lookahead_yaw = get_angle(cur, aim);        /* 当前位置指向目标点的方位角(度) */
    }
    playback_target_yaw = lookahead_yaw;

    kart_steer_set_target_yaw(lookahead_yaw);       /* 喂航向外环 */

    /* 速度跟随录制点:录制用遥控油门控制,速度精准,直接复用左右轮均值喂速度环。
     * 手推是备用方案。KART_PLAYBACK_SPEED_MAX 做上限钳位,防录制毛刺冲出。 */
    target_v = 0.5f * (wp[playback_index].v_left + wp[playback_index].v_right);
    if(target_v >  KART_PLAYBACK_SPEED_MAX) target_v =  KART_PLAYBACK_SPEED_MAX;
    if(target_v < -KART_PLAYBACK_SPEED_MAX) target_v = -KART_PLAYBACK_SPEED_MAX;
    kart_control_set_target(target_v);
}

uint8  kart_playback_is_running(void)     { return playback_running; }
uint16 kart_playback_get_index(void)      { return playback_index; }
float  kart_playback_get_target_yaw(void) { return playback_target_yaw; }
