#include "kart_odom.h"
#include "kart_imu.h"
#include "kart_encoder.h"
#include <math.h>

#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#endif

kart_odom_t kart_odom = {0};

/* 左右编码器分辨率不同,必须先各自换算成米,再取车体中心行程。 */
static float kart_odom_read_center_distance(void)
{
    float left_m = (float)kart_encoder_get_left_sum() * KART_LEFT_ENC_PULSE_TO_M;
    float right_m = (float)kart_encoder_get_right_sum() * KART_RIGHT_ENC_PULSE_TO_M;
    return (left_m + right_m) * 0.5f;
}

void kart_odom_init(void)
{
    kart_odom.pos_now.x = 0.0f;
    kart_odom.pos_now.y = 0.0f;
    kart_odom.yaw_now   = 0.0f;
    kart_odom.dist_sum  = 0.0f;
    kart_odom.distance_last = kart_odom_read_center_distance();
    kart_odom.distance_now  = kart_odom.distance_last;
    kart_odom.active    = 1;
}

void kart_odom_reset(void)
{
    kart_odom.pos_now.x = 0.0f;
    kart_odom.pos_now.y = 0.0f;
    kart_odom.dist_sum  = 0.0f;
    kart_odom.distance_last = kart_odom_read_center_distance();
    kart_odom.distance_now  = kart_odom.distance_last;
}

void kart_odom_set_origin(float x, float y)
{
    kart_odom.pos_now.x = x;
    kart_odom.pos_now.y = y;
    kart_odom.distance_last = kart_odom_read_center_distance();
    kart_odom.distance_now  = kart_odom.distance_last;
}

void kart_odom_set_active(uint8 on)
{
    if(on)
    {
        /* 恢复积分前先把脉冲基准对齐,避免把冻结期间累计的脉冲一次性吃进来 */
        kart_odom.distance_last = kart_odom_read_center_distance();
    }
    kart_odom.active = on ? 1 : 0;
}

/* 航位推算一拍:左右脉冲各自换算成米后求车体中心 ds,按航向投影到 (x,y) 累加。
 * 照搬 TopSpeed INS_POS_EST_update:dx=-sin(yaw)*ds, dy=+cos(yaw)*ds。*/
void kart_odom_update(void)
{
    float yaw;
    float ds;
    float rad;

    if(!kart_odom.active)
    {
        /* 冻结:只跟踪脉冲基准,不积分位置 */
        kart_odom.distance_last = kart_odom_read_center_distance();
        return;
    }

    yaw = KART_ODOM_YAW_SIGN * kart_imu_get_yaw();
    kart_odom.yaw_now = yaw;

    kart_odom.distance_now = kart_odom_read_center_distance();
    ds = kart_odom.distance_now - kart_odom.distance_last;
    kart_odom.distance_last = kart_odom.distance_now;

    rad = degree_to_rad(yaw);
    kart_odom.pos_now.x += -sinf(rad) * ds;
    kart_odom.pos_now.y +=  cosf(rad) * ds;

    kart_odom.dist_sum += fabsf(ds);
}

Point_2D kart_odom_get_pos(void)  { return kart_odom.pos_now; }
float    kart_odom_get_x(void)    { return kart_odom.pos_now.x; }
float    kart_odom_get_y(void)    { return kart_odom.pos_now.y; }
float    kart_odom_get_yaw(void)  { return kart_odom.yaw_now; }
float    kart_odom_get_dist(void) { return kart_odom.dist_sum; }

/* 一致位姿快照:x/y/yaw/路程一次性打包取出。
 * kart_odom_update() 跑在 5ms 中断(cc60_pit_ch0_isr),消费者(kart_playback_poll/
 * kart_steer_ctrl)跑在主循环,同核。分开 get 会被中断插到半路取到撕裂值(旧 x+新 yaw)。
 * 关中断复制四字段 → 拿到整帧一致位姿,再恢复。复制只几条指令,关中断窗口极短。 */
void kart_odom_get_snapshot(kart_odom_snapshot_t *snap)
{
    uint32 primask;

    if(snap == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    snap->x        = kart_odom.pos_now.x;
    snap->y        = kart_odom.pos_now.y;
    snap->yaw      = kart_odom.yaw_now;
    snap->dist_sum = kart_odom.dist_sum;
    interrupt_global_enable(primask);
}

#if defined(__TASKING__)
#pragma section all restore
#endif
