#include "kart_motion.h"
#include "kart_voice.h"
#include "kart_odom.h"
#include "kart_imu.h"
#include "kart_calc.h"
#include "kart_steer_ctrl.h"
#include "kart_control.h"
#include "kart_remote.h"
#include <math.h>

/*
 * 科目二语音运动控制实现 —— 见 kart_motion.h 头注释。
 * 状态机每拍推进,只设目标(转角/航向/速度)+使能,实际输出由主循环控制环产生。
 */

/* -------------------- 内部状态机 -------------------- */
typedef enum
{
    MOTION_IDLE = 0,
    MOTION_FWD,             /* 直行前进(航向环保向) */
    MOTION_BACK,            /* 直行后退(内环锁中位) */
    MOTION_SNAKE_FWD,       /* 蛇形前进 */
    MOTION_SNAKE_BACK,      /* 蛇形后退 */
    MOTION_CIRCLE,          /* 转圈(固定打角,累计 yaw 到 360) */
    MOTION_TURN_APPROACH,   /* 左右转:先直行 approach */
    MOTION_TURN_ROTATE,     /* 左右转:原地转向到目标角 */
} motion_phase_t;

static motion_phase_t motion_phase = MOTION_IDLE;

static float motion_dist0   = 0.0f;     /* 起始累计路程基准(米) */
static float motion_yaw0    = 0.0f;     /* 起始航向(度,直行保向用) */
static float motion_yaw_prev = 0.0f;    /* 上一拍航向(累计 yaw 用) */
static float motion_yaw_accum = 0.0f;   /* 累计转过角度(度,绝对值判完成) */

static float motion_speed   = 0.0f;     /* 本动作速度(带符号,脉冲/5ms) */
static float motion_snake_delta = 0.0f; /* 蛇形当前打角(翻转) */
static float motion_turn_delta  = 0.0f; /* 转弯打角(带方向符号) */

/* 累计 yaw:把本拍航向增量(wrap 到 ±180)累加,不受 ±180 回绕影响。 */
static void motion_accum_yaw(void)
{
    float now = kart_imu_get_yaw();
    motion_yaw_accum += get_relative_angle(motion_yaw_prev, now);   /* now-prev,规整±180 */
    motion_yaw_prev = now;
}

/* -------------------- 对外接口 -------------------- */
void kart_motion_init(void)
{
    motion_phase = MOTION_IDLE;
}

uint8 kart_motion_is_busy(void)
{
    return (motion_phase != MOTION_IDLE) ? 1 : 0;
}

void kart_motion_stop(void)
{
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);
    kart_steer_set_target_delta(0.0f);
    motion_phase = MOTION_IDLE;
}

uint8 kart_motion_start(uint8 voice_cmd)
{
    if(kart_motion_is_busy())
    {
        return 0;                       /* 忙不打断,交给队列串行 */
    }

    /* 公共基准:路程/航向清基准,累计 yaw 归零。 */
    motion_dist0     = kart_odom_get_dist();
    motion_yaw0      = kart_imu_get_yaw();
    motion_yaw_prev  = motion_yaw0;
    motion_yaw_accum = 0.0f;

    switch(voice_cmd)
    {
        case KART_VOICE_CMD_FWD_10M:
            motion_speed = +KART_MOTION_SPEED;
            /* 航向环保向:锁当前 yaw,速度环给正速直行。 */
            kart_steer_set_head_enable(1);
            kart_steer_set_target_yaw(motion_yaw0);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_FWD;
            break;

        case KART_VOICE_CMD_BACK_10M:
            motion_speed = -KART_MOTION_SPEED;
            /* 倒车不用航向环(正反馈发散),内环锁中位 + 负速直倒(同科目一倒库)。 */
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            kart_steer_set_target_delta(0.0f);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_BACK;
            break;

        case KART_VOICE_CMD_SNAKE_FWD_10M:
            motion_speed = +KART_MOTION_SPEED;
            motion_snake_delta = +KART_MOTION_SNAKE_DELTA;
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            kart_steer_set_target_delta(motion_snake_delta);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_SNAKE_FWD;
            break;

        case KART_VOICE_CMD_SNAKE_BACK_10M:
            motion_speed = -KART_MOTION_SPEED;
            motion_snake_delta = +KART_MOTION_SNAKE_DELTA;
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            kart_steer_set_target_delta(motion_snake_delta);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_SNAKE_BACK;
            break;

        case KART_VOICE_CMD_CCW_CIRCLE:
            motion_speed = +KART_MOTION_SPEED;
            /* 逆时针:固定打左角(+),边走边累计 yaw 到 360°。 */
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            kart_steer_set_target_delta(+KART_MOTION_CIRCLE_DELTA);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_CIRCLE;
            break;

        case KART_VOICE_CMD_CW_CIRCLE:
            motion_speed = +KART_MOTION_SPEED;
            /* 顺时针:固定打右角(-)。 */
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            kart_steer_set_target_delta(-KART_MOTION_CIRCLE_DELTA);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_CIRCLE;
            break;

        case KART_VOICE_CMD_TURN_LEFT:
            motion_speed = +KART_MOTION_SPEED;
            motion_turn_delta = +KART_MOTION_TURN_DELTA;    /* 左转打左角(+) */
            /* 先直行 approach(>2m),保向走直线。 */
            kart_steer_set_head_enable(1);
            kart_steer_set_target_yaw(motion_yaw0);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_TURN_APPROACH;
            break;

        case KART_VOICE_CMD_TURN_RIGHT:
            motion_speed = +KART_MOTION_SPEED;
            motion_turn_delta = -KART_MOTION_TURN_DELTA;    /* 右转打右角(-) */
            kart_steer_set_head_enable(1);
            kart_steer_set_target_yaw(motion_yaw0);
            kart_control_set_enable(1);
            kart_control_set_target(motion_speed);
            motion_phase = MOTION_TURN_APPROACH;
            break;

        default:
            return 0;                   /* 非运动命令 */
    }

    return 1;
}

void kart_motion_update(void)
{
    if(motion_phase == MOTION_IDLE)
    {
        return;
    }

    /* deadman 急停:遥控失联或三段拨低挡 → 立即停(与 playback 一致)。 */
    if(!kart_remote_is_online() ||
       kart_remote_get_sw3() == KART_REMOTE_SW3_L)
    {
        kart_motion_stop();
        return;
    }

    float dist = kart_odom_get_dist() - motion_dist0;

    switch(motion_phase)
    {
        case MOTION_FWD:
            /* 保向由航向环持续维持(target_yaw 已设),到路程停。 */
            if(dist >= KART_MOTION_FWD_DIST)
            {
                kart_motion_stop();
            }
            break;

        case MOTION_BACK:
            /* 内环持续锁中位,到路程停。 */
            kart_steer_set_target_delta(0.0f);
            if(dist >= KART_MOTION_BACK_DIST)
            {
                kart_motion_stop();
            }
            break;

        case MOTION_SNAKE_FWD:
        case MOTION_SNAKE_BACK:
        {
            /* 每 HALF_DIST 翻转一次打角,形成左右摆动;到总路程停。 */
            float seg = fmodf(dist, 2.0f * KART_MOTION_SNAKE_HALF_DIST);
            float delta = (seg < KART_MOTION_SNAKE_HALF_DIST)
                          ? +KART_MOTION_SNAKE_DELTA : -KART_MOTION_SNAKE_DELTA;
            kart_steer_set_target_delta(delta);

            if(dist >= KART_MOTION_SNAKE_DIST)
            {
                kart_motion_stop();
            }
            break;
        }

        case MOTION_CIRCLE:
            /* 固定打角边走边累计 yaw,到一整圈停。 */
            motion_accum_yaw();
            if(fabsf(motion_yaw_accum) >= KART_MOTION_CIRCLE_ANGLE)
            {
                kart_motion_stop();
            }
            break;

        case MOTION_TURN_APPROACH:
            /* 直行到 approach 距离,切原地转向段:重置 yaw 累计基准,打转弯角。 */
            if(dist >= KART_MOTION_TURN_APPROACH)
            {
                motion_yaw_prev  = kart_imu_get_yaw();
                motion_yaw_accum = 0.0f;
                kart_steer_set_head_enable(0);
                kart_steer_set_angle_enable(1);
                kart_steer_set_target_delta(motion_turn_delta);
                motion_phase = MOTION_TURN_ROTATE;
            }
            break;

        case MOTION_TURN_ROTATE:
            /* 保持转弯打角,累计 yaw 到目标角(方向转正)停。 */
            motion_accum_yaw();
            kart_steer_set_target_delta(motion_turn_delta);
            if(fabsf(motion_yaw_accum) >= KART_MOTION_TURN_ANGLE)
            {
                kart_motion_stop();
            }
            break;

        default:
            kart_motion_stop();
            break;
    }
}
