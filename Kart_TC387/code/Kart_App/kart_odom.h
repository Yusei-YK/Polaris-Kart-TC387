#ifndef KART_ODOM_H_
#define KART_ODOM_H_

#include "zf_common_headfile.h"
#include "kart_calib.h"     /* KART_ODOM_YAW_SIGN、脉冲当量 */
#include "kart_calc.h"

/*
 * 卡丁车航位推算库(dead reckoning / 惯导积分)
 * ------------------------------------------------------------------
 * 移植自 TopSpeed 的 INS_POS_EST(TopSpeed_GPS_INS.c:607-653)。
 * 只搬"纯航位推算"这一段:用 IMU 航向 yaw + 编码器累计脉冲,
 * 每拍把位移增量投影到本地平面坐标(x=东,y=北),累加出 (x,y) 轨迹。
 * 不搬 GPS 互补滤波 / 卡尔曼融合(卡丁车科目一先纯惯导,没 GPS 校正)。
 * ------------------------------------------------------------------
 * 坐标约定(照搬 TopSpeed):
 *   dx = -sin(yaw) * ds     dy = +cos(yaw) * ds
 *   => yaw=0 时车头朝 +Y(北),yaw 增大朝 -X(西)转。
 *   若发现 X 轴镜像/转向反了,先翻 KART_ODOM_YAW_SIGN,再考虑改这两行符号。
 * ------------------------------------------------------------------
 * 两个必须标定的量:
 *   1. KART_LEFT/RIGHT_ENC_PULSE_TO_M:左右编码器各自一个脉冲对应多少米。
 *      左右原始计数分辨率不同,必须先分别换算成米,不能直接平均脉冲。
 *   2. KART_ODOM_YAW_SIGN:IMU yaw 正方向与坐标系是否一致(+1/-1)。
 * ------------------------------------------------------------------
 * 调用位置:kart_odom_update() 放 5ms 定时中断里,紧跟 kart_imu_update()
 *          和 kart_encoder_update() 之后。本模块只读编码器累计和,
 *          绝不调用 kart_encoder_update()(那会偷走计数、搞坏速度环)。
 */

/* IMU 航向正负号 KART_ODOM_YAW_SIGN → kart_calib.h 第六节(控制环反馈符号) */

typedef struct
{
    Point_2D pos_now;       // 当前位置(米):x=东, y=北
    float    yaw_now;       // 最近一次用于积分的航向(度)
    float    dist_sum;      // 累计路程(米,标量,不含方向)
    float    distance_last; // 上拍左右轮换算后的车体中心累计距离(米)
    float    distance_now;  // 本拍左右轮换算后的车体中心累计距离(米)
    uint8    active;        // =1 才积分(标定/静止时可置 0 冻结)
} kart_odom_t;

extern kart_odom_t kart_odom;

/* 一致位姿快照:x/y/yaw 一次性打包取出,避免分三次 get 期间被 5ms 中断改到
 * 半路(取到旧 x + 新 yaw 这类撕裂值)。复现/路径跟踪按整帧位姿算才对得上。*/
typedef struct
{
    float x;        // 东向(米)
    float y;        // 北向(米)
    float yaw;      // 航向(度)
    float dist_sum; // 累计路程(米)
} kart_odom_snapshot_t;

/* --- 对外接口 --- */
void     kart_odom_init(void);                          // 复位位置/脉冲基准
void     kart_odom_update(void);                        // 放 5ms 中断,读 yaw+脉冲积分
void     kart_odom_reset(void);                         // 位置清零(脉冲基准同步刷新)
void     kart_odom_set_origin(float x, float y);        // 设当前点为指定坐标(打点用)
void     kart_odom_set_active(uint8 on);                // 冻结/恢复积分

Point_2D kart_odom_get_pos(void);                       // 取当前 (x,y)
float    kart_odom_get_x(void);
float    kart_odom_get_y(void);
float    kart_odom_get_yaw(void);                        // 最近积分用的航向(度)
float    kart_odom_get_dist(void);                       // 累计路程(米)
void     kart_odom_get_snapshot(kart_odom_snapshot_t *snap);  // 一次性取整帧位姿(x/y/yaw/路程)

#endif
