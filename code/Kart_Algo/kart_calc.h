#ifndef KART_CALC_H_
#define KART_CALC_H_
#include "zf_common_headfile.h"

/*
 * 卡丁车几何计算库
 * 移植自 TopSpeed 的 Calcustom.c / GPS.c 里的纯数学部分
 * ------------------------------------------------------------------
 * 移植原则:照搬。结构体名、字段、函数名全部保持与 TopSpeed 一致,
 * 一个字不改。这样后面搬 GPS.c / Subject_1.c 时不用改任何名字。
 * (原工程注释是乱码的 GBK,这里换成干净中文,但类型和字段名一字未动)
 * ------------------------------------------------------------------
 */

/* -------------------------------------------------------------------------
 * 三维向量(四元数/姿态解算用)
 * ------------------------------------------------------------------------- */
typedef struct{float x;float y;float z;}                                        Vec3;   // 三维向量

/* -------------------------------------------------------------------------
 * 四元数(姿态解算内部表示,Madgwick 算法用)
 * 说明:a 是实部,b/c/d 是虚部(对应 i/j/k)。姿态用四元数存,避免欧拉角万向锁。
 * ------------------------------------------------------------------------- */
typedef struct{float a;float b;float c;float d;}                                QUAT;   // 四元数

/* -------------------------------------------------------------------------
 * 欧拉角(度)。惯导只用 yaw(航向),roll/pitch 保留照搬。
 * ------------------------------------------------------------------------- */
typedef struct{
    float roll;     // 横滚角(绕 X)
    float pitch;    // 俯仰角(绕 Y)
    float yaw;      // 方位角/航向(绕 Z)
}                                                                               EulerAngles;

/* -------------------------------------------------------------------------
 * GPS 原始经纬度点(惯导用不到经纬度,但照搬保留,以后接 GPS 校正直接能用)
 * ------------------------------------------------------------------------- */
typedef struct{
    double lat_ff;  // 纬度(浮点度)
    double lon_ff;  // 经度(浮点度)

    uint32_t lat_d; // 纬度(整数,×1e7)
    uint32_t lon_d; // 经度(整数,×1e7)
}                                                                               GPS_Point_struct;

/* -------------------------------------------------------------------------
 * 本地平面坐标点(x=东向, y=北向, 单位米)
 * 说明:点和向量共用同一个结构体。Point_2D、Vec2 都是它的别名。
 * ------------------------------------------------------------------------- */
typedef struct{
    float x;    // 经向(东)
    float y;    // 纬向(北)
}                                                                               GPS_local_Point_struct;
typedef GPS_local_Point_struct                                                  Point_2D;
typedef GPS_local_Point_struct                                                  Vec2;

/* -------------------------------------------------------------------------
 * 一条直线(两个端点 + 标准式 ax+by+c=0 系数)
 * 说明:用 ax+by+c=0 而不是 y=kx+b,因为竖直线斜率无穷大会算爆。
 * ------------------------------------------------------------------------- */
typedef struct{
    GPS_local_Point_struct start;   // 起点
    GPS_local_Point_struct end;     // 终点
    uint8_t same_point;             // 起终点几乎重合标志(=1 表示这条线无效)

    float a;                        // 标准式系数 a
    float b;                        // 标准式系数 b
    float c;                        // 标准式系数 c
}                                                                               straight_line_struct;

/* --- 角度 / 弧度互转(Calcustom.c) --- */
float degree_to_rad(float degree);
float rad_to_degree(float rad);

/* --- 绝对值(Calcustom.c) --- */
float abs_float(float num);

/* --- 快速平方根倒数 1/sqrt(x)(Calcustom.c,Madgwick 算法用) --- */
float invSqrt(float x);

/* --- 四元数 → 欧拉角(度)(Calcustom.c) --- */
EulerAngles quaternionToEuler(QUAT q);

/* --- 两点方位角与距离(GPS.c) --- */
float get_angle(GPS_local_Point_struct now, GPS_local_Point_struct aim);        // now 指向 aim 的方位角(度,-180~180)
float get_distance(GPS_local_Point_struct now, GPS_local_Point_struct aim);     // 两点欧氏距离(米)

/* --- 角度差,处理 ±180 突变(GPS.c) --- */
float get_relative_angle(float now, float aim);                                 // aim-now,规整到 [-180,180]

/* --- 直线相关(GPS.c)。以下三个目前无调用者,备用件,详见 kart_calc.c --- */
straight_line_struct draw_straight_line(GPS_local_Point_struct start, GPS_local_Point_struct end);      // 由两点造直线
float point_to_straight_line_distance(straight_line_struct line, GPS_local_Point_struct Point_2D);      // 点到直线带符号垂距
int   get_point_to_line_dir(straight_line_struct line, GPS_local_Point_struct Point_2D);                // 点在直线哪一侧(+1/-1)

#endif
