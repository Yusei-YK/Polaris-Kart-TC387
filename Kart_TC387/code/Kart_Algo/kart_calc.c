#include "kart_calc.h"
#include "math.h"

/* 有的编译器 math.h 不带 M_PI(C99 标准里 M_PI 不是必须的),这里兜底补一个 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * 卡丁车几何计算库 —— 实现
 * ------------------------------------------------------------------
 * 移植自 TopSpeed 的 Calcustom.c / GPS.c,函数体一字不改照搬,
 * 仅把原工程的 GBK 乱码注释换成干净中文。
 * 这些函数全是纯数学,不依赖任何硬件,可在 PC 上单独测试。
 * ------------------------------------------------------------------
 */

/* =========================== 角度 / 弧度互转 =========================== */
/* 移植自 Calcustom.c */
float degree_to_rad(float degree){return 0.0174533*degree;}     // 度 → 弧度 (×π/180)
float rad_to_degree(float rad){return 57.295779513*rad;}        // 弧度 → 度 (×180/π)

/* =========================== 浮点绝对值 =========================== */
/* 移植自 Calcustom.c */
float abs_float(float num){return num>=0?num:-num;}

/* =========================== 快速平方根倒数 1/sqrt(x) =========================== */
/* 移植自 Calcustom.c(著名的 Quake III 快速平方根倒数算法)
 * 用途:四元数归一化每次都要算 1/sqrt(...),用这个比标准库 1.0f/sqrtf() 快很多。
 * 原理:先用位运算给出一个很接近的初值,再用一次牛顿迭代把误差压下去。
 *   - *(uint32_t*)&x  : 把 float 的 4 个字节"当成"一个 uint32 整数来读(不做数值转换,只借位模式)
 *   - >> 1            : 右移一位,相当于把指数部分近似减半 → 这就是 sqrt 的雏形
 *   - 0x5F1F1412      : 魔数,专门调出来让初值尽量准
 *   - tmp*(1.69.. - 0.71..*x*tmp*tmp) : 一次牛顿迭代收敛
 * 注意:这里用指针把 float 和 uint32 互相"重解释",是这个算法的精髓,别改。 */
float invSqrt(float x){
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);            // 位运算算出接近的初值
   float tmp = *(float*)&i;                                    // 把整数位模式再当 float 读回来
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);  // 牛顿迭代优化
}

/* =========================== 四元数 → 欧拉角(度) =========================== */
/* 移植自 Calcustom.c
 * 把姿态解算得到的四元数 q(a 实部,b/c/d 虚部)转成人能看懂的 roll/pitch/yaw。
 * 惯导只用 yaw(航向),roll/pitch 一起算出来照搬保留。
 * 结构体传参:C 里结构体是"整个拷贝"进来的,函数里改 q 不影响外面的原件。 */
EulerAngles quaternionToEuler(QUAT q) {
    EulerAngles angles;

    /* 横滚角 Roll(绕 X 轴) */
    float sinr_cosp = 2 * (q.a * q.b + q.c * q.d);
    float cosr_cosp = 1 - 2 * (q.b * q.b + q.c * q.c);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    /* 俯仰角 Pitch(绕 Y 轴) */
    float sinp = 2 * (q.a * q.c - q.d * q.b);
    if (fabs(sinp) >= 1)                            // 防 asin 定义域越界(参数只能在 -1~1)
        angles.pitch = copysignf(M_PI / 2, sinp);   // 越界就钳到 ±90 度
    else
        angles.pitch = asinf(sinp);

    /* 方位角 Yaw(绕 Z 轴)—— 惯导航向就是它 */
    float siny_cosp = 2 * (q.a * q.d + q.b * q.c);
    float cosy_cosp = 1 - 2 * (q.c * q.c + q.d * q.d);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    /* 弧度 → 度 */
    angles.pitch*=180.0f/M_PI;
    angles.roll*=180.0f/M_PI;
    angles.yaw*=180.0f/M_PI;

    return angles;
}

/* =========================== 两点方位角 =========================== */
/* 移植自 GPS.c
 * 返回:从 now 指向 aim 的方位角(度,-180~180)
 * 注意 atan2f 的参数顺序是 (x差, y差),因为这里 y 轴朝北当 0 度基准 */
float get_angle(GPS_local_Point_struct now,GPS_local_Point_struct aim){
    return -rad_to_degree(atan2f(aim.x-now.x, aim.y-now.y));  // 用 atan2 求角度,注意 y 在前
}

/* =========================== 两点欧氏距离 =========================== */
/* 移植自 GPS.c */
float get_distance(GPS_local_Point_struct now,GPS_local_Point_struct aim){
    return sqrtf((aim.x-now.x)*(aim.x-now.x)+(aim.y-now.y)*(aim.y-now.y));  // 欧氏距离公式
}

/* =========================== 角度差(处理 ±180 突变) =========================== */
/* 移植自 GPS.c
 * 求从 now 转到 aim 需要转过的角度,结果规整到 [-180,180]。
 * 这是防"179 到 -179 假差 358 度"的关键函数,凡角度相减都走这里。 */
float get_relative_angle(float now,float aim){
    float difference=aim-now;                       // 先直接相减
    if(difference<-180)return difference+360;       // 小于 -180,加 360
    if(difference>180)return difference-360;        // 大于 180,减 360
    return difference;                              // 返回规整后的差值
}

/* =========================== 由两点造一条直线 =========================== */
/* 移植自 GPS.c
 * 用标准式 ax+by+c=0 记录直线,以便计算斜率无穷大的竖直线 */
straight_line_struct draw_straight_line(GPS_local_Point_struct start,GPS_local_Point_struct end){
    straight_line_struct line={0};
    line.start=start;
    line.end=end;
    if(abs_float(start.x-end.x)<0.1 && abs_float(start.y-end.y)<0.1){  // 两点太近,直线无效
        line.same_point=1;
        return line;
    }
    else{
        line.a=end.y-start.y;
        line.b=start.x-end.x;
        line.c=(end.x-start.x)*start.y-(end.y-start.y)*start.x;
        return line;
    }
}

/* =========================== 点到直线的带符号垂距 =========================== */
/* 移植自 GPS.c
 * 返回:点到直线的垂直距离,带符号(从起点看向终点,右侧为正)
 * 符号靠 get_relative_angle 判方向,这样距离环 PID 知道往哪边打方向 */
float point_to_straight_line_distance(straight_line_struct line,GPS_local_Point_struct Point_2D){
    float dir_line=get_angle(line.start,line.end);          // 路线方向
    float dir_aim=get_angle(Point_2D,line.end);             // 目标方向
    int sign=get_relative_angle(dir_line,dir_aim)>0?1:-1;   // 从直线方向看,点在右侧为正

    if (line.a == 0 && line.b == 0) {
        // 非法直线,返回一个极大值提醒调用者出错
        return 10000.0f;
    }

    return abs_float((line.a*Point_2D.x+line.b*Point_2D.y+line.c)/sqrtf(line.a*line.a+line.b*line.b))*sign;
}

/* =========================== 点在直线哪一侧 =========================== */
/* 移植自 GPS.c:只取符号,不算距离 */
int get_point_to_line_dir(straight_line_struct line,GPS_local_Point_struct Point_2D){
    float dir_line=get_angle(line.start,line.end);          // 路线方向
    float dir_aim=get_angle(Point_2D,line.end);             // 目标方向
    int sign=get_relative_angle(dir_line,dir_aim)>0?1:-1;   // 从直线方向看,点在右侧为正

    return sign;
}
