#ifndef KART_STEER_CTRL_H_
#define KART_STEER_CTRL_H_

#include "zf_common_headfile.h"
#include "kart_pid.h"

/*
 * 卡丁车转向控制(串级:航向外环 → 转角内环)
 * ------------------------------------------------------------------
 * 硬件事实:转向是【有刷电机 + SPI 绝对编码器】,不是自动回中的 RC 舵机。
 *   → 有刷电机断电不回中,必须靠"转角位置环"把它按在目标角度上。
 *   → 这是刚需,不是可选项。
 *
 * 架构(两级串级):
 *   外环(航向环 heading):  目标航向 − IMU 实测 yaw(wrap±180) → PID → 目标转角
 *   内环(转角环 angle):    目标转角 − 绝对编码器 center_delta → PID → 转向电机 duty
 *
 * 内环是地基:先只跑内环(手给目标转角,验证能不能稳定按住方向盘),
 *   稳了再套外环。外环关掉时,内环目标转角由串口 a<val> 直接给,方便单测。
 *
 * 调用节拍:必须放【主循环】,紧跟 kart_steer_abs_update() 之后 ——
 *   因为 center_delta 靠 SPI 读、只在主循环刷新,放 5ms 中断会吃到上电那帧的死数据。
 * ------------------------------------------------------------------
 * 量纲:
 *   转角误差单位 = 绝对编码器 raw 计数(4096/圈,约 0.088°/计数)
 *   center_delta 范围见 board_pins.h:右 −1182 ~ 左 +1066(实测限位换算 CENTER=1961)
 *   航向误差单位 = 度(IMU yaw ∈ −180~180)
 * ------------------------------------------------------------------
 */

/* 转角内环:目标转角软限幅(留 50 计数余量,别顶到机械硬限位磨电机)。
 * 2026-07-19 齿轮重装后实测:右 477、中 1575、左 2728。 */
#define KART_STEER_DELTA_LIMIT_L        (+1103)     /* 左软限(2728-50-1575) */
#define KART_STEER_DELTA_LIMIT_R        (-1048)     /* 右软限(477+50-1575) */

/* 转向电机符号:若上车发现"越纠越歪"(正反馈发散),把这个从 +1 改 −1。
 * 悬空验证:手给 a300(目标偏左),电机应把方向盘往 center_delta 增大方向推。 */
#define KART_STEER_LOOP_SIGN            (-1)         /* 开环实测定论(2026-07-19):so1000→ch3减小(向右),so-1000→ch3增大(向左)。正duty=右=ch3减,故负反馈需LOOP_SIGN=-1。此前'打死'是大初始误差超调,非正反馈。 */

/* 内环 PID 默认(悬空验证已冻结)。
 * out_max 先给 4000(不放满 10000),限幅防打飞,上车可再放。
 * 冻结依据:悬空 sa0→I6=3、sa200→I6=203,静差 ~3 计数(≈0.26°),si0.5 顶过静摩擦。
 * Kd=0:阶跃尖峰来自 Kp 比例冲击,加 D 只会更大(D 对阶跃正向踢),故保持 0。 */
#define KART_STEER_KP_DEFAULT           (15.0f)
#define KART_STEER_KI_DEFAULT           (0.5f)
#define KART_STEER_KD_DEFAULT           (0.0f)
#define KART_STEER_IMAX_DEFAULT         (2000.0f)
#define KART_STEER_OUTMAX_DEFAULT       (4000.0f)   /* 2026-07-19:收敛验证通过,恢复 40% 输出 */

/* 航向外环 PID 默认(先占位,内环验证通过后再调 hp/hi/hd)。
 * 输出是"目标转角计数",限幅到转角软限位量级。 */
#define KART_HEAD_KP_DEFAULT            (30.0f)
#define KART_HEAD_KI_DEFAULT            (0.0f)
#define KART_HEAD_KD_DEFAULT            (0.0f)
#define KART_HEAD_IMAX_DEFAULT          (500.0f)
#define KART_HEAD_OUTMAX_DEFAULT        (1000.0f)

typedef struct
{
    kart_pid_t  angle_pid;          // 内环:转角位置 PID
    kart_pid_t  head_pid;           // 外环:航向 PID

    uint8       angle_enable;       // 内环使能(=1 才驱动转向电机)
    uint8       head_enable;        // 外环使能(=1 时目标转角由航向环给;=0 时由 target_delta 直给)

    float       target_yaw;         // 外环目标航向(度)
    float       meas_yaw;           // 外环实测航向(度)
    float       target_delta;       // 内环目标转角(编码器计数);外环开时被航向环覆写
    float       meas_delta;         // 内环实测转角(center_delta)
    int16       output_duty;        // 转向电机输出 duty(送 power_set_steer_duty)
} kart_steer_ctrl_t;

extern kart_steer_ctrl_t kart_steer;

/* --- 对外接口 --- */
void  kart_steer_ctrl_init(void);           // 初始化两级 PID,默认全不使能
void  kart_steer_ctrl_update(void);         // 串级一拍:放主循环 steer_abs_update 之后

void  kart_steer_set_angle_enable(uint8 en);    // 内环使能
void  kart_steer_set_head_enable(uint8 en);     // 外环使能(会连带开内环)
void  kart_steer_set_target_delta(float delta); // 直给内环目标转角(外环关时用,单测内环)
void  kart_steer_set_target_yaw(float yaw);     // 设外环目标航向

void  kart_steer_set_angle_pid(float kp, float ki, float kd);   // 在线调内环
void  kart_steer_set_head_pid(float kp, float ki, float kd);    // 在线调外环

/* --- 给 VOFA/调试读的取值接口 --- */
float kart_steer_get_target_delta(void);
float kart_steer_get_meas_delta(void);
int16 kart_steer_get_output(void);
float kart_steer_get_target_yaw(void);

#endif
