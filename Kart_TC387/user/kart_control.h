#ifndef KART_CONTROL_H_
#define KART_CONTROL_H_

#include "zf_common_headfile.h"
#include "kart_pid.h"

/*
 * 卡丁车控制环(速度环先行)
 * ------------------------------------------------------------------
 * 移植自 TopSpeed 速度环(Subject_1 里的 vel_ctrler + Speed_encoder 滤波)。
 * 对照原版的改动(都是为卡丁车硬件适配,不是瞎改):
 *   1. 测速:TopSpeed 靠无刷电机板 UART 回传脉冲;卡丁车编码器直连 TC387,
 *      本地读 delta,固定 5ms 采样(原版是收到帧才算,异步)。
 *   2. duty:TopSpeed 用 += 累加(它同节拍多个环叠加);卡丁车速度环单独跑,
 *      改成直接赋值 duty = output,更直观好调(已跟用户确认)。
 *   3. 下发:TopSpeed 走 send_motor_duty 无刷串口;卡丁车是有刷 DRV8701,
 *      换成 power_set_rear_duty() 出 PWM。
 * 保留照搬的:10 点滑动平均滤波、位置式 PID、目标速度斜坡(后面接规划时用)。
 * ------------------------------------------------------------------
 * 调用节拍:kart_control_speed_update() 必须放 5ms 定时中断里,和 IMU 同拍。
 * 单位说明:悬空调参阶段速度单位用"编码器脉冲/5ms",不折算 m/s;
 *          等地面实测标定出 KART_ENC_PULSE_TO_M 后再换算成真实车速。
 * ------------------------------------------------------------------
 */

/* 滑动平均滤波窗口长度(照搬 TopSpeed WHEEL_SPD_LPF_TEMP_LEN) */
#define KART_SPEED_LPF_LEN              (10)

/* 每编码器脉冲对应的米数 —— 占位,待地面实测标定(量一段实际距离÷脉冲数)。
 * 悬空调参用不到它,先留着,接航位推算/真实车速时再填真值。
 * 标定备忘:后轮齿比 40:20 = 2:1(电机端转 2 圈,轮子转 1 圈)。
 *          编码器若装在电机端,轮子每转 = 编码器 2×线数×倍频 个脉冲;
 *          装在轮端则直接对应。标定时按实际安装位置换算。 */
#define KART_ENC_PULSE_TO_M            (0.001f)
#define KART_REAR_GEAR_RATIO           (2.0f)      // 后轮减速比 40:20,标定真实车速时用

/* -------------------------------------------------------------------------
 * 速度环控制块:把速度环所有状态打包在一起
 * ------------------------------------------------------------------------- */
typedef struct
{
    kart_pid_t  pid;                            // 速度环 PID(复用 kart_pid)
    uint8       enable;                         // =1 才输出,=0 输出 0(悬空/急停用)

    float       target;                         // 目标速度(脉冲/5ms),串口可在线改
    float       meas_raw;                       // 本次实测速度(滤波前,脉冲/5ms)
    float       meas;                           // 滤波后实测速度(脉冲/5ms),PID 反馈用这个

    float       lpf_buf[KART_SPEED_LPF_LEN];    // 滑动平均缓冲区
    int         lpf_idx;                        // 缓冲区写指针(环形)

    int16       output_duty;                    // 速度环算出的 duty(下发给两后轮)
} kart_speed_ctrl_t;

/* --- 对外接口 --- */
void  kart_control_init(void);                  // 初始化速度环(填默认 PID 参数)
void  kart_control_speed_update(void);          // 速度环一拍:读编码器→滤波→PID→下发。放 5ms 中断
void  kart_control_set_enable(uint8 en);        // 使能/关闭速度环输出
void  kart_control_set_target(float target);    // 设目标速度(脉冲/5ms)
void  kart_control_set_pid(float kp, float ki, float kd);   // 在线改 PID 参数
void  kart_control_get_pid(float *kp, float *ki, float *kd); // 原子读取当前 PID 参数

/* --- 给调试/VOFA 读的取值接口 --- */
float kart_control_get_target(void);            // 目标速度
float kart_control_get_meas(void);              // 滤波后实测速度
int16 kart_control_get_output(void);            // 当前输出 duty
uint8 kart_control_is_enabled(void);            // 速度环是否使能(主循环用它仲裁:谁来管后轮 duty)

#endif
