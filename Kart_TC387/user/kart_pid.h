#ifndef KART_PID_H_
#define KART_PID_H_

#include "zf_common_headfile.h"

/* PID 控制器的"记忆本"：每一路 PID 都要有自己的一份，用来记住上一次的状态 */
typedef struct
{
    uint8  first_cal;   // 是不是第一次计算(第一次没有"上一次误差",不能算 D)
    float  Kp;          // 比例系数:偏差放大多少倍
    float  Ki;          // 积分系数:累计偏差放大多少倍
    float  Kd;          // 微分系数:偏差变化率放大多少倍

    float  err;         // 当前这一次的误差
    float  err_last;    // 上一次的误差(用来算 D)
    float  err_sum;     // 历史误差的累加(用来算 I)

    float  i_max;       // 积分项最大限幅(防止 I 攒太多失控)
    float  out_max;     // 最终输出的最大限幅(防止指令超出电机能力)

    float  output;      // 算出来的最终结果:该给多少劲
} kart_pid_t;

/* 初始化一个 PID 控制器:填入三个系数和两个限幅值 */
void kart_pid_init(kart_pid_t *pid, float Kp, float Ki, float Kd, float i_max, float out_max);

/* 喂给它一个"当前误差",它算出"该给多少劲",结果存在 pid->output 里 */
void kart_pid_update(kart_pid_t *pid, float err);

/* 清零:把这个 PID 的记忆全部抹掉,重新开始(比如车停下来时用) */
void kart_pid_reset(kart_pid_t *pid);

#endif
