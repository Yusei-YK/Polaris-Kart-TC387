#include "kart_pid.h"

/*
 * PID 控制器实现
 * 说明:这是位置式 PID(输出 = P + I + D 三项之和)
 * 移植自 TopSpeed 越野车久经比赛验证的 PID,仅去掉原工程的头文件依赖
 */

/* -------------------------------------------------------------------------
 * 初始化:开机时调用一次,把参数填进记忆本,并把所有状态清零
 * ------------------------------------------------------------------------- */
void kart_pid_init(kart_pid_t *pid, float Kp, float Ki, float Kd, float i_max, float out_max)
{
    pid->first_cal = 1;         // 标记为"第一次",第一次不算 D

    pid->Kp = Kp;               // 填入三个脾气参数
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->err      = 0.0f;       // 当前误差清零
    pid->err_last = 0.0f;       // 上次误差清零
    pid->err_sum  = 0.0f;       // 累计误差清零

    pid->i_max   = i_max;       // 填入两个限幅值
    pid->out_max = out_max;

    pid->output = 0.0f;         // 输出清零
}

/* -------------------------------------------------------------------------
 * 更新:每个控制周期(5ms)调用一次
 * 输入:当前误差 err
 * 输出:结果写进 pid->output
 * ------------------------------------------------------------------------- */
void kart_pid_update(kart_pid_t *pid, float err)
{
    pid->err = err;             // 记下这一次的误差

    /* --- 积分项:把每次的误差累加起来 --- */
    pid->err_sum += pid->err;   // 累加(err_sum = err_sum + err)

    /* 积分限幅:防止 err_sum 越攒越大失控
     * i_max 是"积分项"的上限,除以 Ki 换算成"累加值"的上限 */
    if(pid->Ki != 0.0f)         // 防止除以 0(Ki 为 0 时不需要限幅)
    {
        float sum_limit = pid->i_max / pid->Ki;
        if(pid->err_sum > sum_limit)
        {
            pid->err_sum = sum_limit;
        }
        else if(pid->err_sum < -sum_limit)
        {
            pid->err_sum = -sum_limit;
        }
    }

    /* --- 分别算出 P、I、D 三项 --- */
    float p = pid->err * pid->Kp;                      // P = 当前误差 × Kp
    float i = pid->err_sum * pid->Ki;                  // I = 累计误差 × Ki
    float d = (pid->err - pid->err_last) * pid->Kd;    // D = (这次-上次) × Kd

    pid->err_last = pid->err;   // 存下当前误差,供下次算 D 用

    /* --- 三项相加得到输出 --- */
    if(pid->first_cal == 1)     // 第一次没有"上次误差",D 无意义,只用 P+I
    {
        pid->output = p + i;
        pid->first_cal = 0;     // 清掉旗子,以后就是正常的 P+I+D
    }
    else
    {
        pid->output = p + i + d;
    }

    /* --- 输出限幅:把结果压在 [-out_max, +out_max] 之间 --- */
    if(pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
        pid->err_sum -= pid->err;   // 已经饱和了,把刚才这次累加撤回(抗积分饱和)
    }
    else if(pid->output < -pid->out_max)
    {
        pid->output = -pid->out_max;
        pid->err_sum -= pid->err;   // 同上
    }
}

/* -------------------------------------------------------------------------
 * 清零:停车时调用,抹掉累计记忆,避免下次启动带着旧账
 * ------------------------------------------------------------------------- */
void kart_pid_reset(kart_pid_t *pid)
{
    pid->first_cal = 1;
    pid->err      = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum  = 0.0f;
    pid->output   = 0.0f;
}
