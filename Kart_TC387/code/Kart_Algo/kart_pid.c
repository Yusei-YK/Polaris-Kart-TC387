#include "kart_pid.h"

/*
 * PID 控制器实现
 * 说明:这是位置式 PID(输出 = P + I + D 三项之和)
 * 移植自 TopSpeed 越野车久经比赛验证的 PID,仅去掉原工程的头文件依赖
 */

/* -------------------------------------------------------------------------
 * 初始化:开机时调用一次,把参数填进记忆本,并把所有状态清零
 * ------------------------------------------------------------------------- */
void kart_pid_init(kart_pid_t *kart_pid, float Kp, float Ki, float Kd, float i_max, float out_max)
{
    kart_pid->first_cal = 1;         // 标记为"第一次",第一次不算 D

    kart_pid->Kp = Kp;               // 填入三个脾气参数
    kart_pid->Ki = Ki;
    kart_pid->Kd = Kd;

    kart_pid->err      = 0.0f;       // 当前误差清零
    kart_pid->err_last = 0.0f;       // 上次误差清零
    kart_pid->err_sum  = 0.0f;       // 累计误差清零

    kart_pid->i_max   = i_max;       // 填入两个限幅值
    kart_pid->out_max = out_max;

    kart_pid->output = 0.0f;         // 输出清零
}

/* -------------------------------------------------------------------------
 * 更新:每个控制周期(5ms)调用一次
 * 输入:当前误差 err
 * 输出:结果写进 kart_pid->output
 * ------------------------------------------------------------------------- */
void kart_pid_update(kart_pid_t *kart_pid, float err)
{
    kart_pid->err = err;             // 记下这一次的误差

    /* --- 积分项:把每次的误差累加起来 --- */
    kart_pid->err_sum += kart_pid->err;   // 累加(err_sum = err_sum + err)

    /* 积分限幅:防止 err_sum 越攒越大失控
     * i_max 是"积分项"的上限,除以 Ki 换算成"累加值"的上限 */
    if(kart_pid->Ki != 0.0f)         // 防止除以 0(Ki 为 0 时不需要限幅)
    {
        float sum_limit = kart_pid->i_max / kart_pid->Ki;
        if(kart_pid->err_sum > sum_limit)
        {
            kart_pid->err_sum = sum_limit;
        }
        else if(kart_pid->err_sum < -sum_limit)
        {
            kart_pid->err_sum = -sum_limit;
        }
    }

    /* --- 分别算出 P、I、D 三项 --- */
    float p = kart_pid->err * kart_pid->Kp;                      // P = 当前误差 × Kp
    float i = kart_pid->err_sum * kart_pid->Ki;                  // I = 累计误差 × Ki
    float d = (kart_pid->err - kart_pid->err_last) * kart_pid->Kd;    // D = (这次-上次) × Kd

    kart_pid->err_last = kart_pid->err;   // 存下当前误差,供下次算 D 用

    /* --- 三项相加得到输出 --- */
    if(kart_pid->first_cal == 1)     // 第一次没有"上次误差",D 无意义,只用 P+I
    {
        kart_pid->output = p + i;
        kart_pid->first_cal = 0;     // 清掉旗子,以后就是正常的 P+I+D
    }
    else
    {
        kart_pid->output = p + i + d;
    }

    /* --- 输出限幅:把结果压在 [-out_max, +out_max] 之间 --- */
    if(kart_pid->output > kart_pid->out_max)
    {
        kart_pid->output = kart_pid->out_max;
        kart_pid->err_sum -= kart_pid->err;   // 已经饱和了,把刚才这次累加撤回(抗积分饱和)
    }
    else if(kart_pid->output < -kart_pid->out_max)
    {
        kart_pid->output = -kart_pid->out_max;
        kart_pid->err_sum -= kart_pid->err;   // 同上
    }
}

/* -------------------------------------------------------------------------
 * 清零:停车时调用,抹掉累计记忆,避免下次启动带着旧账
 * ------------------------------------------------------------------------- */
void kart_pid_reset(kart_pid_t *kart_pid)
{
    kart_pid->first_cal = 1;
    kart_pid->err      = 0.0f;
    kart_pid->err_last = 0.0f;
    kart_pid->err_sum  = 0.0f;
    kart_pid->output   = 0.0f;
}
