#include "kart_encoder.h"
#include "IfxGpt12_IncrEnc.h"

static kart_encoder_state_t kart_encoder_state = {0};

/*
 * TIM2 正交模式的硬件计数是 4 倍边沿计数。逐飞 encoder_get_count()
 * 会在每次读取时直接做整数 /4；本工程每 5 ms 读取并清零一次，慢速时
 * 每周期不足 4 个边沿的余数会永久丢失，导致左轮计数随转速改变。
 * 这里直接读取 TIM2 原始计数，并把 /4 的余数保留到下一周期。
 *
 * 【右轮没做同样处理】右轮走的还是 encoder_get_count()(见 update() 里那一行),
 * 也就是说右轮每 5ms 一样会丢 /4 的余数,本文件没给它留 remainder。
 * 为什么只修左轮:当初查的是左轮计数随转速改变这个具体现象,右轮当时没暴露
 * 问题,而左右 PPR 已实测一致(左十圈 20481、右 20489,kart_calib.h 第三节),
 * 所以没有一并动。
 * 要确认右轮是否同病:低速匀速走一段直线,比两侧累计和 —— 当前 43 通道剖面里
 * CH35/CH36 是两侧 delta,差值随速度系统性变化就是同一个病,那时再给右轮
 * 补一份 remainder(照抄左轮那三行即可)。
 */
static int32 kart_left_quad_remainder = 0;

void kart_encoder_init(void)
{
    encoder_quad_init(KART_LEFT_ENCODER_INDEX, KART_LEFT_ENCODER_CH1, KART_LEFT_ENCODER_CH2);
    encoder_quad_init(KART_RIGHT_ENCODER_INDEX, KART_RIGHT_ENCODER_CH1, KART_RIGHT_ENCODER_CH2);
    encoder_clear_count(KART_LEFT_ENCODER_INDEX);
    encoder_clear_count(KART_RIGHT_ENCODER_INDEX);
}

void kart_encoder_update(void)
{
    int16 left_hw_raw = (int16)IfxGpt12_T2_getTimerValue(&MODULE_GPT120);
    int32 left_scaled;

    kart_encoder_state.right_raw = encoder_get_count(KART_RIGHT_ENCODER_INDEX);

    encoder_clear_count(KART_LEFT_ENCODER_INDEX);
    encoder_clear_count(KART_RIGHT_ENCODER_INDEX);

    left_scaled = (int32)left_hw_raw * KART_LEFT_ENCODER_SIGN + kart_left_quad_remainder;
    kart_encoder_state.left_delta = (int16)(left_scaled / 4);
    kart_left_quad_remainder = left_scaled - (int32)kart_encoder_state.left_delta * 4;

    /* left_raw 继续保持与 left_delta 相同的单倍脉冲量纲。
     * 这里再乘一次 KART_LEFT_ENCODER_SIGN 不是笔误:上面算 left_delta 时已经套过
     * 一次极性,再乘一次相当于把符号除回去,好让 left_raw 与下一行的 right_raw
     * 语义一致 —— 两个 raw 都是"未套极性的本拍增量"。
     * SIGN = +1 时两者数值相同,看不出区别;哪天极性改成 -1 才分得开。
     * 别顺手把这一乘当冗余删掉。 */
    kart_encoder_state.left_raw = (int16)(kart_encoder_state.left_delta * KART_LEFT_ENCODER_SIGN);
    kart_encoder_state.right_delta = (int16)(kart_encoder_state.right_raw * KART_RIGHT_ENCODER_SIGN);

    kart_encoder_state.left_sum += kart_encoder_state.left_delta;
    kart_encoder_state.right_sum += kart_encoder_state.right_delta;
}

/* 清零累计并丢掉余数。关全局中断的理由:清零这几个量与 update() 的
 * 读-清-累加必须互斥,否则会留下半拍的脏值 —— update() 在 5ms 环里,
 * 而本函数不在环里(当前没有调用点,见头文件),随时可能被环打断。
 * update() 自己不关中断,因为它本身就是环的一部分,不会被自己打断。 */
void kart_encoder_reset(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    encoder_clear_count(KART_LEFT_ENCODER_INDEX);
    encoder_clear_count(KART_RIGHT_ENCODER_INDEX);

    kart_encoder_state.left_delta = 0;
    kart_encoder_state.right_delta = 0;
    kart_encoder_state.left_raw = 0;
    kart_encoder_state.right_raw = 0;
    kart_encoder_state.left_sum = 0;
    kart_encoder_state.right_sum = 0;
    kart_left_quad_remainder = 0;

    interrupt_global_enable(interrupt_state);
}

int16 kart_encoder_get_left_delta(void)
{
    return kart_encoder_state.left_delta;
}

int16 kart_encoder_get_right_delta(void)
{
    return kart_encoder_state.right_delta;
}

int16 kart_encoder_get_left_raw(void)
{
    return kart_encoder_state.left_raw;
}

int16 kart_encoder_get_right_raw(void)
{
    return kart_encoder_state.right_raw;
}

int32 kart_encoder_get_left_sum(void)
{
    return kart_encoder_state.left_sum;
}

int32 kart_encoder_get_right_sum(void)
{
    return kart_encoder_state.right_sum;
}

const kart_encoder_state_t *kart_encoder_get_state(void)
{
    return &kart_encoder_state;
}
