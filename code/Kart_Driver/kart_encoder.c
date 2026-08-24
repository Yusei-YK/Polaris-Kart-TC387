#include "kart_encoder.h"
#include "IfxGpt12_IncrEnc.h"

static kart_encoder_state_t kart_encoder_state = {0};

/*
 * TIM2 正交模式的硬件计数是 4 倍边沿计数。逐飞 encoder_get_count()
 * 会在每次读取时直接做整数 /4；本工程每 5 ms 读取并清零一次，慢速时
 * 每周期不足 4 个边沿的余数会永久丢失，导致左轮计数随转速改变。
 * 这里直接读取 TIM2 原始计数，并把 /4 的余数保留到下一周期。
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

    /* left_raw 继续保持与 left_delta 相同的单倍脉冲量纲。 */
    kart_encoder_state.left_raw = (int16)(kart_encoder_state.left_delta * KART_LEFT_ENCODER_SIGN);
    kart_encoder_state.right_delta = (int16)(kart_encoder_state.right_raw * KART_RIGHT_ENCODER_SIGN);

    kart_encoder_state.left_sum += kart_encoder_state.left_delta;
    kart_encoder_state.right_sum += kart_encoder_state.right_delta;
}

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
