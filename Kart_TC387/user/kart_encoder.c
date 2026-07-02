#include "kart_encoder.h"

static kart_encoder_state_t kart_encoder_state = {0};

void kart_encoder_init(void)
{
    encoder_quad_init(KART_LEFT_ENCODER_INDEX, KART_LEFT_ENCODER_CH1, KART_LEFT_ENCODER_CH2);
    encoder_quad_init(KART_RIGHT_ENCODER_INDEX, KART_RIGHT_ENCODER_CH1, KART_RIGHT_ENCODER_CH2);
    encoder_clear_count(KART_LEFT_ENCODER_INDEX);
    encoder_clear_count(KART_RIGHT_ENCODER_INDEX);
}

void kart_encoder_update(void)
{
    kart_encoder_state.left_raw = encoder_get_count(KART_LEFT_ENCODER_INDEX);
    kart_encoder_state.right_raw = encoder_get_count(KART_RIGHT_ENCODER_INDEX);
    kart_encoder_state.left_delta = (int16)(kart_encoder_state.left_raw * KART_LEFT_ENCODER_SIGN);
    kart_encoder_state.right_delta = (int16)(kart_encoder_state.right_raw * KART_RIGHT_ENCODER_SIGN);

    encoder_clear_count(KART_LEFT_ENCODER_INDEX);
    encoder_clear_count(KART_RIGHT_ENCODER_INDEX);

    kart_encoder_state.left_sum += kart_encoder_state.left_delta;
    kart_encoder_state.right_sum += kart_encoder_state.right_delta;
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
