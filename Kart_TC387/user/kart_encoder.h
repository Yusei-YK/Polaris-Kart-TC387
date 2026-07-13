#ifndef KART_ENCODER_H_
#define KART_ENCODER_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

typedef struct
{
    int16 left_delta;
    int16 right_delta;
    int16 left_raw;
    int16 right_raw;
    int32 left_sum;
    int32 right_sum;
} kart_encoder_state_t;

void kart_encoder_init(void);
void kart_encoder_update(void);
int16 kart_encoder_get_left_delta(void);
int16 kart_encoder_get_right_delta(void);
int16 kart_encoder_get_left_raw(void);
int16 kart_encoder_get_right_raw(void);
int32 kart_encoder_get_left_sum(void);
int32 kart_encoder_get_right_sum(void);
const kart_encoder_state_t *kart_encoder_get_state(void);

#endif
