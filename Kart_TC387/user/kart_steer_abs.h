#ifndef KART_STEER_ABS_H_
#define KART_STEER_ABS_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

void kart_steer_abs_init(void);
void kart_steer_abs_update(void);
uint16 kart_steer_abs_get_frame(void);
uint16 kart_steer_abs_get_raw(void);
int16 kart_steer_abs_get_center_delta(void);
int16 kart_steer_abs_get_deg_x100(void);

#endif
