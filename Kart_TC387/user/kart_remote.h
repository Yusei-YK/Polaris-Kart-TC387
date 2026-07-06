#ifndef KART_REMOTE_H_
#define KART_REMOTE_H_

#include "zf_common_headfile.h"

#define KART_REMOTE_CHANNEL_NUM         (6)
#define KART_REMOTE_FRAME_LEN           (25)
#define KART_REMOTE_FRAME_HEAD          (0x0F)
#define KART_REMOTE_FRAME_TAIL          (0x00)
#define KART_REMOTE_FAILSAFE_FLAG       (0x04)

#define KART_REMOTE_UART_BAUD           (100000)
#define KART_REMOTE_TIMEOUT_TICKS       (100)

#define KART_REMOTE_CH_MIN              (200)
#define KART_REMOTE_CH_MID              (1000)
#define KART_REMOTE_CH_MAX              (1800)
#define KART_REMOTE_CH_DEAD_ZONE        (50)
#define KART_REMOTE_ENABLE_CH_LOW       (500)
#define KART_REMOTE_ENABLE_CH_HIGH      (1300)

typedef struct
{
    uint16 channel[KART_REMOTE_CHANNEL_NUM];
    int16  steering;
    int16  throttle;
    uint8  switch_stage;
    uint8  online;
    uint8  frame_ready;
} kart_remote_t;

extern volatile kart_remote_t kart_remote;

void kart_remote_init(void);
void kart_remote_uart_callback(void);
void kart_remote_poll(void);

#endif
