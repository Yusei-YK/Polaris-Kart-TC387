#include "kart_boot_anim.h"

#if BOOT_ANIM_ENABLE
#include "zf_device_ips200.h"
#include "kart_boot_anim_data.h"

#define BOOT_BG             (0xFFFFu)   /* 白 */
#define BOOT_BLUE           (0x03D9u)   /* #007ACC */
#define BOOT_NAVY           (0x194Du)   /* 深蓝正文 */
#define BOOT_LIGHT_BLUE     (0xCE7Fu)   /* 浅蓝进度槽 */
#define BOOT_IMAGE_X        (40u)
#define BOOT_IMAGE_Y        (74u)
#define BOOT_IMAGE_W        (160u)
#define BOOT_IMAGE_H        (174u)
#define BOOT_PROGRESS_X     (24u)
#define BOOT_PROGRESS_Y     (284u)
#define BOOT_PROGRESS_W     (192u)
#define BOOT_DISPLAY_FRAMES (15u)     /* 30 帧素材隔帧显示，兼顾流畅度与启动速度 */
#define BOOT_READY_MS       (120u)

/* 一帧解码缓冲 16KB；压缩素材常驻 Flash，不占数据 RAM。 */
static uint16 boot_frame[BOOT_FRAME_W * BOOT_FRAME_H];

static void boot_decode_frame(uint8 frame)
{
    uint32 src = kart_boot_frame_offsets[frame];
    uint32 end = kart_boot_frame_offsets[frame + 1u];
    uint32 dst = 0;

    while(src + 1u < end && dst < BOOT_FRAME_W * BOOT_FRAME_H)
    {
        uint8 count = kart_boot_rle[src++];
        uint16 color = kart_boot_palette[kart_boot_rle[src++]];

        while(count-- > 0u && dst < BOOT_FRAME_W * BOOT_FRAME_H)
        {
            boot_frame[dst++] = color;
        }
    }

    /* 损坏/截断素材时以白色补足，绝不把未初始化 RAM 送上屏。 */
    while(dst < BOOT_FRAME_W * BOOT_FRAME_H)
    {
        boot_frame[dst++] = BOOT_BG;
    }
}

void kart_boot_anim_play(void)
{
    uint8 display_frame;
    uint16 progress_x = BOOT_PROGRESS_X;

    ips200_set_color(BOOT_NAVY, BOOT_BG);
    ips200_full(BOOT_BG);
    ips200_show_string(84u, 14u, "SMART CAR");
    ips200_set_color(BOOT_BLUE, BOOT_BG);
    ips200_show_string(40u, 34u, "Kart_TC387 CONTROL SYSTEM");
    ips200_draw_line(BOOT_PROGRESS_X, BOOT_PROGRESS_Y,
                     BOOT_PROGRESS_X + BOOT_PROGRESS_W - 1u,
                     BOOT_PROGRESS_Y, BOOT_LIGHT_BLUE);


    /* 放大后每帧要经 SPI 写 192*210 个像素，刷屏本身远慢于原来的显式延时。
     * 15 帧均匀抽样比之前的 10 帧更顺滑，仍只刷原素材一半；使用
     * (COUNT-1)/(DISPLAY-1) 映射，确保
     * 第一帧和最后一帧都显示，不会在动作尚未结束时跳入 READY。 */
    /* Play all 30 frames sampled uniformly across the source video. */
    for(display_frame = 0u; display_frame < BOOT_FRAME_COUNT; display_frame++)
    {
        uint16 next_x;

        boot_decode_frame(display_frame);
        ips200_show_rgb565_image(BOOT_IMAGE_X, BOOT_IMAGE_Y, boot_frame,
                                 BOOT_FRAME_W, BOOT_FRAME_H,
                                 BOOT_IMAGE_W, BOOT_IMAGE_H, 0u);

        next_x = (uint16)(BOOT_PROGRESS_X
                 + ((uint32)(display_frame + 1u) * (BOOT_PROGRESS_W - 1u))
                   / BOOT_FRAME_COUNT);
        if(next_x > progress_x)
        {
            ips200_draw_line(progress_x, BOOT_PROGRESS_Y, next_x,
                             BOOT_PROGRESS_Y, BOOT_BLUE);
            progress_x = next_x;
        }
        /* 不再额外 delay：整帧 SPI 刷屏已经形成可见的帧间隔。 */
    }

    ips200_set_color(BOOT_BLUE, BOOT_BG);
    ips200_show_string(100u, 300u, "READY");
    system_delay_ms(BOOT_READY_MS);
}

#else

void kart_boot_anim_play(void) { }

#endif
