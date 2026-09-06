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
#define BOOT_READY_MS       (120u)

/* 一帧解码缓冲：BOOT_FRAME_W*BOOT_FRAME_H = 112*122 = 13664 个 uint16，
 * 也就是 27328 字节的 .bss，整个开机期间常驻。
 * 【2026-09-06 订正】原注释写的 16KB 不对（16KB 只够 8192 个像素），
 * 按现在的宏算就是上面这个 27328。改帧尺寸时这块 RAM 按面积涨，先看 .bss 余量。
 * “压缩素材常驻 Flash，不占数据 RAM”这半句是对的：素材数组都是 const。 */
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
    /* x=40 不是随手填的:8x16 字体、屏宽 240,20 个字符 = 160px,
     * (240-160)/2 = 40 正好居中。改这行文字就必须重算 x,
     * 否则整行偏出中线(2026-09-06 就是这么错位的)。 */
    ips200_show_string(40u, 34u, "TC387 CONTROL SYSTEM");
    ips200_draw_line(BOOT_PROGRESS_X, BOOT_PROGRESS_Y,
                     BOOT_PROGRESS_X + BOOT_PROGRESS_W - 1u,
                     BOOT_PROGRESS_Y, BOOT_LIGHT_BLUE);


    /* 【2026-09-06 订正】这两段原注释有三处与代码不符，一并改掉：
     *   1) “SPI 写 192*210 个像素”——写上去的是显示尺寸
     *      BOOT_IMAGE_W*BOOT_IMAGE_H = 160*174 = 27840 个像素。
     *      192x210 是更早一版的放大尺寸，改小之后没回来改这句。
     *   2) “15 帧均匀抽样，仍只刷原素材一半”——没有抽样。下面的循环是
     *      display_frame < BOOT_FRAME_COUNT，30 帧全放，进度条同一个分母。
     *      英文那行 sampled uniformly 说的是同一件不存在的事，一起删。
     *   3) “使用 (COUNT-1)/(DISPLAY-1) 映射”——本文件里没有这个映射，
     *      循环变量直接就是素材帧号，中间没有换算。
     * 仍然成立的是“刷屏本身远慢于显式延时”：每帧 27840 个像素的 SPI 写就是
     * 天然的帧间隔，所以循环里没有 delay（见循环末尾那条）。
     * 既然 30 帧全放，第一帧和最后一帧当然都显示，不会在动作没演完时跳进 READY。 */
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
