#ifndef KART_BOOT_ANIM_H_
#define KART_BOOT_ANIM_H_
#include "zf_common_headfile.h"

/* 置 0 可完全裁掉开机动画与压缩素材：素材全在 kart_boot_anim_data.h 里，
 * 而那个 #include 就写在 .c 的 #if BOOT_ANIM_ENABLE 里面，所以置 0 连编都不编，
 * 不是编了再丢。
 * 体积：kart_boot_rle[] 72292 字节 + offsets 31*4 + palette 6*2 = 72428 字节
 * ≈ 70.7KB Flash。它们都是 const，常驻 Flash 不占数据 RAM；占 RAM 的是 .c 里
 * 那个解码缓冲 boot_frame，另算。 */
#define BOOT_ANIM_ENABLE    (1)

/* IPS200 初始化完成后调用一次。全工程只有一个调用点：
 * kart_menu.c 的开机路径（搜 kart_boot_anim_play();）。
 * 30 帧一帧不落全放完，进度条按 30 做分母，没有抽帧。素材本身是 112x122，
 * 上屏时由 ips200_show_rgb565_image 放大到 160x174 才画，不是原尺寸贴图。 */
void kart_boot_anim_play(void);

#endif
