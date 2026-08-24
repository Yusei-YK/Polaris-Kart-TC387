#ifndef KART_BOOT_ANIM_H_
#define KART_BOOT_ANIM_H_
#include "zf_common_headfile.h"

/* 置 0 可完全裁掉开机动画与 47KB 压缩素材。 */
#define BOOT_ANIM_ENABLE    (1)

/* IPS200 初始化完成后调用一次；放大画面保持不变，30 帧素材抽取 15 帧播放。 */
void kart_boot_anim_play(void);

#endif
