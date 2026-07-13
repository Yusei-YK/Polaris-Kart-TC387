#ifndef KART_MISSION_UI_H_
#define KART_MISSION_UI_H_

#include "zf_common_headfile.h"

/*
 * IPS200 比赛任务菜单/状态页。
 *
 * 本模块不直接读取旋钮或按键 GPIO。后续完成消抖与旋钮方向判断后，只需要映射：
 *   旋钮逆/顺时针 -> PREVIOUS / NEXT
 *   旋钮短按       -> ENTER
 *   独立 START 键  -> START
 *   长按/急停      -> STOP
 * 即可保持界面逻辑与硬件驱动解耦。
 */

#define KART_MISSION_UI_REFRESH_MS  (100U)

typedef enum
{
    KART_MISSION_UI_PREVIOUS = 0,
    KART_MISSION_UI_NEXT,
    KART_MISSION_UI_ENTER,
    KART_MISSION_UI_BACK,
    KART_MISSION_UI_START,
    KART_MISSION_UI_STOP,
    KART_MISSION_UI_PAGE
} kart_mission_ui_event_t;

void kart_mission_ui_init  (void);
void kart_mission_ui_poll  (uint16 elapsed_ms);
void kart_mission_ui_event (kart_mission_ui_event_t event);

#endif
