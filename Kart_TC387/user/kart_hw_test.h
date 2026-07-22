#ifndef KART_HW_TEST_H_
#define KART_HW_TEST_H_

#include "zf_common_headfile.h"

/*
 * 硬件自测(临时) —— 并口 IPS200 屏 + 旋钮编码器 + 5 向按键
 * ------------------------------------------------------------------
 * 目的:验证 v2 主板"并口 GPIO 屏"能否点亮/不花屏,旋钮和按键能否读到。
 *      这是上车跑科目前的硬件确认工具,验证完把 KART_HW_TEST 关回 0 即可,
 *      对正式主循环零副作用。
 *
 * 用法(二选一,只在自测时开):
 *   在 cpu0_main.c 顶部 #define KART_HW_TEST 1,烧录。
 *   现象:开机先全屏刷 红→绿→蓝(各 500ms)验证刷屏无花点,
 *        然后常驻显示旋钮计数 + 6 个按键实时电平。
 *   转旋钮看 ENC 数字增减;按键按下看对应位 0/1 翻转。
 *
 * 引脚(取自 v2 网表 H1 排针,已核对无冲突):
 *   旋钮 A/B/SW = P11.2 / P11.3 / P20.6
 *   UP/DOWN/LEFT/RIGHT/MID = P33.11 / P20.0 / P21.6 / P21.7 / P33.4
 *   屏并口数据 D0~D7 = P11.9~P11.12 / P13.0~P13.3,控制线 P15.0~P15.5
 * ------------------------------------------------------------------
 */

/* 旋钮引脚 */
#define KART_KNOB_A_PIN         (P11_2)
#define KART_KNOB_B_PIN         (P11_3)
#define KART_KNOB_SW_PIN        (P20_0)         /* 实测旋钮按键是 P20.0 */

/* 5 向按键引脚（硬件 UP/LEFT(P21.6) 短路，用 SW3(P20.7) 代替 LEFT）*/
#define KART_KEY_UP_PIN         (P33_11)        /* UP (与P21.6短路) */
#define KART_KEY_DOWN_PIN       (P20_6)         /* 实测 DOWN 是 P20.6 */
#define KART_KEY_LEFT_PIN       (P33_4)         /* SW2.6 (实测是LEFT) */
#define KART_KEY_RIGHT_PIN      (P21_7)
#define KART_KEY_MID_PIN        (P20_7)         /* SW3 独立按键 (实测是MID) */

/* 自测主入口:内部死循环,不返回。由 cpu0_main 在 KART_HW_TEST 开时调用。 */
void kart_hw_test_run(void);

#endif
