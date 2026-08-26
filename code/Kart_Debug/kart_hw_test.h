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
 * 【当前状态 2026-08-26:开关常 0,不参与运行】
 *      KART_HW_TEST 在 cpu0_main.c 是 (0),唯一调用点被它包着,整季没开过。
 *      并口屏/旋钮/按键现在天天在用(菜单就跑在这三样上),自测的作用已经被日常
 *      使用覆盖。代码留着当换板时的落地工具 —— 换主板或改按键板网表后,开它比在
 *      正式主循环里排查快。开之前记住:内部是死循环、不返回、不跑正式主循环。
 *
 * 用法(二选一,只在自测时开):
 *   在 cpu0_main.c 顶部 #define KART_HW_TEST 1,烧录。
 *   现象:开机先全屏刷 红→绿→蓝(各 500ms)验证刷屏无花点,
 *        然后常驻显示旋钮计数 + 6 个按键实时电平。
 *   转旋钮看 ENC 数字增减;按键按下看对应位 0/1 翻转。
 *
 * 引脚:2026-07-28 起统一在 board_pins.h 定义(那里有 按键板.tel 网表出处),
 *      这里只做别名。旧板的实测修正(DOWN=P20.6、MID=P20.7、KART_LEFT 顶 P33.4)已作废,
 *      新板 SW2 五个方向各自独立,不再有 UP/KART_LEFT 短路。
 *   屏并口数据 D0~D7 = P11.9~P11.12 / P13.0~P13.3,控制线 P15.0~P15.5
 * ------------------------------------------------------------------
 */
#include "board_pins.h"

/* 旋钮引脚(EC11:A/B 双相 + 按下) */
#define KART_KNOB_A_PIN         (BOARD_ENC_A_PIN)
#define KART_KNOB_B_PIN         (BOARD_ENC_B_PIN)
#define KART_KNOB_SW_PIN        (BOARD_ENC_SW_PIN)

/* 5 向按键 + 独立发车键 */
#define KART_KEY_UP_PIN         (BOARD_KEY_UP_PIN)
#define KART_KEY_DOWN_PIN       (BOARD_KEY_DOWN_PIN)
#define KART_KEY_LEFT_PIN       (BOARD_KEY_LEFT_PIN)
#define KART_KEY_RIGHT_PIN      (BOARD_KEY_RIGHT_PIN)
#define KART_KEY_MID_PIN        (BOARD_KEY_MID_PIN)
#define KEY_START_PIN      (BOARD_START_KEY_PIN)   /* SW3 独立轻触键 */

/* 自测主入口:内部死循环,不返回。由 cpu0_main 在 KART_HW_TEST 开时调用。 */
void kart_hw_test_run(void);

#endif
