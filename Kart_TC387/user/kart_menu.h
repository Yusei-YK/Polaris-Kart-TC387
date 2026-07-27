#ifndef KART_MENU_H_
#define KART_MENU_H_

#include "zf_common_headfile.h"

/*
 * IPS200 菜单系统（科目选择 + 语音控制 + 路线管理）
 * ------------------------------------------------------------------
 * 功能：取代 VOFA m 命令，通过屏幕菜单 + 五向按键完成科目切换、语音控制、
 *       路线录制和复现准备。所有界面顶部固定显示状态栏（Yaw/遥控在线/挡位）。
 *
 * 交互：UP/DOWN 光标移动，MID 确认/开始录制/结束录制，LEFT 返回上级。
 *       2026-07-28 起旋转编码器也接入：旋钮左右旋 = UP/DOWN，旋钮按下 = MID。
 *       调数值时旋钮比连点按键顺手，两套输入完全等价，可任意混用。
 *
 * 菜单结构（3级）：
 *   [一级] Subject 1 / Subject 2
 *   [二级-科目一] Record / Playback / Enter Remote / Exit Remote
 *   [二级-科目二] Voice Control / Gate Recording / Back
 *   [三级-语音控制] 显示"Speak Command"，解析语音执行，按MID退出
 *   [三级-门洞录制] Record / Playback / Back
 *   [四级-门洞槽位] Gate1 Left / Gate1 / Gate2 / Gate3 / Gate3 Right
 *
 * 录制流程（科目一）：
 *   选"Record" → 进遥控(m3) → 屏显"Press MID to Start" → 用户遥控到起点 →
 *   按MID开始录制(r1) → 屏显录制点数实时刷新 → 按MID结束录制(r0) →
 *   弹出保存菜单（Slot1/Don't Save）→ 选槽位执行 w<slot> → 返回二级菜单。
 *
 * 录制流程（科目二门洞）：
 *   选Gate Recording → Record → 选门洞槽位(1~5) → 进遥控 → 录制 → 保存到对应槽位
 *
 * 复现流程：
 *   选"Playback" → 弹出槽位选择（显示[已录xxx点]/[Empty]）→
 *   选中后自动执行 z(清里程) + f<slot>(加载) + 退出遥控模式 →
 *   等待 START 键触发 playback（由 kart_mission.c 状态机处理）。
 *
 * 语音控制流程：
 *   选"Voice Control" → 显示"Speak Command" → 调用kart_odom_reset()清零 →
 *   开始解析语音(kart_voice_poll/dispatch) → 执行灯光/鸣笛/运动/门洞命令 →
 *   按MID退出返回二级菜单
 *
 * 槽位分配：
 *   科目一：槽位 0（Flash slot 0，1500点）
 *   科目二门洞：槽位 1~5（Flash slot 1~5，每槽1000点）
 *
 * 调用位置：
 *   kart_menu_init()  —— cpu0_main.c 初始化段（IPS200/按键 GPIO 初始化后）
 *   kart_menu_poll()  —— 主循环每拍（替代 KART_HW_TEST 的死循环）
 * ------------------------------------------------------------------
 */

/* -------------------- 按键引脚 --------------------
 * 2026-07-28 换按键板:引脚统一在 board_pins.h 定义(那里有网表出处和占用核对),
 * 这里只做别名,避免同一个脚在两处各写一遍号码、改板时漏改一处。 */
#include "board_pins.h"

#define KART_MENU_KEY_UP        (BOARD_KEY_UP_PIN)
#define KART_MENU_KEY_DOWN      (BOARD_KEY_DOWN_PIN)
#define KART_MENU_KEY_MID       (BOARD_KEY_MID_PIN)     /* 确认/开始录制/结束录制 */
#define KART_MENU_KEY_LEFT      (BOARD_KEY_LEFT_PIN)
#define KART_MENU_KEY_RIGHT     (BOARD_KEY_RIGHT_PIN)

/* 旋转编码器:转动=光标/改值(等价连按 UP/DOWN),按下=确认(等价 MID)。
 * 转一格发一次按键事件,长按 UP/DOWN 的活儿现在可以直接旋。 */
#define KART_MENU_ENC_A         (BOARD_ENC_A_PIN)
#define KART_MENU_ENC_B         (BOARD_ENC_B_PIN)
#define KART_MENU_ENC_SW        (BOARD_ENC_SW_PIN)

/* EC11 每个机械档位输出 4 个正交边沿。软件解码计满这么多计数才算"转过一格",
 * 否则转一下光标窜 4 行。若实测一格走多行就加大、转一格没反应就减小。 */
#define KART_MENU_ENC_DIV       (4)

/* 待处理格数上限。快速猛旋一把可能攒下十几格,一次全兑成按键会让光标窜到底、
 * 或把参数一路顶到边界,不好收手,故截顶。 */
#define KART_MENU_ENC_PEND_MAX  (8)

/* -------------------- 槽位分配 -------------------- */
#define KART_MENU_S1_SLOT_NUM   (1)             /* 科目一槽位数(Flash slot 0, 1500点) */
#define KART_MENU_S2_GATE_SLOT_NUM (5)          /* 科目二门洞槽位数(Flash slot 1~5, 每槽1000点) */

/* -------------------- 对外接口 -------------------- */

/* 初始化：GPIO + IPS200 + 菜单状态复位。上电调一次。 */
void kart_menu_init(void);

/* 主循环 50ms 拍调：按键扫描 + 状态机更新 + 屏幕刷新。 */
void kart_menu_poll(void);

/* 旋钮正交解码，必须放 10ms 拍(不能跟 kart_menu_poll 一起放 50ms)。
 * EC11 手旋时单相最快约 25ms 一变，50ms 采样会漏边沿导致计数丢失/判错方向。
 * 只读两个 GPIO 加几个整数比较，耗时可忽略。 */
void kart_menu_enc_poll(void);

#endif
