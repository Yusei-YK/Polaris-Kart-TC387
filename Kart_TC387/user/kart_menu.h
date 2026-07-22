#ifndef KART_MENU_H_
#define KART_MENU_H_

#include "zf_common_headfile.h"

/*
 * IPS200 菜单系统（科目选择 + 语音控制 + 路线管理）
 * ------------------------------------------------------------------
 * 功能：取代 VOFA m 命令，通过屏幕菜单 + 五向按键完成科目切换、语音控制、
 *       路线录制和复现准备。所有界面顶部固定显示状态栏（Yaw/遥控在线/挡位）。
 *
 * 交互：UP/DOWN 光标移动，MID(P20.7) 确认/开始录制/结束录制，LEFT 返回上级。
 *       旋钮暂不用于菜单导航。
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

/* -------------------- 按键引脚（复用 kart_hw_test.h 定义）-------------------- */
#define KART_MENU_KEY_UP        (P33_11)
#define KART_MENU_KEY_DOWN      (P20_6)
#define KART_MENU_KEY_MID       (P20_7)         /* 确认/开始录制/结束录制 */
#define KART_MENU_KEY_LEFT      (P33_4)
#define KART_MENU_KEY_RIGHT     (P21_7)

/* -------------------- 槽位分配 -------------------- */
#define KART_MENU_S1_SLOT_NUM   (1)             /* 科目一槽位数(Flash slot 0, 1500点) */
#define KART_MENU_S2_GATE_SLOT_NUM (5)          /* 科目二门洞槽位数(Flash slot 1~5, 每槽1000点) */

/* -------------------- 对外接口 -------------------- */

/* 初始化：GPIO + IPS200 + 菜单状态复位。上电调一次。 */
void kart_menu_init(void);

/* 主循环每拍调：按键扫描 + 状态机更新 + 屏幕刷新。 */
void kart_menu_poll(void);

#endif
