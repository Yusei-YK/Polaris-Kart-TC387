#ifndef KART_MENU_H_
#define KART_MENU_H_
#include "zf_common_headfile.h"

/*
 * IPS200 菜单系统（科目选择 + 语音控制 + 路线管理）
 * ------------------------------------------------------------------
 * 功能：取代 VOFA m 命令，通过屏幕菜单 + 五向按键完成科目切换、语音控制、
 *       路线录制和复现准备。所有界面顶部固定显示状态栏（Yaw/遥控在线/挡位）。
 *
 * 交互：UP/DOWN 光标移动，MID 确认/开始录制/结束录制，KART_LEFT 返回上级。
 *       旋转编码器等价接入：旋钮左右旋 = UP/DOWN，旋钮按下 = MID。
 *       调数值时旋钮比连点按键顺手，两套输入完全等价，可任意混用。
 *       RIGHT 刻意留空（"右=进入"与 MID 重复，误触会直接执行菜单项）。
 *
 * 手感（2026-07-28 重做，参考 TopSpeed）：
 *   · 输入采样在 10ms 拍（kart_menu_input_poll），画屏在 50ms 拍（kart_menu_poll）。
 *     以前两件事都挤在 50ms 拍，"按一下 → 整屏清 → 重画"，按键与旋钮手感一样迟钝。
 *   · 清屏与重绘拆开：只有换页才 ips200_clear()（软件 SPI 整屏十几 ms），
 *     移光标/改值靠定宽文本直接覆盖，改值更是只重画光标那一行。
 *   · UP/DOWN 按住不放会自动重复：300ms 后每 100ms 一次，按住超过 1s 自动升粗调
 *     （一次走 10 个步长）。旋钮连续快拧同样升粗调，慢拧保持细调。
 *   · 新增界面务必走 kart_menu.c 里的 ui_bar/ui_item（补空格到定宽），
 *     直接 ips200_show_string 写不定长串会留上一帧的尾巴。
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
 *   等待 START 键触发 kart_playback（由 kart_mission.c 状态机处理）。
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
 *   kart_menu_init()       —— cpu0_main.c 初始化段（IPS200/按键 GPIO 初始化后）
 *   kart_menu_input_poll() —— 10ms 拍（按键+旋钮采样，不碰屏）
 *   kart_menu_poll()       —— 50ms 拍（画屏）
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
#define MENU_ENC_A         (BOARD_ENC_A_PIN)
#define MENU_ENC_B         (BOARD_ENC_B_PIN)
#define MENU_ENC_SW        (BOARD_ENC_SW_PIN)

/* EC11 每个机械档位输出 4 个正交边沿。软件解码计满这么多计数才算"转过一格",
 * 否则转一下光标窜 4 行。若实测一格走多行就加大、转一格没反应就减小。 */
#define MENU_ENC_DIV       (4)

/* 待处理格数上限。快速猛旋一把可能攒下十几格,一次全兑成按键会让光标窜到底、
 * 或把参数一路顶到边界,不好收手,故截顶。 */
#define MENU_ENC_PEND_MAX  (8)

/* -------------------- 槽位分配 -------------------- */
#define KART_MENU_S1_SLOT_NUM   (1)             /* 科目一槽位数(Flash slot 0, 1500点) */
#define KART_MENU_S2_GATE_SLOT_NUM (5)          /* 科目二门洞槽位数(Flash slot 1~5, 每槽1000点) */
#define MENU_S2_RET_SLOT_NUM  (5)          /* 科目二返程槽位数(Flash slot 6~10, 每槽510点) */

/* -------------------- 对外接口 -------------------- */

/* 初始化：GPIO + IPS200 + 菜单状态复位。上电调一次。 */
void kart_menu_init(void);

/* 50ms 拍调：只画屏（换页清屏 / 页内覆盖 / 单行刷值三档）。不读按键。 */
void kart_menu_poll(void);

/* 10ms 拍调：旋钮正交解码 + 按键判定 + 菜单状态跳转。不碰屏，只读 GPIO
 * 加几个整数比较，进控制窗口无风险。
 * 必须 10ms（不能跟 kart_menu_poll 一起放 50ms）：EC11 一格 4 个正交边沿，
 * 手旋时单相最快约 25ms 一变，50ms 采样会漏边沿导致计数丢失/判错方向；
 * 长按自动重复也要靠这个拍做 100ms 分辨率。 */
void kart_menu_input_poll(void);

/* 单独暴露旋钮解码，供只想跑解码的场合（如硬件自检）。
 * 正常流程不用直接调，kart_menu_input_poll 内部已经调了 —— 两处都调会双倍计数。 */
void kart_menu_enc_poll(void);

#endif
