#ifndef KART_TRAJ_VIEW_H_
#define KART_TRAJ_VIEW_H_
#include "zf_common_headfile.h"

/*
 * 科目一录制轨迹可视化(IPS200 屏上画出录好的路径)
 * ------------------------------------------------------------------
 * 【当前状态:已接入菜单】
 *   Subject 1 → View Sampled Path。车辆静止时查看或修正前进段。
 *
 * 画什么:
 *   · 路径本体 —— 相邻路径点连线;前进段蓝、倒车段红(按录制速度符号判,
 *     判据与 kart_playback.c 的倒车段判定一致);
 *   · 每个真实采样点再画一颗深蓝点,可直接观察采样密度;
 *   · 起点蓝方块、终点红方块;
 *   · 当前光标位置一颗亮黄方块(比起终点小一号,最后画,盖在路径上面);
 *   · 自动缩放:取路径包围盒等比例铺满绘图区,长短路径都占满屏;
 *   · 第一行数字:点数、光标序号 P、VIEW/EDIT、改过没存的 *、
 *     以及光标落在倒车段时的 LOCK;
 *   · 第二行数字:包围盒尺寸、当前比例(px/m);
 *   · 底部一行按键提示,VIEW 和 EDIT 两态不同。
 *
 * 【总里程没画在屏上】kart_record_get_total_dist() 现在还在 .c 里算一遍并
 *   格式化进 na,但紧跟的 sprintf 没用 na,再往下 na 就被包围盒宽度覆盖了。
 *   也就是说这个数被算出来又丢掉了。要看总里程去 Playback 页或 VOFA,
 *   或者自己把它插回第一行(第一行现在最宽 26 字,还有余量)。
 *
 * 坐标系:直接用 kart_record 的录制起点车体系(x 向右、y 向前)。
 *   屏上 x 向右 = 车右,屏上 y 向上 = 车前(屏幕 y 轴朝下,故取负号)。
 *   所以看到的图就是"站在起点、朝车头方向看下去"的俯视图。
 *
 * 数据来源:科目一 Flash 槽 0。菜单进入本页时先调
 *   kart_record_load_from_flash(0) 把持久化路径读回 kart_record RAM 缓冲,
 *   再由本页通过 kart_record_get_waypoints() / _get_count() / _get_total_dist()
 *   统一绘制。因此冷启动后不需要先走 Playback 载入。
 *
 * 【为什么只在操作后重画】
 *   这块屏是软件 SPI,ips200_draw_point 每画一个像素都要重发一次开窗命令
 *   (0x2A/0x2B/0x2C + 4 个 16bit 坐标),单点开销约十几 us。整页轨迹一次画完
 *   要几十 ms —— 只能在进页、移动选择或修正后重画,不能每 50ms 重画。
 *   另一个硬约束:不清屏就没法擦掉旧线(填一遍绘图区要 4 万多个点,太慢),
 *   所以本函数必须跑在 ui_clear() 之后 —— 走 need_clear 那条路径正好满足。
 *
 * 【为什么不做实时描点】
 *   录制在 CPU2(kart_record_poll),菜单画屏在 CPU0 的 50ms 拍。边跑边描点会把
 *   几十 ms 的 SPI 写插进控制窗口,得不偿失。录完停车再看。
 */

#define TRAJ_VIEW_ENABLE   (1)

#if TRAJ_VIEW_ENABLE

/* 画一整页轨迹图。必须在整屏清过之后调 —— 菜单侧那个函数叫 ui_clear(),
 * 由 need_clear 触发(见上方说明)。
 * 路径点不足 2 个时显示提示文字,不画图。 */
void kart_traj_view_draw(void);

/* 菜单在重画前传入当前选择和编辑状态。 */
void kart_traj_view_set_cursor(uint16 index, uint8 editing, uint8 dirty);

#endif

#endif
