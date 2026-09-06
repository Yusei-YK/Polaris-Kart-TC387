#ifndef KART_PEDAL_H_
#define KART_PEDAL_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

/*
 * 踏板 ECU 链路（CH32 油门/刹车盒 → TC387 速度环）
 * ------------------------------------------------------------------
 * 干什么：把车身外挂的一个 CH32 踏板盒接进速度环，让人坐上车用脚开。
 *         方向【完全靠机械】：人转方向盘直接拖动连杆，转向电机全程不使能、
 *         不通电，所以本模块除了把它按住在"断电"状态以外不碰转向。
 *
 * 为什么转向必须是"每拍写 0"而不是"不调 update"：
 *   power_set_steer_duty() 只写 Power_now.Servo_Duty，真正下发在 power_sync()。
 *   如果只是跳过 kart_steer_ctrl_update()，上一拍的 duty 会被 power_sync 一直
 *   重复写出去 —— 那正好是"和司机抢方向盘"。所以让 update 照常进、
 *   靠 angle_enable=0 走它自己的 duty=0 分支（见 kart_steer_ctrl.c）。
 *
 * 物理链路（2026-09 新增，单向）：
 *   CH32 USART1 TX(PD5) ──→ TC387 P02.1 (UART2_RX_P02_1, ASCLIN2)
 *   CH32 GND            ──→ TC387 GND
 *   TX 侧(P33_8)库函数要求必须给一个脚，实际不接线，留空。
 *   【为什么不是 P33.12】那是 UART1 的 TX 且 ASCLIN1 归灯板/4D7，
 *   见 board_pins.h 的 BOARD_PEDAL_* 段。
 *
 * 进出驾驶模式（只有一步）：
 *   进：主菜单光标停在 "Pedal Drive Manual"，按 MID → 直接进 MISSION_PEDAL，
 *       屏幕切到踏板运行页。没有预备态、不用按 START。
 *       菜单在切之前只查 kart_pedal_can_engage()，不齐就不切、屏幕闪一条
 *       "Pedal not ready"，见 kart_menu.c。不需要人做任何额外动作。
 *   出：按 KART_LEFT → 完整停机回 MISSION_IDLE（同科目三运行页的退出方式）。
 *       遥控油门杆动一下 → 抢占回 MISSION_REMOTE（遥控永远优先）。
 *       链路超时 → 断油但保留机械转向，见下。
 *
 * 合闸两个硬条件（kart_pedal_can_engage()，缺一个就不给切）：
 *   ① 链路在线    ② 油门千分比在死区内
 *   【为什么只剩这两条】它们都不要求人做动作：脚不放在油门上、线接好了，
 *   就自然满足，正常上车流程一次都不会被挡。②挡的是"油门线接触不良卡在半开"
 *   —— 少了它一按 MID 车就冲出去。
 *   【刹车不作为条件】按用户要求去掉了"必须踩着刹车才让进"：
 *   KART_PEDAL_REQUIRE_BRAKE 默认 0。那一条要求人在按菜单键的同时把脚踩在
 *   刹车上，是多一个动作；真正防冲车的是②。想加回来把那个宏改 1。
 *
 * 失联怎么办（这是本模块唯一一处"故意不停机"的地方）：
 *   超过 KART_PEDAL_LOST_TIMEOUT_MS 没收到合法帧 → 目标速度归零，但速度环
 *   【保持 enable=1】让 PID 主动把后轮拖到停；转向继续不使能，司机手上
 *   还有机械转向。故意不跳 MISSION_FAULT：人正坐在车上以速度行驶时，
 *   把控制权整体推进故障态比"断油 + 保留转向"更危险。
 */
/*
 * 协议：8 字节定长，100Hz，115200 8N1，单向无应答
 *   [0] 0xAA        帧头0
 *   [1] 0x55        帧头1
 *   [2] seq         每帧 +1，用来数丢帧(kart_pedal_stat_t.seq_lost)
 *   [3] thr_lo      油门 0..1000 千分比，小端
 *   [4] thr_hi
 *   [5] flags       bit0 刹车  bit1 挡位(1=手动)  bit2 踏板盒自检 OK
 *   [6] check       [0]^[1]^[2]^[3]^[4]^[5]
 *   [7] 0x0D        尾字节，丢字节后靠它重同步
 *   【为什么不用结构体直传】CH32 那边 uint16 会让编译器塞对齐填充字节，
 *   sizeof 是 8 不是 7，校验范围会把 check 自己算进去。本仓库既有做法见
 *   kart_assist_img.c 的 camera_v2_head[12]：全部 uint8 明确布局。
 *
 * 调用位置：
 *   kart_pedal_init()            —— cpu0_main.c 初始化段（uart 初始化之后）
 *   kart_pedal_rx_callback()     —— user/isr.c 的 uart2_rx_isr
 *   kart_pedal_poll(10)          —— 10ms 拍（解帧 + 失联计时 + 遥控抢占）
 *   kart_pedal_control_update()  —— 5ms 拍，必须在 kart_steer_ctrl_update() 之后
 * ------------------------------------------------------------------
 */

/* 1 = 编译进来；0 = 全部函数编译期空壳，零开销、零 RAM。 */
#define PEDAL_ENABLE                (1)

/* -------------------- 协议常量 -------------------- */
#define PEDAL_FRAME_LEN             (8U)
#define PEDAL_HEAD0                 (0xAAU)
#define PEDAL_HEAD1                 (0x55U)
#define PEDAL_TAIL                  (0x0DU)
#define PEDAL_CHECK_COVER_LEN       (6U)        /* 异或覆盖 [0..5] */

#define PEDAL_FLAG_BRAKE            (0x01U)
#define PEDAL_FLAG_MANUAL           (0x02U)
#define PEDAL_FLAG_SELFTEST_OK      (0x04U)

#define PEDAL_BAUD                  (115200U)
#define PEDAL_THROTTLE_PM_MAX       (1000U)     /* CH32 侧 Throttle_Map 的满量程 */

/* -------------------- 诊断量 -------------------- */
typedef struct
{
    uint32 rx_bytes;        /* 收到的字节总数(含垃圾) */
    uint32 frame_ok;        /* 校验通过的帧数 */
    uint32 frame_bad;       /* 校验/帧头/尾字节失败丢弃的帧数 */
    uint32 seq_lost;        /* 按 seq 断层累计的丢帧数 */
    uint32 engage_deny;     /* 按 MID 想进但三条件没过、被菜单拒绝的次数 */
    uint32 link_lost_cnt;   /* 失联次数(在线→离线的跳变) */
    uint16 last_thr_pm;     /* 最近一帧油门千分比 */
    uint8  last_flags;      /* 最近一帧 flags 原样 */
    uint8  online;          /* 当前是否在线 */
}kart_pedal_stat_t;

/* -------------------- 对外接口 -------------------- */

/* 初始化：UART_2 + RX 中断 + 状态复位。上电调一次。 */
void  kart_pedal_init(void);

/* uart2_rx_isr 里调：取一个字节进环形缓冲，不解析。 */
void  kart_pedal_rx_callback(void);

/* 10ms 拍：解帧、失联计时、遥控抢占。 */
void  kart_pedal_poll(uint16 period_ms);

/* 5ms 拍：油门 → 速度环目标 + 把转向电机按在断电状态。
 * 必须放在 kart_steer_ctrl_update() 之后、power_sync() 之前。 */
void  kart_pedal_control_update(void);

/* kart_mission_set_mode() 的 MISSION_PEDAL 进出钩子。 */
void  kart_pedal_enter(void);
void  kart_pedal_exit(void);

/* 查询 */
uint8  kart_pedal_is_online(void);
uint8  kart_pedal_can_engage(void);     /* 两个硬条件是否都满足 */
uint16 kart_pedal_get_throttle_pm(void);
uint8  kart_pedal_get_brake(void);
uint8  kart_pedal_is_reverse(void);    /* 1 = 当前 R 挡(刹车踩住且车已停) */
float  kart_pedal_get_target_ms(void);  /* 本拍下发的目标速度(m/s)，遥测/屏幕用 */
void   kart_pedal_get_stat(kart_pedal_stat_t *dst);

/* 菜单切模式被拒时记一次，供屏幕和日志看。 */
void   kart_pedal_note_deny(void);


#endif  /* KART_PEDAL_H_ */
