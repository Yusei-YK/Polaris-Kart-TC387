#ifndef KART_REMOTE_H_
#define KART_REMOTE_H_

#include "zf_common_headfile.h"
#include "kart_calib.h"     /* 通道端点标定、KART_REMOTE_STEER_SIGN */
#include "board_pins.h"

/*
 * SBUS 枪式遥控接收 —— 解析 + 失联兜底层(第一版:纯观测,不接管控制)
 * ------------------------------------------------------------------
 * 硬件: 接收机只接 5V/GND/TX,TX → 主板 P15.7(UART3 RX)。
 * 串口: UART_3 @100000 8E2(SBUS 标准),uart_sbus_init() 已内含电平反相,
 *       无需外加反相电路。TX 脚 P15.6 仅初始化占位,不实际发送。
 * 中断: 走 UART3 RX 中断(isr.c 的 uart3_rx_isr),回调逐字节收满 25 字节、
 *       校验帧头 0x0F / 帧尾 0x00 后解析 6 通道。独立于语音(UART_2)与
 *       无线调试(UART_1/set_wireless_type),互不干涉。
 *
 * 帧格式: 25 字节,byte[0]=0x0F 帧头,byte[24]=0x00 帧尾,
 *         6×11bit 通道打包在 byte[1..9],byte[23]&0x04 = 失控标志。
 *
 * !!! 安全红线(吸取 2026-07-06 转向打死烧驱动 + 电流倒灌烧电源事故)!!!
 *   本版只做"解析 + 失联计数 + VOFA 观测",绝不接管电机/转向。
 *   确认收帧稳定、失联判据生效后,再单独实现接管层,且接管必须同时满足:
 *     ① SBUS→转角目标钳到软限位 KART_STEER_DELTA_LIMIT_L/R;
 *     ② 失联超时兜底(连续 N 拍无有效帧 → 转向回中 + 速度清零 + 关使能);
 *     ③ 帧校验,坏帧丢弃不更新目标。
 * ------------------------------------------------------------------
 */

/* -------------------- SBUS 帧参数 -------------------- */
#define KART_REMOTE_CHANNEL_NUM     (6)         /* 解析通道数 */
#define KART_REMOTE_FRAME_LEN       (25)        /* SBUS 帧长 */
#define KART_REMOTE_FRAME_HEAD      (0x0F)      /* 帧头 */
#define KART_REMOTE_FRAME_END       (0x00)      /* 帧尾 */
#define KART_REMOTE_LOST_BIT        (0x04)      /* byte[23] & 0x04 置位 = 失控 */
#define KART_REMOTE_BAUD            (100000)    /* SBUS 波特率 */

/* -------------------- 通道语义(枪式遥控器说明书) -------------------- */
/* 通道值 11bit,范围 0~2047,中值约 1024。下标从 0 起对应 CH1~CH6。 */
#define KART_REMOTE_CH_STEER        (0)         /* CH1 方向 */
#define KART_REMOTE_CH_THROTTLE     (1)         /* CH2 油门 */
#define KART_REMOTE_CH_SW3          (3)         /* CH4 三段开关 */
/* CH3(idx2)/CH5(idx4)/CH6(idx5) 为两态按键,接管阶段再定义用途 */

/* -------------------- 失联判据 -------------------- */
/* 主循环每拍(5ms)调 kart_remote_poll() 检查:若自上次有效帧以来
 * 累计超过 KART_REMOTE_LOST_TIMEOUT_MS 无新帧,判失联。
 * SBUS 正常约 14ms/帧,阈值取 100ms(约 7 帧余量),既不误报又能快速兜底。 */
#define KART_REMOTE_LOST_TIMEOUT_MS (100)

/* 通道端点标定(方向/油门/三段开关的中位与满行程)→ kart_calib.h 第七节。
 * 换遥控器或重做遥控器行程校准后只改那里。 */

/* -------------------- 接管参数(已与用户确认) -------------------- */
#define KART_REMOTE_DEADZONE        (60)     /* 方向/油门中位死区,防抖动漂移(07-20 实车加大) */
/* 满油门目标速度(脉冲/5ms)。运行时可由 kart_params 覆盖(菜单 RC Vmax),此宏为出厂默认。
 * 07-20 曾因"实车全速偏快"下调到 20;07-25 22:00 科目一/四实车高速跑通时本地是 60,
 * 但该改动未进提交,07-26 00:17 又回到 20 → 07-27 科目一录制与复现速度只剩 1/3
 * (速度环是闭环,目标只有 20,换满电电池也不会更快)。07-27 上调,当前值 50。
 * 【2026-07-28 注释纠错】原文写"07-27 恢复 60"与实际值 50 不符,已按实际值改写。
 * 【原"必须与 KART_PLAYBACK_SPEED_MAX 同步"已作废】剖面(PB Prof)出厂打开后,
 * 复现速度由路径几何算,与录制速度无关。录制反而应该录慢些:路径更准、不打滑。
 * 本值现在只决定【录制/遥控手感】,不再决定复现快慢,两者不需要同步。 */
#define KART_REMOTE_MAX_SPEED       (50.0f)

/* 摇杆左右方向 KART_REMOTE_STEER_SIGN → kart_calib.h 第六节(控制环反馈符号) */

/* 三段开关挡位语义(接管总闸):
 *   LOW  = 全关急停(速度=0关使能;转向内环回中 target_delta=0)
 *   MID  = 只转向(转向内环随方向摇杆;速度=0关使能)
 *   HIGH = 转向 + 速度(转向随摇杆,速度随油门) */
typedef enum
{
    KART_REMOTE_SW3_L = 0,  /* 低挡 */
    KART_REMOTE_SW3_M,      /* 中挡 */
    KART_REMOTE_SW3_H,      /* 高挡 */
} kart_remote_sw3_t;

/* -------------------- 对外接口 -------------------- */

/* 初始化:uart_sbus_init(UART_3) + 挂 RX 中断回调。上电调一次。 */
void kart_remote_init(void);

/* 主循环每拍调:更新失联计时/状态。不接管任何控制。
 * period_ms = 本次调用间隔(通常 KART_MAIN_LOOP_PERIOD_MS)。 */
void kart_remote_poll(uint16 period_ms);

/* RX 中断回调:isr.c 的 uart3_rx_isr 里调。逐字节收帧,合法帧解析入结构。 */
void kart_remote_rx_callback(void);

/* -------------------- 接管控制(只在 MISSION_REMOTE 模式下调用) -------------------- */
/* 遥控接管一拍:读通道 → 死区/限幅/软限位 → 驱动转向内环 + 速度环。
 * 失联(超时或接收机失控)或三段低挡 → 强制急停(速度0关使能、转向回中)。
 * 由 kart_mission_poll() 的 REMOTE loop 调用,设的目标转角/速度下一拍被
 * kart_steer_ctrl_update()/速度环执行(与科目一同架构,5ms 延迟无感)。
 * 全程硬钳软限位 KART_STEER_DELTA_LIMIT_L/R,遵守 07-06 防打死红线。 */
void kart_remote_control_update(void);

/* 接管退出时调:速度清零关使能、转向回中关内环(等同急停)。
 * mission 切出 REMOTE 模式时由 stop_all 统一处理,此接口备用。 */
void kart_remote_control_stop(void);

/* 读当前三段挡位(VOFA/调试用)。 */
kart_remote_sw3_t kart_remote_get_sw3(void);

/* -------------------- 观测接口(只读) -------------------- */

/* 取某通道原始值(0~2047)。idx 越界返回 0。 */
uint16 kart_remote_get_channel(uint8 idx);

/* 遥控是否在线(收到有效帧且未超时且接收机未报失控):在线 1 / 失联 0。 */
uint8 kart_remote_is_online(void);

/* 校验通过的合法帧累计计数(VOFA 观测收帧是否稳定)。 */
uint32 kart_remote_get_frame_count(void);

/* 接收机自身的失控标志解析结果(byte[23]&0x04):1=正常,0=失控。
 * 与 is_online 区分:此为遥控器→接收机链路状态,online 还叠加了主控侧超时。 */
uint8 kart_remote_get_signal_state(void);

#endif
