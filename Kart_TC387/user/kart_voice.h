#ifndef KART_VOICE_H_
#define KART_VOICE_H_

#include "zf_common_headfile.h"
#include "board_pins.h"

/*
 * 科目二离线语音识别 —— 接收解析层(ASR-PRO 模块)
 * ------------------------------------------------------------------
 * 只在科目二用。语音芯片(ASR-PRO)识别口令后,按约定帧格式往 UART 发命令,
 * 本模块负责收字节 → 状态机解析 → 校验 → 入队。执行层(灯光/鸣笛/门洞/运动)
 * 各自从队列 kart_voice_get_cmd() 取命令处理。
 *
 * 本模块绝不直接控制电机/转向/灯/喇叭 —— 只解析和排队(符合比赛规则)。
 *
 * 帧格式(8 字节): 6B 79 00 81 CMD 00 CHECK FB
 *   6B 79 00 81 : 固定帧头
 *   CMD         : 语音命令码 0x04~0x26 (共35条,见下方宏定义)
 *   00          : 固定填充字节
 *   CHECK       : 校验和 = (0x6B + 0x79 + 0x00 + 0x81 + CMD + 0x00) & 0xFF
 *   FB          : 固定帧尾
 *
 * 串口: UART_10 @115200 8N1 (BOARD_VOICE_UART_*, P13.0 TX / P13.1 RX)
 *       与 VOFA 日志(UART_10 @460800)同一 ASCLIN,按科目分时复用 ——
 *       一个 ASCLIN 只有一套波特率发生器,不可能两个模块同时挂。
 *       进科目二:kart_debug_uart_set_enabled(0) 关日志闸 → kart_voice_uart_acquire()
 *                切 115200;退出时 release() 切回 460800 再开闸(顺序相反,见 kart_mission.c)。
 *       比赛不许接无线模块,所以科二把无线模块拔掉、语音模块插同一排针即可。
 *       P33.12/P33.13 属 ASCLIN1,已被灯板 TLD7002(P11.12/P11.10 @2M)占用,不可用。
 * 收发: 主循环(科目二执行循环)查询 uart_query_byte,不进 RX 中断。
 * 触发: ASR-PRO 每识别一条口令发一帧,一条一帧不重复。
 * ------------------------------------------------------------------
 */

/* -------------------- 帧头 / 帧尾 / 长度 -------------------- */
#define KART_VOICE_FRAME_HEAD0      (0x6B)
#define KART_VOICE_FRAME_HEAD1      (0x79)
#define KART_VOICE_FRAME_HEAD2      (0x00)
#define KART_VOICE_FRAME_HEAD3      (0x81)
#define KART_VOICE_FRAME_TAIL       (0xFB)
#define KART_VOICE_FRAME_LEN        (8)         /* 6B 79 00 81 CMD 00 CHECK FB */

/* -------------------- 语音命令码(CMD 0x04~0x26,共35条) -------------------- */
/* 灯光类(0x04~0x0B,8条) */
#define KART_VOICE_CMD_LEFT_LIGHT       (0x04)  /* 打开左转向灯 */
#define KART_VOICE_CMD_RIGHT_LIGHT      (0x05)  /* 打开右转向灯 */
#define KART_VOICE_CMD_FAR_LIGHT        (0x06)  /* 打开远光灯 */
#define KART_VOICE_CMD_NEAR_LIGHT       (0x07)  /* 打开近光灯 */
#define KART_VOICE_CMD_FOG_LIGHT        (0x08)  /* 打开雾灯 */
#define KART_VOICE_CMD_DOUBLE_FLASH     (0x09)  /* 打开双闪灯 */
#define KART_VOICE_CMD_CABIN_LIGHT      (0x0A)  /* 打开车内照明灯 */
#define KART_VOICE_CMD_WIPER            (0x0B)  /* 打开雨刷器 */

/* 鸣笛类(0x0C~0x14,9条) */
#define KART_VOICE_CMD_HORN_1S          (0x0C)  /* 鸣笛一秒钟 */
#define KART_VOICE_CMD_HORN_2S          (0x0D)  /* 鸣笛两秒钟 */
#define KART_VOICE_CMD_HORN_3S          (0x0E)  /* 鸣笛三秒钟 */
#define KART_VOICE_CMD_HORN_2TIMES      (0x0F)  /* 鸣笛两声 */
#define KART_VOICE_CMD_HORN_3TIMES      (0x10)  /* 鸣笛三声 */
#define KART_VOICE_CMD_HORN_4TIMES      (0x11)  /* 鸣笛四声 */
#define KART_VOICE_CMD_HORN_LONG_SHORT  (0x12)  /* 长短鸣笛 */
#define KART_VOICE_CMD_HORN_URGENT      (0x13)  /* 急促鸣笛 */
#define KART_VOICE_CMD_HORN_ALARM       (0x14)  /* 警报鸣笛 */

/* 门洞类(0x15~0x1E,10条) */
#define KART_VOICE_CMD_GATE_1_LEFT      (0x15)  /* 通过门洞一左侧 */
#define KART_VOICE_CMD_GATE_1           (0x16)  /* 通过门洞一 */
#define KART_VOICE_CMD_GATE_2           (0x17)  /* 通过门洞二 */
#define KART_VOICE_CMD_GATE_3           (0x18)  /* 通过门洞三 */
#define KART_VOICE_CMD_GATE_3_RIGHT     (0x19)  /* 通过门洞三右侧 */
#define KART_VOICE_CMD_GATE_1_RIGHT_RET (0x1A)  /* 门洞一右侧返回 */
#define KART_VOICE_CMD_GATE_1_RET       (0x1B)  /* 门洞一返回 */
#define KART_VOICE_CMD_GATE_2_RET       (0x1C)  /* 门洞二返回 */
#define KART_VOICE_CMD_GATE_3_RET       (0x1D)  /* 门洞三返回 */
#define KART_VOICE_CMD_GATE_3_LEFT_RET  (0x1E)  /* 门洞三左侧返回 */

/* 动作类(0x1F~0x26,8条) */
#define KART_VOICE_CMD_FWD_10M          (0x1F)  /* 前行十米 */
#define KART_VOICE_CMD_BACK_10M         (0x20)  /* 后退十米 */
#define KART_VOICE_CMD_SNAKE_FWD_10M    (0x21)  /* 蛇形前进十米 */
#define KART_VOICE_CMD_SNAKE_BACK_10M   (0x22)  /* 蛇形后退十米 */
#define KART_VOICE_CMD_CCW_CIRCLE       (0x23)  /* 逆时针转一圈 */
#define KART_VOICE_CMD_CW_CIRCLE        (0x24)  /* 顺时针转一圈 */
#define KART_VOICE_CMD_TURN_LEFT        (0x25)  /* 左转 */
#define KART_VOICE_CMD_TURN_RIGHT       (0x26)  /* 右转 */

/* -------------------- 命令队列 -------------------- */
/* 指令间隔长、串行执行,深度 8 足够容纳一组(四区八条)的排队缓冲。 */
#define KART_VOICE_QUEUE_SIZE       (8)

typedef struct
{
    uint8 cmd;          /* 语音命令码 KART_VOICE_CMD_* (0x04~0x26) */
} kart_voice_cmd_t;

/* -------------------- 对外接口 -------------------- */

/* 启动时调一次:把语音串口(BOARD_VOICE_UART_*)配到 115200 + 清队列 + 复位状态机。 */
void kart_voice_init(void);

/* 科目二进入时调:抢占共用外设(切 115200)+ 清队列。
 * 与日志共用 UART_10 时,调用方必须先停日志再调本函数。 */
void kart_voice_uart_acquire(void);

/* 科目二退出时调:把共用外设还给 VOFA 日志(切回 460800)。 */
void kart_voice_uart_release(void);

/* 科目二执行循环里调:收干净 UART FIFO,逐字节喂状态机,合法帧入队。
 * 不阻塞、不控执行机构。 */
void kart_voice_poll(void);

/* 执行层调:从队列头取一条命令。
 * 有命令返回 1 并填 *out;队列空返回 0。 */
uint8 kart_voice_get_cmd(kart_voice_cmd_t *out);

/* 队列里待处理命令条数(执行层判断忙闲用)。 */
uint8 kart_voice_pending(void);

/* 调试接口:获取接收到的合法帧计数(校验通过并入队的)。 */
uint32 kart_voice_get_frame_count(void);

/* 调试接口:获取最后一帧的 CMD。 */
uint8 kart_voice_get_last_cmd(void);

/* 调试接口:获取收到的原始字节总数(判断UART是否真的收到数据)。 */
uint32 kart_voice_get_byte_count(void);

/* -------------------- 临时命令分发(待科目二状态机接管) -------------------- */
/* 取一条队列命令,按 CMD 范围路由到对应执行层:
 *   鸣笛(0x0C~0x14) → kart_horn_start()   已实现,可测
 *   灯光(0x04~0x0B)  → kart_light_set_command()  已接通,7x15 图案/动画
 *   门洞(0x15~0x19)  → kart_record_load_from_flash + kart_playback_start(槽1~5)
 *   返回(0x1A~0x1E)  → kart_mission_subject2_start_return()(2026-07-29 接通,槽6~10)
 *                       原来落 else 丢弃,现在走 GOTO 摆位 + 按录制原点复现两步
 *   动作(0x1F~0x26)  → kart_motion_start()
 * 每调一次处理一条,长动作(鸣笛)期间忙则本拍不取新命令,让当前动作跑完。
 * 注意:这是临时链路验证用。科目二状态机建好后,分发逻辑应挪进状态机,
 *      各执行层由状态机按区域调度,本函数删除。 */
void kart_voice_dispatch(void);

#endif
