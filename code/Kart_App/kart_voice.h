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
 *
 * 【当前编译配置下本模块是活的】两个开关决定它的形态,都在 board_pins.h:
 *   BOARD_VOICE_SHARES_AUX_UART = VOICE_ON_AUX_UART(1) && !LOG_ON_UART0(0) = 1
 *     → 分时复用那条路成立,所以 kart_voice_init() 里那段独占式 uart_init 是
 *       #if 掉的,init 实际不配串口;115200 是 acquire() 现场切的。
 *   VOICE_MUTED = PERSON_LINK_ENABLE(0) && ... = 0
 *     → 语音没被静音。它只在 4D7 人体视觉链路选 VOFA 口(UART_10)时才变 1,
 *       那时语音模块的座子被占,硬件上确实不在,整个模块自动空转。
 *   改了 PERSON_LINK_* 之后要回头核这一段,别照着旧结论判断语音有没有工作。
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
/* 帧长常量,全仓库没有读者:解析器是逐字节状态机(kart_voice.c 的 voice_feed_byte),
 * 8 个状态一对一盯 8 个字节,不需要长度。留着是因为它是协议事实的一部分 ——
 * 换模块/改协议时先看这里对不上对不上。 */
#define KART_VOICE_FRAME_LEN        (8)         /* 6B 79 00 81 CMD 00 CHECK FB */

/* -------------------- 语音命令码(CMD 0x04~0x26,共35条) -------------------- */
/* 分四类:灯光 0x04~0x0B(8) 鸣笛 0x0C~0x14(9) 门洞 0x15~0x1E(10) 动作 0x1F~0x26(8),
 * 8+9+10+8 = 35 = 0x26-0x04+1,铺满不留空号。
 *
 * 【35 个名字里 12 个真被代码引用,23 个只是协议表 —— 不是死代码,别删】
 * 三种用法各不相同,改编号前先分清自己在改哪种:
 *   动作 8 条:名字是真的分支标签,kart_motion.c:611~705 逐条 case。
 *             kart_voice_dispatch 把原始码整个传给 kart_motion_start(),
 *             改这里的编号,motion 会跟着走,不会错位。
 *   灯光、鸣笛:只用到首尾两个当区间上下界 ——
 *             灯光 LEFT_LIGHT..WIPER 在 kart_voice.c 的 dispatch,
 *             鸣笛 HORN_1S..HORN_ALARM 在 kart_mission.c:1013(科目三信号阶段),
 *             中间那 6 + 7 个名字没有读者,靠"码 - 首码"算偏移。
 *             所以改编号必须保持【连续且顺序不变】,否则偏移全错。
 *   门洞 10 条:一个读者都没有。dispatch 写的是裸 0x15~0x19 / 0x1A~0x1E,
 *             改这里的编号编译不报错、行为也不跟着变 —— 要改就同时改那两条分支。
 * 表本身照引脚表看待:记录 ASR-PRO 固件里烧的口令编号,人照着查。 */
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
/* 【实际能存 7 条,不是 8 条】环形队列留一个空位区分满/空(kart_voice.c 的
 * voice_queue_push:next == head 就算满),所以可用深度是 SIZE-1。
 * 满了不是丢新的,是丢最旧的一条 —— 一口气来 8 条会掉掉第 1 条。
 * 现场不会这样:ASR-PRO 一条口令一帧,人说话的间隔远大于执行一条的时间,
 * 执行层每拍取一条。要真按"一组八条"留余量就把这个数改成 9。 */
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

/* -------------------- 通用命令分发 -------------------- */
/* 取一条队列命令,按 CMD 范围路由到对应执行层:
 *   鸣笛(0x0C~0x14) → kart_horn_start()   已实现,可测
 *   灯光(0x04~0x0B)  → kart_light_set_command()  已接通,7x15 图案/动画
 *   门洞(0x15~0x19)  → kart_record_load_from_flash + kart_playback_start(槽1~5)
 *   返回(0x1A~0x1E)  → 两条路,由 kart_mission_subject2_get_manual_return() 选:
 *                       0 = Voice A → kart_mission_subject2_start_return()
 *                           (2026-07-29 接通,槽6~10;原来落 else 丢弃,
 *                            现在走 GOTO 摆位 + 按录制原点复现两步)
 *                       1 = Voice B → kart_mission_subject2_start_return_here()
 *                           跳过 GOTO,按车当前位姿复现 —— 人遥控把车摆回集结点,
 *                           等于把 kart_odom 的累计漂移一次性归零。
 *                       开关在菜单 S2 Voice 页(kart_menu.c:1180 显示 Voice A/B)。
 *   动作(0x1F~0x26)  → kart_motion_start()
 * 每调一次处理一条,长动作(鸣笛)期间忙则本拍不取新命令,让当前动作跑完。
 * 科目二用它执行灯光、鸣笛、门洞和固定动作；科目三信号阶段
 * 另行限制只接受灯光和鸣笛命令。 */
void kart_voice_dispatch(void);

#endif
