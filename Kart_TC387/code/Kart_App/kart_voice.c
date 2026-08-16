#include "kart_voice.h"
#include "kart_light.h"
#include "kart_horn.h"
#include "kart_motion.h"
#include "kart_record.h"
#include "kart_playback.h"
#include "kart_mission.h"

/*
 * 科目二离线语音识别 —— 接收解析层实现
 * ------------------------------------------------------------------
 * 收字节 → 8 状态机解析 6B 79 00 81 CMD 00 CHECK FB → 校验 → 入环形队列。
 * 执行层从队列取命令,本模块不碰任何执行机构。
 * ------------------------------------------------------------------
 */

/* -------------------- 解析状态机 -------------------- */
typedef enum
{
    VOICE_WAIT_HEAD0 = 0,   /* 等 0x6B */
    VOICE_WAIT_HEAD1,       /* 等 0x79 */
    VOICE_WAIT_HEAD2,       /* 等 0x00 */
    VOICE_WAIT_HEAD3,       /* 等 0x81 */
    VOICE_GET_CMD,          /* 收 CMD */
    VOICE_GET_PAD,          /* 收 0x00 填充字节 */
    VOICE_GET_CHECK,        /* 收 CHECK 并校验 */
    VOICE_GET_TAIL,         /* 收 0xFB 帧尾 */
} kart_voice_state_t;

static kart_voice_state_t voice_state = VOICE_WAIT_HEAD0;
static uint8 voice_rx_cmd  = 0;

static volatile uint32 voice_frame_count = 0;
static volatile uint8  voice_last_cmd    = 0;
static volatile uint32 voice_byte_count  = 0;

/* -------------------- 环形队列 -------------------- */
/* head 取、tail 存;留一个空位区分满/空,故实际可存 SIZE-1 条。 */
static kart_voice_cmd_t voice_queue[KART_VOICE_QUEUE_SIZE];
static volatile uint8   voice_q_head = 0;
static volatile uint8   voice_q_tail = 0;

static void voice_queue_reset(void)
{
    voice_q_head = 0;
    voice_q_tail = 0;
}

/* 入队。满则丢弃最旧一条腾位(比赛口令不能丢新的,宁可挤掉最早未执行的)。
 * 实际队列深 8、指令间隔长,正常跑不会满,此处只是兜底。 */
static void voice_queue_push(uint8 cmd)
{
    uint8 next = (uint8)((voice_q_tail + 1) % KART_VOICE_QUEUE_SIZE);

    if(next == voice_q_head)
    {
        /* 队满:丢弃最旧一条(head 前移),再存新命令。 */
        voice_q_head = (uint8)((voice_q_head + 1) % KART_VOICE_QUEUE_SIZE);
    }

    voice_queue[voice_q_tail].cmd = cmd;
    voice_q_tail = next;
}

/* -------------------- 单字节喂状态机 -------------------- */
static void voice_feed_byte(uint8 byte)
{
    switch(voice_state)
    {
        case VOICE_WAIT_HEAD0:
            if(byte == KART_VOICE_FRAME_HEAD0)
            {
                voice_state = VOICE_WAIT_HEAD1;
            }
            break;

        case VOICE_WAIT_HEAD1:
            if(byte == KART_VOICE_FRAME_HEAD1)
            {
                voice_state = VOICE_WAIT_HEAD2;
            }
            else if(byte == KART_VOICE_FRAME_HEAD0)
            {
                voice_state = VOICE_WAIT_HEAD1;
            }
            else
            {
                voice_state = VOICE_WAIT_HEAD0;
            }
            break;

        case VOICE_WAIT_HEAD2:
            if(byte == KART_VOICE_FRAME_HEAD2)
            {
                voice_state = VOICE_WAIT_HEAD3;
            }
            else if(byte == KART_VOICE_FRAME_HEAD0)
            {
                voice_state = VOICE_WAIT_HEAD1;
            }
            else
            {
                voice_state = VOICE_WAIT_HEAD0;
            }
            break;

        case VOICE_WAIT_HEAD3:
            if(byte == KART_VOICE_FRAME_HEAD3)
            {
                voice_state = VOICE_GET_CMD;
            }
            else if(byte == KART_VOICE_FRAME_HEAD0)
            {
                voice_state = VOICE_WAIT_HEAD1;
            }
            else
            {
                voice_state = VOICE_WAIT_HEAD0;
            }
            break;

        case VOICE_GET_CMD:
            voice_rx_cmd = byte;
            voice_state = VOICE_GET_PAD;
            break;

        case VOICE_GET_PAD:
            voice_state = VOICE_GET_CHECK;
            break;

        case VOICE_GET_CHECK:
        {
            uint8 check = (uint8)(0x6B + 0x79 + 0x00 + 0x81 + voice_rx_cmd + 0x00);
            if(byte == check)
            {
                voice_state = VOICE_GET_TAIL;
            }
            else
            {
                voice_state = VOICE_WAIT_HEAD0;
            }
            break;
        }

        case VOICE_GET_TAIL:
            if(byte == KART_VOICE_FRAME_TAIL)
            {
                voice_queue_push(voice_rx_cmd);
                voice_frame_count++;
                voice_last_cmd = voice_rx_cmd;
            }
            voice_state = VOICE_WAIT_HEAD0;
            break;

        default:
            voice_state = VOICE_WAIT_HEAD0;
            break;
    }
}

/* =========================== 对外接口 =========================== */
/* 2026-07-26 从 UART_1(P33.12/13)改到 BOARD_VOICE_UART_*(默认 UART_10,P13.0/13.1):
 * UART_1=ASCLIN1 与 TLD7002 灯板飞线同外设,共存即互相冲波特率。详见 board_pins.h。
 * 共用日志口时(BOARD_VOICE_SHARES_AUX_UART=1)本函数不碰波特率:
 * 该口已由 kart_debug_uart_init() 配成 460800 跑日志,启动阶段要保持 460800,
 * 真正切到 115200 的时机是进科目二 → kart_voice_uart_acquire()。
 * 不开 RX 中断:kart_voice_poll() 用 uart_query_byte() 直读硬件 RX FIFO
 * (zf_driver_uart.c 里走 IfxAsclin_getRxFifoFillLevel,不依赖 ISR),
 * 故 isr.c 的 uart10_rx_isr 保持为空即可,也不会和日志 TX 抢中断。 */
void kart_voice_init(void)
{
    /* 2026-08-12 VOICE_MUTED：4D7 人体视觉链路选了 VOFA 口(UART_10)，
     * 而语音模块插的是同一个坐子 —— 硬件上已经不在了。
     * 这里必须不碰 uart_init：它末尾是 uart_rx_interrupt(n, 0)，
     * 而本函数在 cpu0_main 里比 kart_person_link_init() 晚，
     * 一跑就把链路刚开的 RX 中断又关了 → 一个字节也收不到。 */
#if (!BOARD_VOICE_SHARES_AUX_UART && !VOICE_MUTED)
    /* 独占外设:启动时就配好 115200,一直挂着。 */
    uart_init(BOARD_VOICE_UART_INDEX, BOARD_VOICE_UART_BAUD,
              BOARD_VOICE_UART_TX_PIN, BOARD_VOICE_UART_RX_PIN);
#endif
    /* 共用日志口时这里刻意不配:该口已由 kart_debug_uart_init() 配成 460800 跑日志,
     * 启动阶段必须保持 460800(台上调试要看 VOFA)。真正切到 115200 的时机
     * 是进科目二 → kart_voice_uart_acquire()。 */
    voice_state = VOICE_WAIT_HEAD0;
    voice_queue_reset();
}

/* 科目二进入时调:把共用外设抢过来配成 115200,并清干净解析状态机与队列。
 * 共用日志口时调用方(kart_mission)必须先停日志,否则 JustFloat 字节
 * 会以 115200 喷向语音模块 RX。不共用时本函数只做状态复位。 */
void kart_voice_uart_acquire(void)
{
#if (BOARD_VOICE_SHARES_AUX_UART && !VOICE_MUTED)
    uart_init(BOARD_VOICE_UART_INDEX, BOARD_VOICE_UART_BAUD,
              BOARD_VOICE_UART_TX_PIN, BOARD_VOICE_UART_RX_PIN);
#endif
    voice_state = VOICE_WAIT_HEAD0;
    voice_queue_reset();
}

/* 科目二退出时调:把共用外设还给 VOFA 日志(460800)。
 * 不共用时是空操作,语音口继续以 115200 挂着无妨。 */
void kart_voice_uart_release(void)
{
#if (BOARD_VOICE_SHARES_AUX_UART && !VOICE_MUTED)
    uart_init(BOARD_AUX_UART_INDEX, BOARD_AUX_UART_BAUD_FAST,
              BOARD_AUX_UART_TX_PIN, BOARD_AUX_UART_RX_PIN);
#endif
    voice_state = VOICE_WAIT_HEAD0;
    voice_queue_reset();
}

void kart_voice_poll(void)
{
    uint8 byte;

#if VOICE_MUTED
    /* UART_10 已归 4D7 视觉链路：uart_query_byte() 会与链路的 RX 中断
     * 抢同一个 1 字节 FIFO，谁先取走另一方就永远收不到。
     * 在入口拦而不在 kart_mission 的三个调用处拦：入口拦一次不可能漏。
     * 命令队列不动，kart_voice_get_cmd() 自然一直返回 0。 */
    (void)byte;
    return;
#endif

    while(uart_query_byte(BOARD_VOICE_UART_INDEX, &byte))
    {
        voice_byte_count++;
        voice_feed_byte(byte);
    }
}

uint8 kart_voice_get_cmd(kart_voice_cmd_t *out)
{
    if(out == NULL)
    {
        return 0;
    }

    if(voice_q_head == voice_q_tail)
    {
        return 0;                       /* 队空 */
    }

    *out = voice_queue[voice_q_head];
    voice_q_head = (uint8)((voice_q_head + 1) % KART_VOICE_QUEUE_SIZE);
    return 1;
}

uint8 kart_voice_pending(void)
{
    return (uint8)((voice_q_tail - voice_q_head + KART_VOICE_QUEUE_SIZE)
                   % KART_VOICE_QUEUE_SIZE);
}

uint32 kart_voice_get_frame_count(void)
{
    return voice_frame_count;
}

uint8 kart_voice_get_last_cmd(void)
{
    return voice_last_cmd;
}

uint32 kart_voice_get_byte_count(void)
{
    return voice_byte_count;
}

/* =========================== 通用命令分发 =========================== */
void kart_voice_dispatch(void)
{
    kart_voice_cmd_t cmd;

    /* 鸣笛/运动/门洞复现都是长动作:任一忙时本拍不取新命令,让当前动作跑完再处理
     * 下一条,保证串行执行不打架(队列缓冲已在解析层入队)。
     * 【2026-07-29 补 kart_playback】原来漏判 kart_playback_is_running():门洞复现
     * 途中来一条语音命令会立刻被派发,kart_motion 和 kart_playback 同时写 target_delta /
     * target 速度,两个都在 5ms/10ms 拍上互相覆盖 → 车在门洞里乱打方向。
     * kart_playback 自己不看队列,所以只能在这里拦。 */
    if(kart_horn_is_busy() || kart_motion_is_busy() || kart_playback_is_running())
    {
        return;
    }

    if(!kart_voice_get_cmd(&cmd))
    {
        return;
    }

    if(cmd.cmd >= KART_VOICE_CMD_LEFT_LIGHT && cmd.cmd <= KART_VOICE_CMD_WIPER)
    {
        /* 灯光类 0x04~0x0B → 点阵屏图案(2026-07-26 接通)。
         * kart_light.c 里早已存在 7x15 图案/动画渲染器,之前全工程无人调用;
         * 命令码与枚举是连续对应的:
         *   0x04左转向 0x05右转向 0x06远光 0x07近光
         *   0x08雾灯 0x09双闪 0x0A车内照明 0x0B雨刷
         * 对应 KART_LIGHT_CMD_LEFT_TURN..WIPER(枚举值 1..8)。
         * 灯光是瞬时命令不占用 kart_horn/kart_motion 忙标志,动画推进在主循环 10ms 拍。 */
        kart_light_set_command((kart_light_command_t)
                               (KART_LIGHT_CMD_LEFT_TURN + (cmd.cmd - KART_VOICE_CMD_LEFT_LIGHT)));
    }
    else if(cmd.cmd >= 0x0C && cmd.cmd <= 0x14)
    {
        kart_horn_start(cmd.cmd - 0x0C + 1);
    }
    else if(cmd.cmd >= 0x15 && cmd.cmd <= 0x19)
    {
        /* 门洞前进 5 条 → gate kart_playback 槽位:与 Gate Playback 菜单同一映射。
         *   0x15 门洞一左侧→slot1  0x16 门洞一→slot2  0x17 门洞二→slot3
         *   0x18 门洞三→slot4      0x19 门洞三右侧→slot5
         * 复用已验证的 kart_playback 链路:加载→启动(相对当前位姿复现)。
         * 前提同手动选槽:说命令时车须已停在发车区标记点且车头摆正。
         *
         * 【2026-07-29 删掉了这里的 kart_odom_reset()】
         * 为什么删:它把发车区原点擦了,之后车永远不知道"发车区在哪",
         *   语音返回(0x1A~0x1E)就无从实现 —— 这是返回功能的头号阻塞项。
         * 为什么删了行为不变(可证明,不是赌):kart_playback_start() 自己把
         *   当前位姿快照存进 play_origin_x/y/yaw(kart_playback.c:204-207),
         *   poll 里只用 kart_odom - play_origin 的【差值】(345-346)。
         *   reset 只改绝对值不改差值 → 本条复现的每一拍输出完全一致。
         * 发车区原点改为整个科目二只清一次,在 mission_enter(MISSION_SUBJECT_2)。 */
        uint8 slot = (uint8)(cmd.cmd - 0x15 + 1);
        if(kart_record_load_from_flash(slot) >= 2)
        {
            kart_playback_start();      /* 内部再判点数<2 不启动 */
        }
    }
    else if(cmd.cmd >= 0x1A && cmd.cmd <= 0x1E)
    {
        /* 返回类 5 条 → 返回槽 6~10(2026-07-29 接通)。
         *   0x1A 门洞一右侧返回→slot6  0x1B 门洞一返回→slot7  0x1C 门洞二返回→slot8
         *   0x1D 门洞三返回→slot9      0x1E 门洞三左侧返回→slot10
         * 【与去程的对应】去程 0x15+j 与返回 0x1A+j 是同一物理通道的两个方向
         *   (去程"门洞一左侧" ↔ 返回"门洞一右侧":同一个洞,车头反过来了,
         *    左右自然互换),所以槽号偏移一致,j = cmd - 0x1A。
         * 【两步】车此刻停在任务区,位姿任意 → 不能直接复现:
         *   ① GOTO 把车摆到返回路径录制起点(集结点 S_j)附近,±0.5m/±10°;
         *   ② 加载槽 6+j,按【录制原点】而不是当前位姿启动复现 ——
         *      这样 Pure Pursuit 才看得见真实横向偏差,靠门洞前 ≥3m 直线引入段
         *      把它压掉(Ld=1.5m,3m 把 0.5m 压到 0.07m,门洞每侧余量 0.40m)。
         *      若按当前位姿启动,整条路径连门洞入口一起跟着车平移,前视再长也没用。
         * 两步的交接由 kart_mission 的科目二时序做(它每拍都跑,能看到 GOTO 何时
         * 结束);本模块只管"受理这条口令",不碰执行机构,与本文件的定位一致。 */
        /* Voice B:跳过 GOTO,按当前位姿复现(人已遥控摆好位)。 */
        if(kart_mission_subject2_get_manual_return())
            kart_mission_subject2_start_return_here((uint8)(cmd.cmd - 0x1A));
        else
            kart_mission_subject2_start_return((uint8)(cmd.cmd - 0x1A));
    }
    else if(cmd.cmd >= 0x1F && cmd.cmd <= 0x26)
    {
        kart_motion_start(cmd.cmd);
    }
}
