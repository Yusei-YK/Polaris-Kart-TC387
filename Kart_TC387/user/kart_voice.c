#include "kart_voice.h"
#include "kart_light.h"
#include "kart_horn.h"
#include "kart_motion.h"
#include "kart_record.h"
#include "kart_playback.h"
#include "kart_odom.h"

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
#if !BOARD_VOICE_SHARES_AUX_UART
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
#if BOARD_VOICE_SHARES_AUX_UART
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
#if BOARD_VOICE_SHARES_AUX_UART
    uart_init(BOARD_AUX_UART_INDEX, BOARD_AUX_UART_BAUD_FAST,
              BOARD_AUX_UART_TX_PIN, BOARD_AUX_UART_RX_PIN);
#endif
    voice_state = VOICE_WAIT_HEAD0;
    voice_queue_reset();
}

void kart_voice_poll(void)
{
    uint8 byte;

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

/* =========================== 临时命令分发 =========================== */
void kart_voice_dispatch(void)
{
    kart_voice_cmd_t cmd;

    /* 鸣笛/运动都是长动作:任一忙时本拍不取新命令,让当前动作跑完再处理下一条,
     * 保证串行执行不打架(队列缓冲已在解析层入队)。 */
    if(kart_horn_is_busy() || kart_motion_is_busy())
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
         * 灯光是瞬时命令不占用 horn/motion 忙标志,动画推进在主循环 10ms 拍。 */
        kart_light_set_command((kart_light_command_t)
                               (KART_LIGHT_CMD_LEFT_TURN + (cmd.cmd - KART_VOICE_CMD_LEFT_LIGHT)));
    }
    else if(cmd.cmd >= 0x0C && cmd.cmd <= 0x14)
    {
        kart_horn_start(cmd.cmd - 0x0C + 1);
    }
    else if(cmd.cmd >= 0x15 && cmd.cmd <= 0x19)
    {
        /* 门洞前进 5 条 → gate playback 槽位:与 Gate Playback 菜单同一映射。
         *   0x15 门洞一左侧→slot1  0x16 门洞一→slot2  0x17 门洞二→slot3
         *   0x18 门洞三→slot4      0x19 门洞三右侧→slot5
         * 复用已验证的 playback 链路:清 odom(当前位姿=复现原点)→加载→启动。
         * 前提同手动选槽:说命令时车须已停在该门洞入口且车头摆正。
         * 返回类 0x1A~0x1E 暂不实现(反向路径未录),落到下方 else 丢弃。
         * 依赖:Voice Control 已切 MISSION_SUBJECT_2,playback_poll 与
         *      subject2_loop(deadman/推进)才会运行。 */
        uint8 slot = (uint8)(cmd.cmd - 0x15 + 1);
        kart_odom_reset();
        if(kart_record_load_from_flash(slot) >= 2)
        {
            kart_playback_start();      /* 内部再判点数<2 不启动 */
        }
    }
    else if(cmd.cmd >= 0x1F && cmd.cmd <= 0x26)
    {
        kart_motion_start(cmd.cmd);
    }
}
