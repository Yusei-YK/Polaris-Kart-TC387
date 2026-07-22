#include "kart_voice.h"
#include "kart_horn.h"

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
void kart_voice_init(void)
{
    uart_init(BOARD_WIRELESS_UART_INDEX, BOARD_WIRELESS_UART_BAUD,
              BOARD_WIRELESS_UART_TX_PIN, BOARD_WIRELESS_UART_RX_PIN);

    voice_state = VOICE_WAIT_HEAD0;
    voice_queue_reset();
}

void kart_voice_poll(void)
{
    uint8 byte;

    while(uart_query_byte(BOARD_WIRELESS_UART_INDEX, &byte))
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

    /* 鸣笛是长动作:正在鸣时本拍不取新命令,让当前节拍跑完再处理下一条。
     * 门洞/运动接入后,同理各自忙时挂起,保证串行执行不打架。 */
    if(kart_horn_is_busy())
    {
        return;
    }

    if(!kart_voice_get_cmd(&cmd))
    {
        return;
    }

    if(cmd.cmd >= 0x0C && cmd.cmd <= 0x14)
    {
        kart_horn_start(cmd.cmd - 0x0C + 1);
    }
}
