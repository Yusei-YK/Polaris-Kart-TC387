#include "kart_debug_uart.h"
#include "kart_control.h"
#include "kart_imu.h"
#include "kart_steer_abs.h"
#include <string.h>
#include <stdlib.h>

/*
 * 调参台实现 —— VOFA+ justfloat 波形 + 在线改参
 * ------------------------------------------------------------------
 * 数据源全部只读:速度环状态从 kart_control_get_xxx() 取,航向从 kart_imu_get_yaw() 取。
 * 关键:这里绝不再调 kart_encoder_update() —— 编码器 delta 的清零权已经独家交给
 *      5ms 中断里的 kart_control_speed_update()。若这里再调,会和中断互抢 delta,
 *      速度测量直接乱套。这是上一版 CSV 的隐患,这版彻底改掉。
 * ------------------------------------------------------------------
 */

/* VOFA justfloat 通道定义(改通道要同步改上位机)。当前 7 通道: */
#define KART_VOFA_CH_NUM    (7)
/*  ch0 = 目标速度(脉冲/5ms)
 *  ch1 = 实测速度(滤波后,脉冲/5ms)
 *  ch2 = 输出 duty
 *  ch3 = 航向 yaw(度)
 *  ch4 = RX 累计收到字节数(诊断用:上位机发命令时看这条涨不涨,
 *        涨=字节进了 MCU,问题在解析;不涨=字节压根没进 FIFO,问题在链路/波特率/接线)
 *  ch5 = 转向绝对编码器 raw(0~4095,掰方向盘看是否平滑变化,验证飞线后编码器好坏)
 *  ch6 = 转向相对中位偏差 center_delta(带 ±2048 环绕,后续角度环的误差反馈源)*/

static uint32 kart_debug_elapsed_ms = 0;

/* RX 累计字节数:每收到一个字节就 +1,永不清零。作诊断波形用。 */
static uint32 kart_debug_rx_count = 0;

/* 命令接收缓冲:攒字符直到遇到结束符再解析 */
static char  kart_cmd_buf[KART_DEBUG_CMD_BUF_LEN] = {0};
static uint8 kart_cmd_len = 0;

/* =========================== 初始化 =========================== */
void kart_debug_uart_init(void)
{
    uart_init(BOARD_WIRELESS_UART_INDEX, BOARD_WIRELESS_UART_BAUD,
              BOARD_WIRELESS_UART_TX_PIN, BOARD_WIRELESS_UART_RX_PIN);
}

/* =========================== VOFA justfloat 下发 =========================== */
/* 组一帧:N 个小端 float + 帧尾 0x00,0x00,0x80,0x7F,一次性 uart_write_buffer 发出。
 * TC387 是小端,float 内存布局和 justfloat 要求一致,直接 memcpy 即可。 */
static void kart_debug_send_vofa(void)
{
    uint8  frame[KART_VOFA_CH_NUM * 4 + 4];
    float  ch[KART_VOFA_CH_NUM];
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};

    ch[0] = kart_control_get_target();
    ch[1] = kart_control_get_meas();
    ch[2] = (float)kart_control_get_output();
    ch[3] = kart_imu_get_yaw();
    ch[4] = (float)kart_debug_rx_count;     // RX 诊断:累计收到的字节数
    ch[5] = (float)kart_steer_abs_get_raw();            // 转向原始角(0~4095)
    ch[6] = (float)kart_steer_abs_get_center_delta();   // 转向相对中位偏差(角度环误差源)

    memcpy(&frame[0], ch, KART_VOFA_CH_NUM * 4);
    memcpy(&frame[KART_VOFA_CH_NUM * 4], tail, 4);

    uart_write_buffer(BOARD_WIRELESS_UART_INDEX, frame, sizeof(frame));
}

/* =========================== 命令解析 =========================== */
/* 一条完整命令(已去掉结束符)交给这里。首字符是指令,其余是数值。 */
static void kart_debug_parse_cmd(const char *cmd, uint8 len)
{
    float val;

    if(len < 1)
    {
        return;                         // 空命令,忽略
    }

    val = (float)atof(&cmd[1]);         // 从第 2 个字符起转数值(没有数值时 atof 返回 0)

    switch(cmd[0])
    {
        case 'p': case 'P':
            kart_control_set_pid(val,
                                 kart_speed.pid.Ki,
                                 kart_speed.pid.Kd);
            break;

        case 'i': case 'I':
            kart_control_set_pid(kart_speed.pid.Kp,
                                 val,
                                 kart_speed.pid.Kd);
            break;

        case 'd': case 'D':
            kart_control_set_pid(kart_speed.pid.Kp,
                                 kart_speed.pid.Ki,
                                 val);
            break;

        case 't': case 'T':
            kart_control_set_target(val);
            break;

        case 'e': case 'E':
            kart_control_set_enable((val != 0.0f) ? 1 : 0);
            break;

        default:
            break;                      // 不认识的指令,丢弃
    }
}

/* 把串口收到的字节攒进缓冲,遇到 \r / \n / 空格 就当一条命令解析。 */
static void kart_debug_recv_cmd(void)
{
    uint8 byte;

    /* 一次 poll 把 FIFO 里攒着的字节都收完,别留到下一拍 */
    while(uart_query_byte(BOARD_WIRELESS_UART_INDEX, &byte))
    {
        kart_debug_rx_count++;              // 诊断:每进来一个字节就 +1

        /* 结束符:回车/换行/空格都算(兑现头文件"兼容空格结尾"的承诺)。
         * 很多串口助手/VOFA 默认不追加换行,只靠 \r\n 会导致命令永远攒着不解析。 */
        if(byte == '\r' || byte == '\n' || byte == ' ')
        {
            if(kart_cmd_len > 0)
            {
                kart_cmd_buf[kart_cmd_len] = '\0';
                kart_debug_parse_cmd(kart_cmd_buf, kart_cmd_len);
                kart_cmd_len = 0;               // 收完清空,准备下一条
            }
        }
        else if(kart_cmd_len < (KART_DEBUG_CMD_BUF_LEN - 1))
        {
            kart_cmd_buf[kart_cmd_len++] = (char)byte;
        }
        else
        {
            kart_cmd_len = 0;                   // 溢出(没等到结束符),丢弃重来
        }
    }
}

/* =========================== 主循环调用 =========================== */
void kart_debug_uart_poll(void)
{
    /* 命令接收每拍都收,保证按键响应及时 */
    kart_debug_recv_cmd();

    /* VOFA 波形按 KART_DEBUG_UART_PERIOD_MS 节流下发 */
    kart_debug_elapsed_ms += KART_MAIN_LOOP_PERIOD_MS;
    if(kart_debug_elapsed_ms < KART_DEBUG_UART_PERIOD_MS)
    {
        return;
    }
    kart_debug_elapsed_ms = 0;

    kart_debug_send_vofa();
}
