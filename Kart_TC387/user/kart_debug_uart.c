#include "kart_debug_uart.h"
#include "kart_control.h"
#include "kart_encoder.h"
#include "kart_imu.h"
#include "kart_odom.h"
#include "kart_steer_abs.h"
#include "kart_steer_ctrl.h"
#include "kart_voice.h"
#include "kart_record.h"
#include "kart_playback.h"
#include "kart_mission.h"
#include "kart_remote.h"        /* SBUS 遥控观测(纯只读上 VOFA) */
#include "kart_hw_test.h"       /* 借用旋钮/按键引脚宏(屏坏后改用 VOFA 看按键输入) */
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

/* VOFA justfloat 通道定义(改通道要同步改上位机)。当前 34 通道: */
#define KART_VOFA_CH_NUM    (34)
/*  ch0  = 目标速度(脉冲/5ms)
 *  ch1  = 实测均值(左右滤波速度均值,脉冲/5ms)
 *  ch2  = 航向 yaw(度)
 *  ch3  = 转向中位偏差 center_delta(角度环误差反馈源)
 *  ch4  = 转向内环目标转角 target_delta(编码器计数;外环开时被航向环覆写)
 *  ch5  = 转向内环输出 duty
 *  ch6  = 外环目标航向 target_yaw(度)
 *  ch7  = 左后轮滤波速度(脉冲/5ms)
 *  ch8  = 右后轮滤波速度(脉冲/5ms)
 *  ch9  = 左后轮 PI 输出 duty
 *  ch10 = 右后轮 PI 输出 duty
 *  ch11 = 航位推算累计路程(m)
 *  ch12 = 航位推算 X 坐标(m)
 *  ch13 = 航位推算 Y 坐标(m)
 *  ch14 = 左后轮累计脉冲
 *  ch15 = 右后轮累计脉冲
 *  ch16 = 语音接收帧计数(校验通过的合法帧)
 *  ch17 = 语音最后一帧 TYPE
 *  ch18 = 语音最后一帧 CMD
 *  ch19 = 录制点数
 *  --- 以下为屏坏后新增,VOFA 直接看旋钮/按键输入(上拉输入:不按=1,按下=0)---
 *  ch20 = 旋钮 A 相电平(P11.2)
 *  ch21 = 旋钮 B 相电平(P11.3)
 *  ch22 = 旋钮 按下键 SW(P20.6)
 *  ch23 = 五向键 UP(P33.11)
 *  ch24 = 五向键 DOWN(P20.0)
 *  ch25 = 五向键 LEFT(P21.6)
 *  ch26 = 五向键 RIGHT(P21.7)
 *  ch27 = 五向键 MID(P33.4)
 *  --- 以下为 SBUS 遥控观测(第一版纯观测,不接管控制)---
 *  ch28 = 遥控在线态(1=在线/0=失联,综合超时+接收机失控标志)
 *  ch29 = SBUS 合法帧累计计数(看收帧是否稳定增长)
 *  ch30 = CH1 方向通道原始值(0~2047,中值约 1024)
 *  ch31 = CH2 油门通道原始值(0~2047,中值约 1024)
 *  ch32 = CH4 三段开关原始值(0~2047)
 *  ch33 = 遥控三段挡位解码(0=低急停/1=中只转向/2=高转向+速度) */

static uint32 kart_debug_elapsed_ms = 0;

/* 命令接收缓冲:攒字符直到遇到结束符再解析 */
static char  kart_cmd_buf[KART_DEBUG_CMD_BUF_LEN] = {0};
static uint8 kart_cmd_len = 0;

/* =========================== 初始化 =========================== */
void kart_debug_uart_init(void)
{
    uart_init(BOARD_WIRELESS_UART_INDEX, BOARD_WIRELESS_UART_BAUD,
              BOARD_WIRELESS_UART_TX_PIN, BOARD_WIRELESS_UART_RX_PIN);

    /* 屏坏后改用 VOFA 观测旋钮/按键:统一初始化为上拉输入(按下接地读 0)。 */
    gpio_init(KART_KNOB_A_PIN,    GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KNOB_B_PIN,    GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KNOB_SW_PIN,   GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KEY_UP_PIN,    GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KEY_DOWN_PIN,  GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KEY_LEFT_PIN,  GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KEY_RIGHT_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(KART_KEY_MID_PIN,   GPI, 0, GPI_PULL_UP);
}

/* =========================== VOFA justfloat 下发 =========================== */
/* 组一帧:N 个小端 float + 帧尾 0x00,0x00,0x80,0x7F,一次性 uart_write_buffer 发出。
 * TC387 是小端,float 内存布局和 justfloat 要求一致,直接 memcpy 即可。 */
static void kart_debug_send_vofa(void)
{
    uint8  frame[KART_VOFA_CH_NUM * 4 + 4];
    float  ch[KART_VOFA_CH_NUM];
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};

    ch[0]  = kart_control_get_target();
    ch[1]  = kart_control_get_meas();
    ch[2]  = kart_imu_get_yaw();
    ch[3]  = (float)kart_steer_abs_get_center_delta();
    ch[4]  = kart_steer_get_target_delta();
    ch[5]  = (float)kart_steer_get_output();
    ch[6]  = kart_steer_get_target_yaw();
    ch[7]  = kart_control_get_left_meas();
    ch[8]  = kart_control_get_right_meas();
    ch[9]  = (float)kart_control_get_left_output();
    ch[10] = (float)kart_control_get_right_output();
    ch[11] = kart_odom_get_dist();
    ch[12] = kart_odom_get_x();
    ch[13] = kart_odom_get_y();
    ch[14] = (float)kart_encoder_get_left_sum();
    ch[15] = (float)kart_encoder_get_right_sum();
    ch[16] = (float)kart_voice_get_frame_count();
    ch[17] = (float)kart_voice_get_last_type();
    ch[18] = (float)kart_voice_get_last_cmd();
    ch[19] = (float)kart_record_get_count();

    /* 旋钮 + 五向键实时电平(上拉:不按=1,按下=0)。屏坏后靠这几路看按键输入。 */
    ch[20] = (float)gpio_get_level(KART_KNOB_A_PIN);
    ch[21] = (float)gpio_get_level(KART_KNOB_B_PIN);
    ch[22] = (float)gpio_get_level(KART_KNOB_SW_PIN);
    ch[23] = (float)gpio_get_level(KART_KEY_UP_PIN);
    ch[24] = (float)gpio_get_level(KART_KEY_DOWN_PIN);
    ch[25] = (float)gpio_get_level(KART_KEY_LEFT_PIN);
    ch[26] = (float)gpio_get_level(KART_KEY_RIGHT_PIN);
    ch[27] = (float)gpio_get_level(KART_KEY_MID_PIN);

    /* SBUS 遥控观测(只读,不接管控制)。 */
    ch[28] = (float)kart_remote_is_online();
    ch[29] = (float)kart_remote_get_frame_count();
    ch[30] = (float)kart_remote_get_channel(KART_REMOTE_CH_STEER);
    ch[31] = (float)kart_remote_get_channel(KART_REMOTE_CH_THROTTLE);
    ch[32] = (float)kart_remote_get_channel(KART_REMOTE_CH_SW3);
    ch[33] = (float)kart_remote_get_sw3();

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

    /* 转向串级用两字符前缀(s=内环转角 / h=外环航向),避免和速度环单字符命令撞车。
     * 数值从第 3 个字符起取。放在单字符 switch 之前拦截。 */
    if((cmd[0] == 's' || cmd[0] == 'S') && len >= 2)
    {
        float sval = (float)atof(&cmd[2]);
        switch(cmd[1])
        {
            case 'e': case 'E':                                     // se<0/1> 内环使能
                kart_steer_set_angle_enable((sval != 0.0f) ? 1 : 0);
                break;
            case 'a': case 'A':                                     // sa<val> 直给内环目标转角(外环关时单测用)
                kart_steer_set_target_delta(sval);
                break;
            case 'p': case 'P':                                     // sp<val> 内环 Kp
                kart_steer_set_angle_pid(sval, kart_steer.angle_pid.Ki, kart_steer.angle_pid.Kd);
                break;
            case 'i': case 'I':                                     // si<val> 内环 Ki
                kart_steer_set_angle_pid(kart_steer.angle_pid.Kp, sval, kart_steer.angle_pid.Kd);
                break;
            case 'd': case 'D':                                     // sd<val> 内环 Kd
                kart_steer_set_angle_pid(kart_steer.angle_pid.Kp, kart_steer.angle_pid.Ki, sval);
                break;
            default:
                break;
        }
        return;
    }

    if((cmd[0] == 'h' || cmd[0] == 'H') && len >= 2)
    {
        float hval = (float)atof(&cmd[2]);
        switch(cmd[1])
        {
            case 'e': case 'E':                                     // he<0/1> 外环使能(连带开内环)
                kart_steer_set_head_enable((hval != 0.0f) ? 1 : 0);
                break;
            case 't': case 'T':                                     // ht<val> 外环目标航向(度)
                kart_steer_set_target_yaw(hval);
                break;
            case 'p': case 'P':                                     // hp<val> 外环 Kp
                kart_steer_set_head_pid(hval, kart_steer.head_pid.Ki, kart_steer.head_pid.Kd);
                break;
            case 'i': case 'I':                                     // hi<val> 外环 Ki
                kart_steer_set_head_pid(kart_steer.head_pid.Kp, hval, kart_steer.head_pid.Kd);
                break;
            case 'd': case 'D':                                     // hd<val> 外环 Kd
                kart_steer_set_head_pid(kart_steer.head_pid.Kp, kart_steer.head_pid.Ki, hval);
                break;
            default:
                break;
        }
        return;
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

        case 'z': case 'Z':
            if(!kart_control_is_enabled())
            {
                uint32 interrupt_state = interrupt_global_disable();
                kart_encoder_reset();
                kart_odom_reset();
                interrupt_global_enable(interrupt_state);
            }
            break;

        case 'r': case 'R':
            if(val != 0.0f)
                kart_record_start();
            else
                kart_record_stop();
            break;

        case 'b': case 'B':
            if(val != 0.0f)
                kart_playback_start();
            else
                kart_playback_stop();
            break;

        case 'w': case 'W':             // w<slot> 把当前 RAM 路径存进 Flash 槽位(阻塞擦写,仅停车静止时用)
            if(!kart_control_is_enabled())
                kart_record_save_to_flash((uint8)(int)val);
            break;

        case 'f': case 'F':             // f<slot> 从 Flash 槽位读回路径到 RAM(看 ch19 点数验证)
            if(!kart_control_is_enabled())
                kart_record_load_from_flash((uint8)(int)val);
            break;

        case 'm': case 'M':             // m0=IDLE m1=科目一 m2=科目二 m3=遥控(切模式统一走停机)
            switch((int)val)
            {
                case 1:  kart_mission_set_mode(MISSION_SUBJECT_1); break;
                case 2:  kart_mission_set_mode(MISSION_SUBJECT_2); break;
                case 3:  kart_mission_set_mode(MISSION_REMOTE);    break;
                default: kart_mission_set_mode(MISSION_IDLE);      break;
            }
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
    /* VOFA 已停用:改用 IPS200 屏幕调试,省算力。命令接收+波形下发整体注释,需要时恢复。
    // 命令接收每拍都收,保证按键响应及时
    kart_debug_recv_cmd();

    // VOFA 波形按 KART_DEBUG_UART_PERIOD_MS 节流下发
    kart_debug_elapsed_ms += KART_MAIN_LOOP_PERIOD_MS;
    if(kart_debug_elapsed_ms < KART_DEBUG_UART_PERIOD_MS)
    {
        return;
    }
    kart_debug_elapsed_ms = 0;

    kart_debug_send_vofa();
    */
}
