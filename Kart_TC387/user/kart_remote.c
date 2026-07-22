#include "kart_remote.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"

/*
 * SBUS 枪式遥控接收 —— 解析 + 失联兜底层实现(第一版:纯观测)
 * ------------------------------------------------------------------
 * 收字节链路: UART3 RX 中断 → kart_remote_rx_callback() 逐字节攒帧 →
 *             满 25 字节且帧头/帧尾对 → remote_analysis() 解 6 通道 →
 *             写共享结构(帧计数 + 通道 + 信号态)。
 * 失联链路:   主循环 kart_remote_poll() 每拍累加计时;中断收到合法帧时
 *             清零计时(idle_ms=0)。计时超阈值判失联。
 *
 * 并发:通道数组与信号态由中断写、主循环/VOFA 读。读取用 interrupt_global_disable
 *       成组快照,避免读到半更新的帧。idle_ms 由主循环写、中断清零,
 *       中断优先级高于主循环,清零是单次写,天然安全。
 * ------------------------------------------------------------------
 */

/* -------------------- 收帧缓冲(仅中断上下文访问) -------------------- */
static uint8  remote_rx_buf[KART_REMOTE_FRAME_LEN] = {0};
static vuint8 remote_rx_len = 0;

/* -------------------- 共享状态(中断写 / 主循环读) -------------------- */
static volatile uint16 remote_channel[KART_REMOTE_CHANNEL_NUM] = {0};
static volatile uint8  remote_signal_state = 0;    /* 接收机失控标志:1 正常 0 失控 */
static volatile uint32 remote_frame_count  = 0;    /* 合法帧累计 */

/* -------------------- 失联计时 -------------------- */
static volatile uint32 remote_idle_ms = KART_REMOTE_LOST_TIMEOUT_MS + 1; /* 上电即视为未在线 */
static uint8  remote_online = 0;                   /* poll 更新的综合在线态 */

/* 三段挡位:由 poll 每拍解码,保证全模式实时(供 VOFA ch33 显示 + playback deadman 判据)。
 * 前置声明解码函数(定义在下方接管层),因 poll 在其之前。 */
static kart_remote_sw3_t remote_sw3 = KART_REMOTE_SW3_L;
static kart_remote_sw3_t remote_decode_sw3(uint16 raw);

/* 挡位滤波:连续 N 次读到同一挡位才切换,防止信号抖动误触发急停 */
#define KART_REMOTE_SW3_FILTER_COUNT  (3)
static kart_remote_sw3_t remote_sw3_candidate = KART_REMOTE_SW3_L;
static uint8 remote_sw3_stable_count = 0;

/* 帧间隔判据:SBUS 正常约 14ms/帧,同一帧内字节间隔 <1ms。
 * 若两字节间隔过久(接收机断流/换帧),说明上一帧残缺,丢弃重攒。
 * 用逐飞封装的 system_getval_us() 算 us 级间隔(内部走 STM 计数器)。 */
static uint32 remote_byte_interval_us(void)
{
    static uint32 time_last = 0;
    uint32 time, interval;

    time = system_getval_us();
    interval = time - time_last;        /* uint32 回绕相减仍得正确间隔 */
    time_last = time;

    return interval;
}

/* -------------------- SBUS 6 通道解包 -------------------- */
/* 11bit/通道,小端打包在 buffer[1..9]。与逐飞例程 uart_receiver_analysis 一致。 */
static void remote_analysis(const uint8 *buffer)
{
    uint16 ch[KART_REMOTE_CHANNEL_NUM];

    ch[0] = (buffer[1]      | buffer[2] << 8)                  & 0x07FF;
    ch[1] = (buffer[2] >> 3 | buffer[3] << 5)                  & 0x07FF;
    ch[2] = (buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF;
    ch[3] = (buffer[5] >> 1 | buffer[6] << 7)                  & 0x07FF;
    ch[4] = (buffer[6] >> 4 | buffer[7] << 4)                  & 0x07FF;
    ch[5] = (buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9)  & 0x07FF;

    for(uint8 i = 0; i < KART_REMOTE_CHANNEL_NUM; i++)
    {
        remote_channel[i] = ch[i];
    }

    /* byte[23] & 0x04 置位 = 接收机失控 */
    remote_signal_state = (KART_REMOTE_LOST_BIT == (buffer[23] & KART_REMOTE_LOST_BIT)) ? 0 : 1;

    remote_frame_count++;
    remote_idle_ms = 0;                 /* 收到合法帧,失联计时清零 */
}

/* -------------------- RX 中断回调 -------------------- */
void kart_remote_rx_callback(void)
{
    /* 字节间隔过久 → 认定上一帧残缺,从头攒 */
    if(remote_byte_interval_us() > 3000)
    {
        remote_rx_len = 0;
    }

    remote_rx_buf[remote_rx_len++] = uart_read_byte(BOARD_GPS_UART_INDEX);

    if(remote_rx_len >= KART_REMOTE_FRAME_LEN)
    {
        if((KART_REMOTE_FRAME_HEAD == remote_rx_buf[0]) &&
           (KART_REMOTE_FRAME_END  == remote_rx_buf[24]))
        {
            remote_analysis(remote_rx_buf);
        }
        remote_rx_len = 0;              /* 无论校验成败都复位,准备下一帧 */
    }
}

/* -------------------- 初始化 -------------------- */
void kart_remote_init(void)
{
    /* UART3 走 SBUS:100000/8E2 + 电平反相,库内封装。
     * TX 脚(P15.6)仅占位,接收机不需要主板发送。 */
    uart_sbus_init(BOARD_GPS_UART_INDEX, KART_REMOTE_BAUD,
                   BOARD_GPS_UART_TX_PIN, BOARD_GPS_UART_RX_PIN);

    /* 开 UART3 RX 中断。回调直接挂在 isr.c 的 uart3_rx_isr,不走 set_wireless_type。 */
    uart_rx_interrupt(BOARD_GPS_UART_INDEX, 1);

    remote_rx_len       = 0;
    remote_frame_count  = 0;
    remote_signal_state = 0;
    remote_idle_ms      = KART_REMOTE_LOST_TIMEOUT_MS + 1;
    remote_online       = 0;
}

/* -------------------- 主循环轮询(只更新失联态,不控制) -------------------- */
void kart_remote_poll(uint16 period_ms)
{
    uint32 idle;

    /* 累加失联计时(封顶防溢出) */
    idle = remote_idle_ms;
    if(idle < 0xFFFF0000u)
    {
        idle += period_ms;
        remote_idle_ms = idle;
    }

    /* 综合在线:未超时 且 接收机未报失控 */
    remote_online = (idle <= KART_REMOTE_LOST_TIMEOUT_MS && remote_signal_state) ? 1 : 0;

    /* 每拍解码三段挡位:全模式实时(VOFA ch33 + playback deadman 判据都读它)。
     * 失联时钳到低挡 L(等同急停语义)。原来只在遥控模式解码,导致 IDLE 下 sw3
     * 冻结在上电默认 L,拨挡无变化且 b1 被 deadman 恒判急停。
     * 增加滤波:连续 N 次读到同一挡位才切换,防止信号抖动误触发急停。 */
    if(remote_online)
    {
        kart_remote_sw3_t sw3_raw = remote_decode_sw3(kart_remote_get_channel(KART_REMOTE_CH_SW3));

        if(sw3_raw == remote_sw3_candidate)
        {
            remote_sw3_stable_count++;
            if(remote_sw3_stable_count >= KART_REMOTE_SW3_FILTER_COUNT)
            {
                remote_sw3 = sw3_raw;
                remote_sw3_stable_count = KART_REMOTE_SW3_FILTER_COUNT;
            }
        }
        else
        {
            remote_sw3_candidate = sw3_raw;
            remote_sw3_stable_count = 1;
        }
    }
    else
    {
        remote_sw3 = KART_REMOTE_SW3_L;
        remote_sw3_candidate = KART_REMOTE_SW3_L;
        remote_sw3_stable_count = 0;
    }
}

/* -------------------- 观测接口 -------------------- */
uint16 kart_remote_get_channel(uint8 idx)
{
    uint16 val;
    uint32 istate;

    if(idx >= KART_REMOTE_CHANNEL_NUM)
    {
        return 0;
    }

    istate = interrupt_global_disable();
    val = remote_channel[idx];
    interrupt_global_enable(istate);

    return val;
}

uint8 kart_remote_is_online(void)
{
    return remote_online;
}

uint32 kart_remote_get_frame_count(void)
{
    return remote_frame_count;
}

uint8 kart_remote_get_signal_state(void)
{
    return remote_signal_state;
}

/* ================================================================== */
/* ===================== 接管控制层(有电机动作)===================== */
/* ================================================================== */
/*
 * 安全设计(严守 2026-07-06 转向打死烧驱动事故教训):
 *   ① 方向:摇杆线性映射后,结果【硬钳】到转向软限位 [R, L],
 *      即使标定漂移/摇杆超程,目标转角也绝不会超软限位,内环不会怼硬限位。
 *   ② 失联兜底:poll 判失联(超时或接收机失控)→ 本函数强制急停,
 *      不吃任何通道值。
 *   ③ 帧校验已在解析层做(帧头/帧尾),坏帧不更新通道,接管读到的恒为上一合法帧。
 *   ④ 三段低挡 = 软件急停;它是接管的总闸,低挡时车绝对不动。
 */

/* 三段开关原始值 → 挡位。取三个标定点的最近邻,抗抖动。 */
static kart_remote_sw3_t remote_decode_sw3(uint16 raw)
{
    int dl = (int)raw - KART_REMOTE_SW3_LOW;  if(dl < 0) dl = -dl;
    int dm = (int)raw - KART_REMOTE_SW3_MID;  if(dm < 0) dm = -dm;
    int dh = (int)raw - KART_REMOTE_SW3_HIGH; if(dh < 0) dh = -dh;

    if(dl <= dm && dl <= dh) return KART_REMOTE_SW3_L;
    if(dh <= dm && dh <= dl) return KART_REMOTE_SW3_H;
    return KART_REMOTE_SW3_M;
}

/* 方向摇杆 → 目标转角(编码器计数),带死区 + 硬钳软限位。
 * 中位→0;偏左(raw>center)→正 delta(左软限 +KART_STEER_DELTA_LIMIT_L);
 * 偏右(raw<center)→负 delta(右软限 KART_STEER_DELTA_LIMIT_R,本身为负)。 */
static float remote_map_steer(uint16 raw)
{
    int diff = (int)raw - KART_REMOTE_STEER_CENTER;
    float delta;

    if(diff > -KART_REMOTE_DEADZONE && diff < KART_REMOTE_DEADZONE)
    {
        return 0.0f;                        /* 死区内视为回中 */
    }

    if(diff > 0)
    {
        /* 偏左:[center+dz, left] 线性映射到 [0, DELTA_LIMIT_L] */
        int span = KART_REMOTE_STEER_LEFT - KART_REMOTE_STEER_CENTER;   /* 正 */
        if(span == 0) return 0.0f;
        delta = (float)(diff) / (float)span * (float)KART_STEER_DELTA_LIMIT_L;
    }
    else
    {
        /* 偏右:[right, center-dz] 线性映射到 [DELTA_LIMIT_R, 0] */
        int span = KART_REMOTE_STEER_CENTER - KART_REMOTE_STEER_RIGHT;  /* 正 */
        if(span == 0) return 0.0f;
        delta = (float)(-diff) / (float)span * (float)KART_STEER_DELTA_LIMIT_R;
    }

    delta *= (float)KART_REMOTE_STEER_SIGN;

    /* 硬钳软限位:双保险,映射再怎么算都出不了 [R, L]。 */
    if(delta > (float)KART_STEER_DELTA_LIMIT_L) delta = (float)KART_STEER_DELTA_LIMIT_L;
    if(delta < (float)KART_STEER_DELTA_LIMIT_R) delta = (float)KART_STEER_DELTA_LIMIT_R;

    return delta;
}

/* 油门扳机 → 目标速度(脉冲/5ms),带死区。
 * 中位 880=停;扳到底 579(<center)=前进满 → +MAX;前推 1180(>center)=倒车满 → −MAX。 */
static float remote_map_throttle(uint16 raw)
{
    int diff = (int)raw - KART_REMOTE_THR_CENTER;
    float spd;

    if(diff > -KART_REMOTE_DEADZONE && diff < KART_REMOTE_DEADZONE)
    {
        return 0.0f;
    }

    if(diff < 0)
    {
        /* 前进:[fwd, center-dz] → [+MAX, 0] */
        int span = KART_REMOTE_THR_CENTER - KART_REMOTE_THR_FWD;        /* 正 */
        if(span == 0) return 0.0f;
        spd = (float)(-diff) / (float)span * KART_REMOTE_MAX_SPEED;     /* 正=前进 */
    }
    else
    {
        /* 倒车:[center+dz, rev] → [0, −MAX] */
        int span = KART_REMOTE_THR_REV - KART_REMOTE_THR_CENTER;        /* 正 */
        if(span == 0) return 0.0f;
        spd = -(float)(diff) / (float)span * KART_REMOTE_MAX_SPEED;     /* 负=倒车 */
    }

    /* 幅值封顶(摇杆超标定端点时) */
    if(spd >  KART_REMOTE_MAX_SPEED) spd =  KART_REMOTE_MAX_SPEED;
    if(spd < -KART_REMOTE_MAX_SPEED) spd = -KART_REMOTE_MAX_SPEED;

    return spd;
}

/* 强制急停:速度清零关使能 + 转向回中(保持内环使能把方向盘按在中位)。 */
static void remote_emergency_stop(void)
{
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);
    kart_steer_set_target_delta(0.0f);      /* 方向盘回中 */
    /* 转向内环是否使能由挡位决定;急停不主动关内环,让方向盘被按在中位更安全。
     * 若要彻底断转向电机,由 mission 切出模式时的 stop_all 统一关。 */
}

void kart_remote_control_update(void)
{
    kart_remote_sw3_t sw3;
    uint16 steer_raw, thr_raw;
    float  target_delta, target_speed;

    /* 失联(超时或接收机失控)→ 无条件急停,不读任何通道。 */
    if(!kart_remote_is_online())
    {
        kart_steer_set_angle_enable(0);     /* 失联彻底断转向电机 */
        remote_emergency_stop();
        return;
    }

    steer_raw = kart_remote_get_channel(KART_REMOTE_CH_STEER);
    thr_raw   = kart_remote_get_channel(KART_REMOTE_CH_THROTTLE);

    /* sw3 由 poll 每拍解码,这里直接读全局变量(已是最新挡位) */
    sw3 = remote_sw3;

    switch(sw3)
    {
        case KART_REMOTE_SW3_L:
            /* 低挡 = 全关急停:速度关、转向电机断。 */
            kart_steer_set_angle_enable(0);
            remote_emergency_stop();
            break;

        case KART_REMOTE_SW3_M:
            /* 中挡 = 只转向:内环使能随摇杆,速度关。 */
            target_delta = remote_map_steer(steer_raw);
            kart_steer_set_angle_enable(1);
            kart_steer_set_head_enable(0);          /* 遥控不用航向外环 */
            kart_steer_set_target_delta(target_delta);
            kart_control_set_target(0.0f);
            kart_control_set_enable(0);
            break;

        case KART_REMOTE_SW3_H:
            /* 高挡 = 转向 + 速度:方向随摇杆,速度随油门。 */
            target_delta = remote_map_steer(steer_raw);
            target_speed = remote_map_throttle(thr_raw);
            kart_steer_set_angle_enable(1);
            kart_steer_set_head_enable(0);
            kart_steer_set_target_delta(target_delta);
            kart_control_set_enable(1);
            kart_control_set_target(target_speed);
            break;

        default:
            remote_emergency_stop();
            break;
    }
}

void kart_remote_control_stop(void)
{
    remote_sw3 = KART_REMOTE_SW3_L;
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);
    kart_steer_set_target_delta(0.0f);
    kart_steer_set_angle_enable(0);
    kart_steer_set_head_enable(0);
}

kart_remote_sw3_t kart_remote_get_sw3(void)
{
    return remote_sw3;
}
