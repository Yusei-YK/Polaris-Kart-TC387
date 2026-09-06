#include "kart_pedal.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"
#include "kart_mission.h"
#include "kart_remote.h"
#include "kart_calib.h"

#if PEDAL_ENABLE

/* -------------------- 收字节：环形缓冲 --------------------
 * ISR 只塞字节，解析全在 10ms 拍做。理由同 kart_person_link：
 * 在中断里跑状态机会把解析耗时压进 UART 中断优先级，且这条链路 100Hz
 * 每帧 8 字节，缓冲 128 字节够存 16 帧，10ms 拍绝不会来不及。 */
#define PEDAL_RX_RING_SIZE          (128U)
#define PEDAL_RX_RING_MASK          (PEDAL_RX_RING_SIZE - 1U)
#define PEDAL_PARSE_BUDGET          (64U)   /* 单拍最多解析这么多字节，防长时间占用 */

static uint8           pedal_rx_ring[PEDAL_RX_RING_SIZE] = {0};
static volatile uint16 pedal_rx_head = 0;
static uint16          pedal_rx_tail = 0;

/* 逐字节攒帧 */
static uint8  pedal_rx_buf[PEDAL_FRAME_LEN] = {0};
static uint8  pedal_rx_len = 0;

/* -------------------- 链路与踏板状态 -------------------- */
static volatile uint32 pedal_idle_ms = KART_PEDAL_LOST_TIMEOUT_MS + 1U; /* 上电即视为未在线 */
static uint8  pedal_online   = 0;
static uint16 pedal_thr_pm   = 0;
static uint8  pedal_brake    = 1;   /* 上电按"踩下"算,偏保守:此时目标必然是 0 */
static uint8  pedal_flags    = 0;
static uint8  pedal_seq_last = 0;
static uint8  pedal_seq_seen = 0;

static float  pedal_target_ms = 0.0f;

static kart_pedal_stat_t pedal_stat = {0};

/* -------------------- 小工具 -------------------- */
static uint8 pedal_check_xor(const uint8 *p)
{
    uint8 i;
    uint8 x = 0;

    for(i = 0; PEDAL_CHECK_COVER_LEN > i; i++)
    {
        x ^= p[i];
    }
    return x;
}

/* 千分比 → 目标车速(m/s)。死区以下一律 0,死区以上线性铺到满量程。
 * 死区不是"减去死区再线性"那种保留满行程的做法:踏板 ADC 窗口只有 550 个码
 * (CH32 侧 THROTTLE_MIN/MAX = 250/800),底部噪声直接变油门抖动,宁可牺牲
 * 一点行程换一个干净的零点。 */
static float pedal_thr_to_ms(uint16 pm)
{
    float span;

    if(pm <= KART_PEDAL_DEADBAND_PM)
    {
        return 0.0f;
    }
    if(pm >= PEDAL_THROTTLE_PM_MAX)
    {
        return KART_PEDAL_MAX_V_MS;
    }

    span = (float)(PEDAL_THROTTLE_PM_MAX - KART_PEDAL_DEADBAND_PM);
    return (float)(pm - KART_PEDAL_DEADBAND_PM) / span * KART_PEDAL_MAX_V_MS;
}

/* -------------------- 解帧 --------------------
 * 一帧收满才判定,任何一处不对【整帧丢弃】,并且【不重置失联计时器】——
 * 这样一根被干扰的线会自然走到超时兜底(断油),而不是把乱码油门喂进速度环。 */
static void pedal_feed_byte(uint8 byte)
{
    pedal_stat.rx_bytes++;

    /* 帧头逐字节对齐:头两个字节错就地重同步,别攒满 8 个再丢 */
    if(0U == pedal_rx_len)
    {
        if(PEDAL_HEAD0 != byte) { return; }
    }
    else if(1U == pedal_rx_len)
    {
        if(PEDAL_HEAD1 != byte)
        {
            /* 这个字节本身可能是新帧的头0 */
            pedal_rx_len = (PEDAL_HEAD0 == byte) ? 1U : 0U;
            pedal_rx_buf[0] = PEDAL_HEAD0;
            return;
        }
    }

    pedal_rx_buf[pedal_rx_len] = byte;
    pedal_rx_len++;

    if(PEDAL_FRAME_LEN > pedal_rx_len)
    {
        return;
    }
    pedal_rx_len = 0;

    if((PEDAL_TAIL != pedal_rx_buf[7])
       || (pedal_check_xor(pedal_rx_buf) != pedal_rx_buf[6]))
    {
        pedal_stat.frame_bad++;
        return;
    }

    /* --- 到这儿才算一帧合法数据 --- */
    pedal_stat.frame_ok++;

    /* seq 断层 = 丢帧。没有它分不清"踏板没动"和"线断了"。 */
    if(pedal_seq_seen)
    {
        uint8 gap = (uint8)(pedal_rx_buf[2] - pedal_seq_last - 1U);
        if(0U != gap)
        {
            pedal_stat.seq_lost += gap;
        }
    }
    pedal_seq_last = pedal_rx_buf[2];
    pedal_seq_seen = 1;

    pedal_thr_pm = (uint16)(((uint16)pedal_rx_buf[4] << 8) | pedal_rx_buf[3]);
    if(pedal_thr_pm > PEDAL_THROTTLE_PM_MAX)
    {
        /* 踏板盒不该发超量程值;真发了就当满油处理而不是当垃圾丢,
         * 否则一个越界值会让这一帧静默失效、油门瞬间掉回上一帧。 */
        pedal_thr_pm = PEDAL_THROTTLE_PM_MAX;
    }
    pedal_flags = pedal_rx_buf[5];
    pedal_brake = (0U != (pedal_flags & PEDAL_FLAG_BRAKE)) ? 1U : 0U;

    pedal_stat.last_thr_pm = pedal_thr_pm;
    pedal_stat.last_flags  = pedal_flags;

    pedal_idle_ms = 0;      /* 只有合法帧才刷新失联计时 */
}

/* ==================== 初始化 / 中断 ==================== */
void kart_pedal_init(void)
{
    pedal_rx_head  = 0;
    pedal_rx_tail  = 0;
    pedal_rx_len   = 0;
    pedal_seq_seen = 0;
    pedal_online   = 0;
    pedal_thr_pm   = 0;
    pedal_brake    = 1;
    pedal_flags    = 0;
    pedal_target_ms = 0.0f;
    pedal_idle_ms  = KART_PEDAL_LOST_TIMEOUT_MS + 1U;

    uart_init(BOARD_PEDAL_UART_INDEX, PEDAL_BAUD,
              BOARD_PEDAL_UART_TX_PIN, BOARD_PEDAL_UART_RX_PIN);
    uart_rx_interrupt(BOARD_PEDAL_UART_INDEX, 1);
}

void kart_pedal_rx_callback(void)
{
    uint8  byte = uart_read_byte(BOARD_PEDAL_UART_INDEX);
    uint16 head = pedal_rx_head;
    uint16 next = (uint16)((head + 1U) & PEDAL_RX_RING_MASK);

    if(next != pedal_rx_tail)       /* 满了就丢新字节,不覆盖旧的 */
    {
        pedal_rx_ring[head] = byte;
        pedal_rx_head = next;
    }
}

/* ==================== 查询 ==================== */
uint8 kart_pedal_is_online(void)  { return pedal_online; }
uint16 kart_pedal_get_throttle_pm(void) { return pedal_thr_pm; }
uint8 kart_pedal_get_brake(void)  { return pedal_brake; }
float kart_pedal_get_target_ms(void) { return pedal_target_ms; }

void kart_pedal_get_stat(kart_pedal_stat_t *dst)
{
    if(NULL != dst)
    {
        pedal_stat.online = pedal_online;
        *dst = pedal_stat;
    }
}

void kart_pedal_note_deny(void)
{
    pedal_stat.engage_deny++;
}

/* 合闸两个条件。两条都不要求人做动作 —— 脚不放在油门上、线接好了就自然满足。
 * 【油门必须在死区内】这一条是防冲车的关键:油门线接触不良卡在半开时,
 * 少了它一按菜单键车就直接窜出去。
 * 【刹车不在条件里】按用户要求,KART_PEDAL_REQUIRE_BRAKE 默认 0 ——
 * 要求人按菜单键的同时踩着刹车是多一个动作,而真正防冲车的是上面那条。 */
uint8 kart_pedal_can_engage(void)
{
    if(0U == pedal_online)                      { return 0; }
#if KART_PEDAL_REQUIRE_BRAKE
    if(0U == pedal_brake)                       { return 0; }
#endif
    if(pedal_thr_pm > KART_PEDAL_DEADBAND_PM)   { return 0; }
    return 1;
}

/* ==================== 进出钩子 ====================
 * 由 kart_mission_set_mode() 调,不要在别处调 —— 模式和这两个钩子必须一一对应。 */
void kart_pedal_enter(void)
{
    /* 顺序有讲究:先把目标清零、再使能速度环。反过来先使能再清零的话,
     * 中间那一瞬速度环会拿着上一个模式留下的目标往外输出。 */
    kart_control_set_ramp_step(KART_PEDAL_RAMP_STEP);
    kart_control_set_target(0.0f);
    kart_control_set_enable(1);

    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);     /* 人开车,转向电机全程不通电 */

    pedal_target_ms = 0.0f;
}

void kart_pedal_exit(void)
{
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);
    kart_steer_set_angle_enable(0);
    pedal_target_ms = 0.0f;
}

/* ==================== 10ms 拍 ==================== */

/* 遥控抢占:遥控在线 且 油门杆离中位超过裕量 → 交回 MISSION_REMOTE。
 * 用油门杆而不是方向杆,因为人接管时第一反应是收油;用"离中位"而不是
 * "在线"本身,否则发射机一开机就永远进不了踏板模式。
 * 遥控永远优先 —— 它是唯一的人工接管安全绳,这条不能破。 */
static uint8 pedal_rc_wants_takeover(void)
{
    uint16 thr;
    int    diff;

    if(0U == kart_remote_is_online())
    {
        return 0;
    }
    thr  = kart_remote_get_channel(KART_REMOTE_CH_THROTTLE);
    diff = (int)thr - (int)KART_PEDAL_RC_CENTER;
    if(diff < 0) { diff = -diff; }

    return (diff > (int)KART_PEDAL_RC_MARGIN) ? 1U : 0U;
}

void kart_pedal_poll(uint16 period_ms)
{
    uint8  count = 0;
    uint8  online_now;
    uint32 idle;

    /* --- 1. 把环形缓冲里攒的字节解析掉 --- */
    while((pedal_rx_tail != pedal_rx_head) && (count < PEDAL_PARSE_BUDGET))
    {
        uint16 tail = pedal_rx_tail;
        uint8  byte = pedal_rx_ring[tail];

        pedal_rx_tail = (uint16)((tail + 1U) & PEDAL_RX_RING_MASK);
        pedal_feed_byte(byte);
        count++;
    }

    /* --- 2. 失联计时 --- */
    idle = pedal_idle_ms + (uint32)period_ms;
    if(idle > (KART_PEDAL_LOST_TIMEOUT_MS * 10U))
    {
        idle = KART_PEDAL_LOST_TIMEOUT_MS * 10U;    /* 防长期离线时溢出 */
    }
    pedal_idle_ms = idle;

    online_now = (idle <= KART_PEDAL_LOST_TIMEOUT_MS) ? 1U : 0U;
    if(pedal_online && (0U == online_now))
    {
        pedal_stat.link_lost_cnt++;
        /* 失联瞬间就把踏板读数清成"松油门 + 踩刹车",别留着最后一帧的油门值:
         * 5ms 拍在这之后还会继续跑,残留的油门会被一直下发出去。 */
        pedal_thr_pm = 0;
        pedal_brake  = 1;
        pedal_seq_seen = 0;
    }
    pedal_online = online_now;

    /* --- 3. 驾驶模式下的遥控抢占 --- */
    if(MISSION_PEDAL == kart_mission_get_mode())
    {
        if(pedal_rc_wants_takeover())
        {
            kart_mission_set_mode(MISSION_REMOTE);
            return;
        }
        /* 失联【不跳 FAULT】:目标归零但 enable 保持 1,让 PID 主动把后轮拖到停,
         * 转向继续不使能 —— 人手上还有机械转向。具体下发在 control_update。
         * 退出驾驶模式靠菜单的 KART_LEFT 键,不在这里判按键。 */
    }
}

/* ==================== 5ms 拍 ====================
 * 放在 kart_steer_ctrl_update() 之后、power_sync() 之前。 */
void kart_pedal_control_update(void)
{
    float v;

    if(MISSION_PEDAL != kart_mission_get_mode())
    {
        return;
    }

    /* --- 不变量:这个模式下转向电机永远不使能 ---
     * 每拍无条件重申一次,不是"进入时设一次就算了"。理由是别的模块
     * (kart_playback_poll、任务状态机、在线调参)都能把 angle_enable 打开,
     * 一旦被打开,转向电机就会和司机手上的机械连杆对抗。
     * set_angle_enable(0) 是幂等的:重复调只是把两个 PID 再清一次零,
     * 代价是几个浮点赋值,买一条硬保证很值。 */
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);

    /* --- 目标速度 --- */
    if(0U == pedal_online)
    {
        /* 失联:断油,但 enable 保持 1 让 PID 主动减速 */
        v = 0.0f;
    }
    else if(0U != pedal_brake)
    {
        /* 刹车:目标归零且【绕过斜坡】。ramp_step=0 在 kart_control 里的语义是
         * "不限速",也就是目标瞬间到位 —— 这才是刹车该有的响应。
         * enable 不关:关了后轮只是断电滑行,减速比 PID 主动拖到零慢得多。 */
        kart_control_set_ramp_step(0.0f);
        v = 0.0f;
    }
    else
    {
        kart_control_set_ramp_step(KART_PEDAL_RAMP_STEP);
        v = pedal_thr_to_ms(pedal_thr_pm);
    }

    pedal_target_ms = v;
    kart_control_set_target(v * KART_PULSE_MS_TO_V);
}

#else   /* !PEDAL_ENABLE ---- 编译期空壳,零开销 ---- */

void  kart_pedal_init(void)                     { }
void  kart_pedal_rx_callback(void)              { }
void  kart_pedal_poll(uint16 period_ms)         { (void)period_ms; }
void  kart_pedal_control_update(void)           { }
void  kart_pedal_enter(void)                    { }
void  kart_pedal_exit(void)                     { }
void  kart_pedal_note_deny(void)                { }
uint8 kart_pedal_is_online(void)                { return 0; }
uint8 kart_pedal_can_engage(void)               { return 0; }
uint16 kart_pedal_get_throttle_pm(void)         { return 0; }
uint8 kart_pedal_get_brake(void)                { return 1; }
float kart_pedal_get_target_ms(void)            { return 0.0f; }

void kart_pedal_get_stat(kart_pedal_stat_t *dst)
{
    if(NULL != dst)
    {
        kart_pedal_stat_t z = {0};
        *dst = z;
    }
}

#endif  /* PEDAL_ENABLE */
