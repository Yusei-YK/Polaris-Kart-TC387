/*********************************************************************************************************************
 * 文件名称  kart_person_link
 * 功能说明  TC4D7 人体跟踪遥测帧接收。帧格式/端口选型/设计取舍见 .h 顶部。
 *
 * 【本文件只做三件事】
 *   1) 中断只收字节入环形缓冲；主循环拼帧并校验，把不合法的全丢掉。
 *   2) 失联计时与在线判定。
 *   3) 把归一化字段折成 kart_vtrack_result_t。
 *   不做任何转向/速度计算 —— 那是 kart_follow 的活，它里面的纯跟踪与近距联锁
 *   已经写好且与转向标定（R×|delta|=1480）对齐，在这里再算一遍就是两套参数。
 ********************************************************************************************************************/
#include "kart_person_link.h"

#if PERSON_LINK_ENABLE
#include "zf_common_interrupt.h"
#include "zf_driver_uart.h"

#pragma section all "cpu0_dsram"

/* -------------------- UART RX 环形缓冲（中断生产，主循环消费）--------------------
 * 115200 满线速约 11.5KB/s。1024B 可吸收约 89ms 的菜单整屏刷新抖动；即使 4D7
 * 串口混入调试输出，中断也只做一次读、一次写和下标推进，不再在中断里跑 CRC。
 * 长度必须是 2 的幂，留一个空槽区分满/空。 */
#define PLINK_RX_RING_SIZE          (1024U)
#define PLINK_RX_RING_MASK          (PLINK_RX_RING_SIZE - 1U)
#define PLINK_PARSE_BUDGET_PER_POLL (192U)

static uint8          plink_rx_ring[PLINK_RX_RING_SIZE] = {0};
static volatile uint16 plink_rx_head = 0;
static volatile uint16 plink_rx_tail = 0;
static volatile uint8  plink_rx_overflow = 0;
static volatile uint8  plink_rx_active = 0;

/* -------------------- 拼帧缓冲（仅主循环上下文访问）-------------------- */
static uint8  plink_rx_buf[PLINK_FRAME_LEN] = {0};
static uint8  plink_rx_len = 0;

/* -------------------- 共享状态（byte_count 中断写，其余主循环写）-------------------- */
static volatile kart_person_frame_t     plink_frame = {0};
static volatile kart_person_link_stat_t plink_stat  = {0};
static volatile uint8  plink_frame_seen = 0;    /* 从上电到现在收到过合法帧 */
static volatile uint8  plink_seq_seen   = 0;    /* 序号基准已建立 */
static volatile uint16 plink_seq_last   = 0;

/* -------------------- 失联计时 -------------------- */
/* 上电即视为未在线（和 kart_remote 一样的初值套路）：否则上电第一拍
 * is_online() 会返回 1，让上层以为有目标而起步。 */
static volatile uint32 plink_idle_ms = PLINK_LOST_TIMEOUT_MS + 1U;
static uint8  plink_online = 0;

/* -------------------- 近距刹车锁存（主循环独占）-------------------- */
static uint8  plink_near_latch = 0;

/* -------------------- kart_vtrack 快照（主循环独占）-------------------- */
static kart_vtrack_result_t plink_vtrack = {0};

/* -------------------- 端口参数展开 -------------------- */
/* board_pins.h 里已经根据 PERSON_LINK_PORT 定义好了下面三个宏，
 * 本文件不重复判断端口，避免两处条件不同步。 */
#define PLINK_UART      BOARD_PERSON_LINK_UART_INDEX
#define PLINK_TX_PIN    BOARD_PERSON_LINK_TX_PIN
#define PLINK_RX_PIN    BOARD_PERSON_LINK_RX_PIN

/* zf_driver_uart.c 的 uart_rx_interrupt(uart_n, ...) 取中断源时误用了全局
 * uart_config.asclin（即“最近一次 init 的串口”），模式切换后可能关错 UART。
 * PLINK 生命周期门控必须按固定模块地址取 SRC，不能调用那个包装函数。 */
static void plink_rx_irq_set(uint8 enabled)
{
    Ifx_ASCLIN *asclin;
    volatile Ifx_SRC_SRCR *src;

    asclin = IfxAsclin_getAddress((IfxAsclin_Index)PLINK_UART);
    src = IfxAsclin_getSrcPointerRx(asclin);
    if(enabled)
    {
        /* 待机期间可能留下 pending SRC；不先清，空 FIFO 也可能立刻进 ISR，
         * 随后卡在 uart_read_byte() 的等待循环里。 */
        IfxSrc_clearRequest(src);
        IfxAsclin_enableRxFifoFillLevelFlag(asclin, (boolean)1);
        IfxSrc_enable(src);
    }
    else
    {
        IfxSrc_disable(src);
        IfxAsclin_enableRxFifoFillLevelFlag(asclin, (boolean)0);
        IfxSrc_clearRequest(src);
    }
}

/*=========================== CRC-16/CCITT-FALSE ===========================*/
/* poly 0x1021，init 0xFFFF，不反转输入输出，xorout 0。
 * 完全照搬 4D7 发送端的 person_crc16_ccitt()，一个字不改 ——
 * CRC 两端必须位位一致，“优化”成查表法只会多一个出错的机会。
 * 21 字节 × 8 位 = 168 次循环。只允许在主循环解析路径调用，禁止搬回中断。 */
static uint16 plink_crc16(const uint8 *data, uint16 length)
{
    uint16 crc = 0xFFFFU;
    uint16 index;
    uint8  bit;

    for(index = 0U; index < length; index++)
    {
        crc ^= (uint16)((uint16)data[index] << 8U);
        for(bit = 0U; bit < 8U; bit++)
        {
            if(crc & 0x8000U)
            {
                crc = (uint16)((uint16)(crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16)(crc << 1U);
            }
        }
    }
    return crc;
}

/*=========================== 小端取值 ===========================*/
static uint16 plink_rd_u16(const uint8 *p)
{
    return (uint16)((uint16)p[0] | (uint16)((uint16)p[1] << 8U));
}

/*=========================== 帧入库 ===========================*/
/* 已过头/类型/CRC 校验的帧。只在主循环上下文调。 */
static void plink_accept(const uint8 *buf)
{
    uint16 seq = plink_rd_u16(&buf[5]);

    /* 序号新鲜度（协议规则 5）。用 (int16)(new - last) > 0 而不是 new > last：
     * 序号是 uint16 自然回绕，65535 → 0 时后者会误判为“旧包”，
     * 把差强转 int16 后符号位自动处理回绕。
     * 只在基准已建立后才判：首帧无论序号多少都接受，否则 4D7 先开机
     * 跑了一阵子后 387 才上电，基准为 0 会把前半个回绕周期的帧全丢。 */
    if(plink_seq_seen)
    {
        if((int16)((int16)seq - (int16)plink_seq_last) <= 0)
        {
            plink_stat.seq_stale++;
            return;
        }
    }
    plink_seq_last = seq;
    plink_seq_seen = 1;

    plink_frame.sequence       = seq;
    plink_frame.flags          = buf[7];
    plink_frame.zone           = (int8)buf[8];
    plink_frame.steering_error = (int16)plink_rd_u16(&buf[9]);
    plink_frame.center_x       = plink_rd_u16(&buf[11]);
    plink_frame.center_y       = plink_rd_u16(&buf[13]);
    plink_frame.width          = plink_rd_u16(&buf[15]);
    plink_frame.height         = plink_rd_u16(&buf[17]);
    plink_frame.confidence     = plink_rd_u16(&buf[19]);
    plink_frame.run_time       = plink_rd_u16(&buf[21]);

    plink_frame_seen = 1;
    plink_stat.frame_ok++;

    /* 【关键】只有带 VALID 位的帧才清失联计时。
     * 4D7 在目标丢失时仍然会持续发帧（字段清零、VALID 位为 0），
     * 若这里无条件清零，链路健康但人走掉了也会被当成“在线”，
     * 车会拿着全零方位笔直往前开 —— 协议规则 6 要求的就是这个情况要停。 */
    if(plink_frame.flags & PLINK_FLAG_VALID)
    {
        plink_idle_ms = 0;
    }
}

/*=========================== 主循环逐字节解析 ===========================*/
static void plink_parse_byte(uint8 byte)
{
    /* 帧头对齐：前两个字节不对就原地滑动，不整帧丢。
     * 为何不像 kart_remote 那样“收满 25 字节再看头尾”：本链路是连续流，
     * 一旦失了一个字节，那种写法要靠字节间隔超时才能重对齐，而 4D7 是
     * 背靠背连发的，可能几秒都等不到一个足够大的间隔 → 长时间全帧丢。 */
    if(0U == plink_rx_len)
    {
        if(PLINK_HEAD0 != byte)
        {
            return;                     /* 不计 hdr_err：这是正常的对齐过程，不是错误 */
        }
    }
    else if(1U == plink_rx_len)
    {
        if(PLINK_HEAD1 != byte)
        {
            /* 0xA5 0xA5 的情况：第二个 0xA5 可能才是真正的帧头起点，
             * 所以不能无条件清零，否则 A5 A5 5A ... 这种流会被错过。 */
            plink_rx_len = (PLINK_HEAD0 == byte) ? 1U : 0U;
            if(0U == plink_rx_len)
            {
                return;
            }
            plink_rx_buf[0] = PLINK_HEAD0;
            return;
        }
    }

    plink_rx_buf[plink_rx_len++] = byte;

    /* 头后紧跟的三个固定字段就地校，不等收满 25 字节：
     * 发现不对就立即重对齐，能把一次错对齐的代价从 25 字节降到 5 字节。 */
    if((3U == plink_rx_len) && (PLINK_VERSION != plink_rx_buf[2]))
    {
        plink_stat.hdr_err++;
        plink_rx_len = 0;
        return;
    }
    if((4U == plink_rx_len) && (PLINK_TYPE_PERSON != plink_rx_buf[3]))
    {
        plink_stat.hdr_err++;
        plink_rx_len = 0;
        return;
    }
    if((5U == plink_rx_len) && (PLINK_PAYLOAD_LEN != plink_rx_buf[4]))
    {
        plink_stat.hdr_err++;
        plink_rx_len = 0;
        return;
    }

    if(plink_rx_len >= PLINK_FRAME_LEN)
    {
        uint16 crc_calc = plink_crc16(&plink_rx_buf[PLINK_CRC_COVER_BEGIN],
                                       PLINK_CRC_COVER_LEN);
        uint16 crc_recv = plink_rd_u16(&plink_rx_buf[23]);

        if(crc_calc == crc_recv)
        {
            plink_accept(plink_rx_buf);
        }
        else
        {
            plink_stat.crc_err++;       /* 丢弃，不重传（协议规则 4）*/
        }
        plink_rx_len = 0;
    }
}

/*=========================== RX 中断回调 ===========================*/
void kart_person_link_rx_callback(void)
{
    uint8 byte;
    uint16 head;
    uint16 next;

    /* ISR 保持最短：不计时、不找帧头、不算 CRC、不写业务帧。UART3 遥控中断
     * 因而不会再被 PLINK 的整帧处理拖延。 */
    byte = uart_read_byte(PLINK_UART);
    if(0U == plink_rx_active)
    {
        return;                         /* 关闭过程中已经挂起的最后一次中断 */
    }
    plink_stat.byte_count++;

    head = plink_rx_head;
    next = (uint16)((head + 1U) & PLINK_RX_RING_MASK);
    if(next == plink_rx_tail)
    {
        /* 满时丢当前字节并通知主循环放弃半帧；绝不在 ISR 里追赶处理。 */
        plink_rx_overflow = 1U;
        return;
    }

    plink_rx_ring[head] = byte;
    plink_rx_head = next;
}

/* 每个 10ms 拍最多处理固定数量，异常满线速输入也不能无限占用菜单/控制时间。 */
static void plink_drain_rx(void)
{
    uint16 count = 0U;
    uint32 istate;
    uint8 overflow;

    istate = interrupt_global_disable();
    overflow = plink_rx_overflow;
    plink_rx_overflow = 0U;
    interrupt_global_enable(istate);

    if(overflow)
    {
        plink_rx_len = 0U;
        plink_stat.resync++;
    }

    while((plink_rx_tail != plink_rx_head) && (count < PLINK_PARSE_BUDGET_PER_POLL))
    {
        uint16 tail = plink_rx_tail;
        uint8 byte = plink_rx_ring[tail];
        plink_rx_tail = (uint16)((tail + 1U) & PLINK_RX_RING_MASK);
        plink_parse_byte(byte);
        count++;
    }
}

/*=========================== 初始化 ===========================*/
void kart_person_link_init(void)
{
    plink_rx_len     = 0;
    plink_rx_head    = 0;
    plink_rx_tail    = 0;
    plink_rx_overflow = 0;
    plink_rx_active  = 0;
    plink_frame_seen = 0;
    plink_seq_seen   = 0;
    plink_seq_last   = 0;
    plink_near_latch = 0;
    plink_idle_ms    = PLINK_LOST_TIMEOUT_MS + 1U;
    plink_online     = 0;

    /* 不用 uart_sbus_init：那个是 2 停止位 + 偶校验（SBUS 专用），
     * 4D7 发的是标准 8N1，必须用 uart_init。 */
    uart_init(PLINK_UART, PERSON_LINK_BAUD, PLINK_TX_PIN, PLINK_RX_PIN);

    /* uart_init 结束时 RX 中断默认关闭。保持关闭，等进入科目三时再开；否则
     * 4D7 在整个菜单阶段持续发数据，毫无收益地抢占软件 SPI 和遥控处理。 */
}

void kart_person_link_set_rx_enabled(uint8 enabled)
{
    uint8 dummy;
    uint16 flush_count;
    uint32 istate;

    if(enabled)
    {
        /* 先保持关中断，清掉待机期间积在硬件 FIFO 里的旧数据，再发布 active。
         * 清理有上限，防对端满线速发送时初始化路径被拖住。 */
        plink_rx_active = 0U;
        plink_rx_irq_set(0U);
        flush_count = 0U;
        while((flush_count < 64U) && uart_query_byte(PLINK_UART, &dummy))
        {
            flush_count++;
        }

        istate = interrupt_global_disable();
        plink_rx_head = 0U;
        plink_rx_tail = 0U;
        plink_rx_overflow = 0U;
        plink_rx_len = 0U;
        plink_frame_seen = 0U;
        plink_seq_seen = 0U;
        plink_idle_ms = PLINK_LOST_TIMEOUT_MS + 1U;
        plink_online = 0U;
        plink_rx_active = 1U;
        interrupt_global_enable(istate);
        plink_rx_irq_set(1U);
    }
    else
    {
        /* 先撤 active，再关中断；即使已有一个 ISR 挂起，它也只读掉字节后返回。 */
        plink_rx_active = 0U;
        plink_rx_irq_set(0U);

        istate = interrupt_global_disable();
        plink_rx_head = 0U;
        plink_rx_tail = 0U;
        plink_rx_overflow = 0U;
        plink_rx_len = 0U;
        plink_frame_seen = 0U;
        plink_idle_ms = PLINK_LOST_TIMEOUT_MS + 1U;
        plink_online = 0U;
        interrupt_global_enable(istate);
    }
}

/*=========================== 10ms 轮询 ===========================*/
void kart_person_link_poll(uint16 period_ms)
{
    uint32 idle;
    uint8  online_now;
    kart_person_frame_t f;
    uint8  got;
    float  err_norm;
    float  height_norm;
    float  conf;

    /* 先排空一部分 RX，再做失联判定。这样本拍刚收到的 VALID 帧能及时清零计时。 */
    plink_drain_rx();

    idle = plink_idle_ms;
    if(idle < 0xFFFF0000U)                  /* 封顶防溢出 */
    {
        idle += period_ms;
        plink_idle_ms = idle;
    }

    online_now = (idle <= PLINK_LOST_TIMEOUT_MS) ? 1U : 0U;

    /* 在线 → 失联 的下降沿计数。看这个数字能区分两种毛病：
     *   lost_events 永远是 0 且 frame_ok 不涨 → 链路从未通过
     *   lost_events 持续涨            → 链路断续，或 4D7 推理时常丢目标 */
    if(plink_online && (0U == online_now))
    {
        plink_stat.lost_events++;
        /* 4D7 可能因掉电/看门狗复位而把 16 位序号重置为 0。链路已确认失联后，
         * 旧序号不再能证明新旧，清掉基准，让重连后第一个 CRC 正确的帧建立新基准。 */
        plink_seq_seen = 0U;
    }
    plink_online = online_now;

    /* -------- 合成 kart_vtrack -------- */
    got = kart_person_link_get_frame(&f);

    if((0U == got) || (0U == plink_online) || (0U == (f.flags & PLINK_FLAG_VALID)))
    {
        /* 无效。不要只把 valid 置 0 就了事：bearing/scale_r 得一并回中立值。
         * 因为 kart_follow 的丢失分支会把上一拍的 near_latch 继续持住，
         * 若 scale_r 冻在 1.40 上，人重新出现时第一拍依旧是锁停态。 */
        plink_near_latch = 0;
        plink_vtrack.valid       = 0;
        plink_vtrack.bearing_rad = 0.0f;
        plink_vtrack.scale_level = KART_VTRACK_SCALE_NORMAL;
        plink_vtrack.scale_r     = 1.0f;
        plink_vtrack.confidence  = 0;
    }
    else
    {
        /* 方位角：steering_error 已是 ±1000 归一化，负=左。
         * kart_vtrack 的约定是 bearing_rad > 0 为右侧，两边同号，不用取反。
         * 【实车第一次跑必须确认这个符号】翻了就是人往右、车往左追。
         * 确认方法：人站车右前方 → VOFA ch43(bearing_deg) 应为正。 */
        err_norm = (float)f.steering_error * 0.001f;
        if(err_norm >  1.0f) { err_norm =  1.0f; }
        if(err_norm < -1.0f) { err_norm = -1.0f; }
        plink_vtrack.bearing_rad = degree_to_rad(err_norm * PLINK_HALF_FOV_DEG);

        /* 距离档 → scale_level / scale_r。FLAGS 三个位理论上互斥，
         * 但接收端不假设对方一定对：按 NEAR > FAR > MID 优先序取，
         * 一个位都没置时当 MID（最保守：不加速也不刹车）。 */
        if(f.flags & PLINK_FLAG_NEAR)
        {
            plink_vtrack.scale_level = KART_VTRACK_SCALE_TOO_NEAR;
            plink_vtrack.scale_r     = PLINK_SCALE_R_NEAR;
        }
        else if(f.flags & PLINK_FLAG_FAR)
        {
            plink_vtrack.scale_level = KART_VTRACK_SCALE_TOO_FAR;
            plink_vtrack.scale_r     = PLINK_SCALE_R_FAR;
        }
        else
        {
            plink_vtrack.scale_level = KART_VTRACK_SCALE_NORMAL;
            plink_vtrack.scale_r     = PLINK_SCALE_R_MID;
        }

        /* 近距刹车：用连续量 height 带回差判，命中时把 scale_r 抬到
         * kart_follow 的 NEAR_STOP_R 以上，让那边的联锁去锁停。
         * 为何不在本模块直接输出“停”：本模块是 Kart_App 层，不该决定车速；
         * 而且 kart_follow 里已经有带计数确认（NEAR_STOP_TICKS=3）的锁存，
         * 两层串联比在这里重新实现一套可靠。 */
        height_norm = (float)f.height * 0.001f;
        if(plink_near_latch)
        {
            if(height_norm <= PLINK_NEAR_HEIGHT_RESUME)
            {
                plink_near_latch = 0;
            }
        }
        else
        {
            if(height_norm >= PLINK_NEAR_HEIGHT_STOP)
            {
                plink_near_latch = 1;
            }
        }
        if(plink_near_latch)
        {
            plink_vtrack.scale_level = KART_VTRACK_SCALE_TOO_NEAR;
            plink_vtrack.scale_r     = PLINK_SCALE_R_STOP;
        }

        /* 置信度 0..1000 → 0..255（乘 0.255，结果同 ×255/1000）。
         * 只有 TRACKING 位也置了才给满值；仅 VALID 无 TRACKING 时打七折，
         * 让调试页能区分“单帧检到”与“稳定跟上”。 */
        conf = (float)f.confidence * 0.255f;
        if(0U == (f.flags & PLINK_FLAG_TRACKING))
        {
            conf *= 0.7f;
        }
        if(conf > 255.0f) { conf = 255.0f; }
        if(conf < 0.0f)   { conf = 0.0f; }
        plink_vtrack.confidence = (uint8)conf;

        plink_vtrack.valid = 1;
    }

    /* 下面四个字段 kart_follow 不读，填成有意义的值方便调试页直接复用。
     * alive_count 借位表示“本帧检到几个目标”：4D7 只传单目标，所以 0/1。 */
    plink_vtrack.alive_count = plink_vtrack.valid;
    plink_vtrack.total_count = plink_vtrack.valid;
    plink_vtrack.fb_error_median = 0.0f;
    plink_vtrack.frames_since_detector =
        plink_vtrack.valid ? 0U : (uint16)VTRACK_MAX_FRAMES_SINCE_DET;
}

/*=========================== 读取接口 ===========================*/
uint8 kart_person_link_is_online(void)
{
    return plink_online;
}

uint8 kart_person_link_get_frame(kart_person_frame_t *dst)
{
    uint32 istate;
    uint8 seen;

    if(NULL == dst)
    {
        return 0;
    }

    /* 成组快照。必须关中断：plink_frame 有20 多个字节，中断里是逐字段写的，
     * 不关中断会读到新旧帧混搭的结果（比如旧帧的 steering_error 配新帧的 flags）。
     * 与 kart_remote 同一套路。关中断窗口只有一次结构体拷贝（~24 字节），
     * 对 5ms 控制环的抖动可忽略。 */
    istate = interrupt_global_disable();
    seen = plink_frame_seen;
    if(seen)
    {
        dst->sequence       = plink_frame.sequence;
        dst->flags          = plink_frame.flags;
        dst->zone           = plink_frame.zone;
        dst->steering_error = plink_frame.steering_error;
        dst->center_x       = plink_frame.center_x;
        dst->center_y       = plink_frame.center_y;
        dst->width          = plink_frame.width;
        dst->height         = plink_frame.height;
        dst->confidence     = plink_frame.confidence;
        dst->run_time       = plink_frame.run_time;
    }
    interrupt_global_enable(istate);

    return seen;
}

void kart_person_link_get_stat(kart_person_link_stat_t *dst)
{
    uint32 istate;

    if(NULL == dst)
    {
        return;
    }

    istate = interrupt_global_disable();
    dst->byte_count  = plink_stat.byte_count;
    dst->frame_ok    = plink_stat.frame_ok;
    dst->crc_err     = plink_stat.crc_err;
    dst->hdr_err     = plink_stat.hdr_err;
    dst->seq_stale   = plink_stat.seq_stale;
    dst->resync      = plink_stat.resync;
    dst->lost_events = plink_stat.lost_events;
    interrupt_global_enable(istate);
}

const kart_vtrack_result_t *kart_person_link_vtrack(void)
{
    /* 不加锁：plink_vtrack 只由 kart_person_link_poll() 写（主循环上下文），
     * 读它的 kart_follow_update() 也在主循环，同一上下文不会打断。
     * 中断里绝不能调本函数。 */
    return &plink_vtrack;
}

#pragma section all restore

#else   /* !PERSON_LINK_ENABLE */

/* 关闭时给空实现。为何不让调用方自己 #if：
 * cpu0_main / isr.c / kart_mission 三处都要调，每处包一层 #if 就是三处可能写错的
 * 条件。空实现占几十字节 kart_flash，换掉调用侧全部条件编译，划得来。
 * 例外：isr.c 里的分发必须用 #if，因为那里是“调 A 还是调 B”的二选一，
 * 不是“调不调”。 */
void kart_person_link_init(void)                        { }
void kart_person_link_set_rx_enabled(uint8 enabled)     { (void)enabled; }
void kart_person_link_poll(uint16 period_ms)            { (void)period_ms; }
void kart_person_link_rx_callback(void)                 { }
uint8 kart_person_link_is_online(void)                  { return 0; }
void kart_person_link_get_stat(kart_person_link_stat_t *dst)
{
    if(NULL != dst)
    {
        dst->byte_count = 0; dst->frame_ok = 0; dst->crc_err = 0;
        dst->hdr_err = 0; dst->seq_stale = 0; dst->resync = 0; dst->lost_events = 0;
    }
}

uint8 kart_person_link_get_frame(kart_person_frame_t *dst)
{
    (void)dst;
    return 0;
}

const kart_vtrack_result_t *kart_person_link_vtrack(void)
{
    /* 永远返回 valid=0 的静态体，kart_follow 会走丢失分支减速停住。
     * 不返回 NULL：虽然 kart_follow_update() 对 NULL 有容错，但接口承诺了非空，
     * 让调用方不必写 NULL 判断。 */
    static const kart_vtrack_result_t empty = {0};
    return &empty;
}

#endif  /* PERSON_LINK_ENABLE */
