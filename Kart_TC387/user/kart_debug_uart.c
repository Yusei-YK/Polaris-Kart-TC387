#include "kart_debug_uart.h"
#include "kart_control.h"
#include "kart_encoder.h"
#include "kart_imu.h"
#include "kart_mission.h"
#include "kart_odom.h"
#include "kart_playback.h"
#include "kart_remote.h"
#include "kart_steer_abs.h"
#include "kart_steer_ctrl.h"
#include "isr.h"                 /* 协作式调度器运行时监测 g_sched_* */
#include <string.h>

/*
 * 科目一串口日志（VOFA JustFloat）。
 *
 * 设计边界：
 * - 5 ms中断只增加tick，不在中断内读传感器或发串口；
 * - 主循环每4 tick(50Hz)发送一帧：33个小端float32 + 帧尾00 00 80 7F；
 * - VOFA选JustFloat即可实时显示并导出CSV；不用printf、动态内存、DMA；
 * - 继续沿用原调试串口文件名，避免扩大工程改动。
 */

#define KART_LOG_CHANNELS           (34U)  /* VOFA JustFloat 浮点通道数 */

#define KART_LOG_FLAG_PLAYBACK      (1U << 0)
#define KART_LOG_FLAG_SPEED_ENABLE  (1U << 1)
#define KART_LOG_FLAG_REMOTE_ONLINE (1U << 2)
#define KART_LOG_FLAG_REMOTE_LOW    (1U << 3)
#define KART_LOG_FLAG_HEAD_ENABLE   (1U << 4)
#define KART_LOG_FLAG_ANGLE_ENABLE  (1U << 5)

static volatile uint32 kart_log_tick_5ms = 0U;
static uint32 kart_log_last_sent_tick = 0U;
static uint32 kart_log_sequence = 0U;
static uint16 kart_log_skipped_frames = 0U;

/* ===== VOFA 发送环形缓冲(非阻塞后台发送)=====
 * 底层 uart_write_buffer→IfxAsclin_write8 是全阻塞:每字节写完 spin 等 TX FIFO 排空。
 * 一帧 33ch=136 字节 @460800 直发≈2.95ms,放 5ms 调度里会把控制拍打爆。
 * 方案:poll 只把整帧塞进环形缓冲;background 每次主循环 spin 排 ≤16 字节
 * (=TX FIFO 深度,单次阻塞≤347us),loop 空转多拍即可发完整帧,永不长阻塞控制窗口。
 * head/tail 均只在主循环访问(poll 与 background 同在主循环,无 ISR 并发)。 */
#define KART_LOG_RING_SIZE          (512U)  /* 2 的幂,位与回绕;容纳数帧 136 字节 */
#define KART_LOG_TX_CHUNK           (16U)   /* 后台单次最多发字节数(TX FIFO 深度) */

static uint8  kart_log_ring[KART_LOG_RING_SIZE];
static uint32 kart_log_ring_head = 0U;      /* 写指针:poll 采样填 */
static uint32 kart_log_ring_tail = 0U;      /* 读指针:background 排空 */
static uint16 kart_log_ring_drop = 0U;      /* 环满丢帧计数(诊断) */

/* 日志总闸(2026-07-26):科目二把 UART_10 让给语音模块时必须关闸。
 * 默认开,行为与改动前一致。 */
static uint8  kart_log_enabled = 1U;

/* 环形缓冲可用空间(留 1 字节区分满/空)。 */
static uint32 kart_log_ring_free(void)
{
    uint32 used = (kart_log_ring_head - kart_log_ring_tail) & (KART_LOG_RING_SIZE - 1U);
    return (KART_LOG_RING_SIZE - 1U) - used;
}

/* 无检查写入(调用前必须已确认空间够,否则半帧)。 */
static void kart_log_ring_push(const uint8 *data, uint32 len)
{
    uint32 i;
    for(i = 0; i < len; i++)
    {
        kart_log_ring[kart_log_ring_head] = data[i];
        kart_log_ring_head = (kart_log_ring_head + 1U) & (KART_LOG_RING_SIZE - 1U);
    }
}

static uint8 kart_log_yaw_initialized = 0U;
static float kart_log_last_yaw_deg = 0.0f;
static float kart_log_yaw_unwrapped_deg = 0.0f;

static float kart_log_update_unwrapped_yaw(float yaw_deg)
{
    float delta;

    if(!kart_log_yaw_initialized)
    {
        kart_log_yaw_initialized = 1U;
        kart_log_last_yaw_deg = yaw_deg;
        kart_log_yaw_unwrapped_deg = yaw_deg;
        return kart_log_yaw_unwrapped_deg;
    }

    delta = yaw_deg - kart_log_last_yaw_deg;
    while(delta > 180.0f) delta -= 360.0f;
    while(delta <= -180.0f) delta += 360.0f;

    kart_log_yaw_unwrapped_deg += delta;
    kart_log_last_yaw_deg = yaw_deg;
    return kart_log_yaw_unwrapped_deg;
}

/* VOFA JustFloat：N 个小端 float32 + 帧尾 00 00 80 7F。
 * TC387 为小端，float 内存布局与 JustFloat 一致，直接按字节入环形缓冲。
 * 整帧原子:先查空间够 payload+4 帧尾才写,否则整帧丢弃(丢帧计数),
 * 绝不写半帧——半帧会让 VOFA 后续所有帧错位。 */
static void kart_log_send_justfloat(const float *ch, uint32 count)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    uint32 need = count * 4U + 4U;

    if(kart_log_ring_free() < need)
    {
        if(kart_log_ring_drop < 65535U) kart_log_ring_drop++;
        return;                         /* 环满:丢整帧,不阻塞、不写半帧 */
    }
    kart_log_ring_push((const uint8 *)ch, count * 4U);
    kart_log_ring_push(tail, 4U);
}

/* 后台非阻塞发送:每次主循环 spin 调一次,最多排 KART_LOG_TX_CHUNK 字节。
 * 单次 uart_write_buffer(≤16B) 阻塞 ≤347us,远小于 5ms 控制窗口。
 * 环形缓冲有货就发一块,空转即返回,整帧靠多拍 spin 累计发完。 */
void kart_debug_uart_background_poll(void)
{
    uint32 used;
    uint32 n;
    uint32 i;
    uint8  chunk[KART_LOG_TX_CHUNK];

    if(!kart_log_enabled) return;       /* 关闸:一个字节都不许出去(口可能已让给语音) */

    used = (kart_log_ring_head - kart_log_ring_tail) & (KART_LOG_RING_SIZE - 1U);
    if(used == 0U) return;

    n = (used < KART_LOG_TX_CHUNK) ? used : KART_LOG_TX_CHUNK;
    for(i = 0; i < n; i++)
    {
        chunk[i] = kart_log_ring[kart_log_ring_tail];
        kart_log_ring_tail = (kart_log_ring_tail + 1U) & (KART_LOG_RING_SIZE - 1U);
    }
    uart_write_buffer(BOARD_AUX_UART_INDEX, chunk, n);
}

void kart_debug_uart_init(void)
{
    kart_log_tick_5ms = 0U;
    kart_log_last_sent_tick = 0U;
    kart_log_sequence = 0U;
    kart_log_skipped_frames = 0U;
    kart_log_yaw_initialized = 0U;
    kart_log_last_yaw_deg = 0.0f;
    kart_log_yaw_unwrapped_deg = 0.0f;

    kart_log_ring_head = 0U;
    kart_log_ring_tail = 0U;
    kart_log_ring_drop = 0U;

    kart_log_enabled = 1U;

    uart_init(BOARD_AUX_UART_INDEX, BOARD_AUX_UART_BAUD_FAST,
              BOARD_AUX_UART_TX_PIN, BOARD_AUX_UART_RX_PIN);
}

/* 关闸时清空环形缓冲:否则重新开闸后先吐出一堆过期帧,VOFA 波形会有一段假历史;
 * 而且残留可能是半帧,会让后续所有 JustFloat 帧错位。 */
void kart_debug_uart_set_enabled(uint8 enabled)
{
    kart_log_enabled = enabled ? 1U : 0U;

    if(!kart_log_enabled)
    {
        kart_log_ring_head = 0U;
        kart_log_ring_tail = 0U;
    }
    else
    {
        /* 重新开闸:把节拍基准对齐到当前,避免 elapsed_ticks 一次跨过很多拍
         * 被 poll() 当成"漏帧"累加进 skipped 统计。 */
        kart_log_last_sent_tick = kart_log_tick_5ms;
    }
}

uint8 kart_debug_uart_is_enabled(void)
{
    return kart_log_enabled;
}

void kart_debug_uart_tick_5ms(void)
{
    kart_log_tick_5ms++;
}

void kart_debug_uart_poll(void)
{
    uint32 now_tick = kart_log_tick_5ms;
    uint32 elapsed_ticks = now_tick - kart_log_last_sent_tick;
    uint32 elapsed_periods;
    uint32 skipped_now;
    uint8 flags = 0U;
    float yaw_deg;
    float yaw_unwrapped_deg;

    if(!kart_log_enabled)
    {
        return;                         /* 关闸期间连组帧都不做,省 CPU 也不污染环形缓冲 */
    }

    if(elapsed_ticks < KART_LOG_PERIOD_TICKS)
    {
        return;
    }

    elapsed_periods = elapsed_ticks / KART_LOG_PERIOD_TICKS;
    if(elapsed_periods > 1U)
    {
        skipped_now = elapsed_periods - 1U;
        if(skipped_now >= (uint32)(65535U - kart_log_skipped_frames))
        {
            kart_log_skipped_frames = 65535U;
        }
        else
        {
            kart_log_skipped_frames = (uint16)(kart_log_skipped_frames + skipped_now);
        }
    }
    kart_log_last_sent_tick = now_tick;  /* 不追发历史帧。 */

    if(kart_playback_is_running()) flags |= KART_LOG_FLAG_PLAYBACK;
    if(kart_control_is_enabled()) flags |= KART_LOG_FLAG_SPEED_ENABLE;
    if(kart_remote_is_online()) flags |= KART_LOG_FLAG_REMOTE_ONLINE;
    if(kart_remote_get_sw3() == KART_REMOTE_SW3_L) flags |= KART_LOG_FLAG_REMOTE_LOW;
    if(kart_steer.head_enable) flags |= KART_LOG_FLAG_HEAD_ENABLE;
    if(kart_steer.angle_enable) flags |= KART_LOG_FLAG_ANGLE_ENABLE;

    yaw_deg = kart_imu_get_yaw();
    yaw_unwrapped_deg = kart_log_update_unwrapped_yaw(yaw_deg);

    {
        float ch[KART_LOG_CHANNELS];
        ch[0]  = (float)kart_mission_get_mode();          /* 0 mission模式 */
        ch[1]  = (float)kart_mission_get_subject1_stage(); /* 1 科目一阶段 */
        ch[2]  = (float)flags;                             /* 2 状态标志位 */
        ch[3]  = (float)kart_playback_get_index();         /* 3 播放索引 */
        ch[4]  = kart_control_get_target();                /* 4 目标速度 */
        ch[5]  = kart_control_get_left_meas();             /* 5 左轮实测 */
        ch[6]  = kart_control_get_right_meas();            /* 6 右轮实测 */
        ch[7]  = (float)kart_control_get_left_output();    /* 7 左输出 */
        ch[8]  = (float)kart_control_get_right_output();   /* 8 右输出 */
        ch[9]  = yaw_deg;                                  /* 9 yaw */
        ch[10] = yaw_unwrapped_deg;                        /* 10 展开yaw */
        ch[11] = kart_imu_get_yaw_rate_dps();              /* 11 yaw速率 */
        ch[12] = kart_imu_get_yaw_bias_dps();              /* 12 yaw零偏 */
        ch[13] = kart_steer_get_target_yaw();              /* 13 目标航向 */
        ch[14] = (float)kart_steer_abs_get_raw();          /* 14 转向raw */
        ch[15] = kart_steer_get_target_delta();            /* 15 目标转角 */
        ch[16] = (float)kart_steer_get_output();           /* 16 转向输出 */
        ch[17] = kart_odom_get_x();                        /* 17 odom x */
        ch[18] = kart_odom_get_y();                        /* 18 odom y */
        ch[19] = kart_odom_get_dist();                     /* 19 odom 里程 */
        ch[20] = kart_playback_get_cur_x();                /* 20 诊断:投影当前x */
        ch[21] = kart_playback_get_cur_y();                /* 21 诊断:投影当前y */
        ch[22] = kart_playback_get_aim_x();                /* 22 诊断:瞄准点x */
        ch[23] = kart_playback_get_aim_y();                /* 23 诊断:瞄准点y */
        ch[24] = kart_imu_get_dt_us();                     /* 24 IMU积分步长(us):稳定应≈5000 */
        ch[25] = (float)g_sched_last_exec_us;              /* 25 调度上拍分发耗时(us) */
        ch[26] = (float)g_sched_max_exec_us;               /* 26 调度历史最大耗时(us):应<5000 */
        ch[27] = (float)g_sched_overrun_count;             /* 27 漏周期累计:稳态应长期为0 */
        ch[28] = kart_steer_get_meas_delta();              /* 28 转向内环实测转角(center_delta):与CH15目标对比判内环跟踪/回中 */
        ch[29] = (float)kart_encoder_get_left_delta();     /* 29 左轮编码器原始delta(未滤波,脉冲/5ms):与CH5滤波值对比看滞后/抖动 */
        ch[30] = (float)kart_encoder_get_right_delta();    /* 30 右轮编码器原始delta(未滤波,脉冲/5ms):与CH6滤波值对比看滞后/抖动 */
        ch[31] = (float)kart_remote_get_channel(KART_REMOTE_CH_THROTTLE); /* 31 油门通道raw:静止应≈THR_CENTER(880),偏则映射出非0目标速度 */
        ch[32] = (float)kart_remote_get_channel(KART_REMOTE_CH_STEER);    /* 32 方向通道raw:静止应≈STEER_CENTER(968) */
        ch[33] = kart_control_get_target_cmd();            /* 33 斜坡前的请求目标(上层写入):与CH4(斜坡后)对比即斜坡曲线,CH4追不上CH33就是在爬坡 */
        kart_log_send_justfloat(ch, KART_LOG_CHANNELS);
    }
    kart_log_sequence++;
}
