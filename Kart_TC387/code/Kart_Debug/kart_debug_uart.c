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
#include "kart_camera.h"         /* 摄像头采集链路诊断计数 CH34-CH38 */
#include "kart_vision.h"         /* 视觉检测结果 CH39-CH42 */
#include "kart_follow.h"         /* 跟随控制律状态 CH43-CH44 */
#include "kart_wifi.h"            /* WiFi 图传链路诊断 CH39-CH44(借用视觉通道,摄像头未启用) */
#include "kart_person_link.h"     /* TC4D7 人体视觉链路诊断 CH39-CH44(与视觉通道二选一) */
#include "isr.h"                 /* 协作式调度器运行时监测 g_sched_* */
#include "kart_assist_img.h"      /* CH12 图传占用标志 g_kart_aimg_busy */
#include "kart_multicore.h"       /* CH22/23/31 core3 视觉耗时与投递统计 */
#include <string.h>

/*
 * 科目一串口日志（VOFA JustFloat）。
 *
 * 设计边界：
 * - 5 ms中断只增加tick，不在中断内读传感器或发串口；
 * - 主循环每4 tick(50Hz)发送一帧：51个小端float32 + 帧尾00 00 80 7F（共208字节）；
 * - VOFA选JustFloat即可实时显示并导出CSV；不用printf、动态内存、DMA；
 * - 继续沿用原调试串口文件名，避免扩大工程改动。
 */

/* 2026-08-08 由 34 加到 39:新增 CH34-CH38 摄像头采集链路诊断。
 * 2026-08-09 由 39 加到 45:新增 CH39-CH44 科目三视觉跟随(检测结果 + 跟随状态)。
 * 2026-08-14 由 45 加到 48:新增 CH45-CH47 科目三事件编号、级别和时间。
 * 2026-08-14 由48加到50:新增CH48跟随目标速度、CH49跟随目标转角。
 * 注意 VOFA 端的通道数要同步改成50,否则帧长不匹配、波形会全体错位。 */
/* 【日志通道组】1 = 科目三跟随精简 26 路,0 = 原 51 路全量(科目一/二/四用)。
 * 精简的理由是分析耗时:51 列 CSV 每次定位都要先翻注释找列号,而跟随段真正
 * 在变的只有二十几路,其余是 playback/遥控/odom,全程恒定。
 * 【改这个宏必须同步改 VOFA 的通道数】否则帧长不匹配、波形全体错位。 */
#define KART_LOG_PROFILE_S3         (1)

#if KART_LOG_PROFILE_S3
#define KART_LOG_CHANNELS           (26U)  /* VOFA 通道数:帧长 26*4+4=108B */
#else
#define KART_LOG_CHANNELS           (51U)  /* 帧长 51*4+4=208B */
#endif

static volatile uint16 kart_event_id = 0U;
static volatile uint8  kart_event_level = KART_EVENT_LEVEL_INFO;
static volatile uint32 kart_event_tick = 0U;

void kart_debug_uart_set_event(uint16 event_id, uint8 level)
{
    kart_event_id = event_id;
    kart_event_level = level;
    kart_event_tick = g_kart_tick_5ms;
}

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
 * 一帧 34ch=140 字节 @460800 直发≈3.04ms,放 5ms 调度里会把控制拍打爆。
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
#if KART_LOG_PROFILE_S3
        {
            const kart_vision_result_t *v = kart_multicore_vision_get();
            const kart_follow_out_t    *f = kart_follow_get();

            ch[0]  = (float)kart_mission_get_mode();
            ch[1]  = (float)kart_mission_get_subject3_stage();
            ch[2]  = (float)flags;
            ch[3]  = (float)kart_mission_get_s3_vision_seq();         /* 不涨=没出帧 */
            ch[4]  = (float)kart_mission_get_s3_vision_age() * 10.0f; /* 快照年龄 ms */
            ch[5]  = (float)kart_multicore_vision_last_us() * 0.001f; /* core3 单帧耗时 ms */
            ch[6]  = (float)kart_multicore_vision_reject();
            ch[7]  = (float)g_sched_overrun_count;                    /* 搬核后应压平 */
            ch[8]  = (float)g_sched_max_exec_us;
            ch[9]  = (float)v->valid;
            ch[10] = (float)v->reject;
            ch[11] = f->bearing_deg;
            ch[12] = v->dist_m;
            ch[13] = (float)v->width_px;
            ch[14] = kart_steer_get_target_delta();
            ch[15] = kart_steer_get_meas_delta();
            ch[16] = yaw_deg;
            ch[17] = yaw_unwrapped_deg;
            ch[18] = kart_imu_get_yaw_rate_dps();
            ch[19] = f->target_v_pulse;
            ch[20] = kart_control_get_left_meas();
            ch[21] = (float)v->hue_pct;
            ch[22] = (float)v->dark_pct;
            ch[23] = (float)v->over_pct;
            ch[24] = (float)g_kart_cam_wb_ret;                        /* 0=白平衡已锁 */
            ch[25] = (float)g_kart_cam_fps;
        }
#else
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
        /* 【CH12 换过】原 yaw 零偏:标定完就是常数,一整条日志同一个值,占一路没意义。
         * 现改成图传占用标志 —— 它是漏拍的直接嫌疑人:阻塞整帧写发生在 exec_us
         * 计时窗之外,CH25/26 看不见,只能靠这一路和 CH27 对齐时间轴才能定位。
         * KART_AIMG_ENABLE=0 时恒 0(变量本体在 #if 之外定义,取值安全)。 */
        ch[12] = (float)g_kart_aimg_busy;                  /* 12 图传正在阻塞发送:跟随段应恒0 */
        ch[13] = kart_steer_get_target_yaw();              /* 13 目标航向 */
        /* 【CH14 换过】原转向 raw:CH16(角度)和 CH28(实测转角)已覆盖同一路信息。
         * 现改成视觉快照年龄(ms)。上一版日志缺这一路,导致无法区分
         * "识别不到目标" 和 "视觉根本没在更新、follow 一直吃旧快照" ——
         * 判读:相机 30FPS,正常应在 0~33ms 抖动;贴着 100ms 上限说明采集链断了。 */
        ch[14] = (float)kart_mission_get_s3_vision_age() * 10.0f; /* 14 视觉快照年龄(ms):>100=采集链断 */
        ch[15] = kart_steer_get_target_delta();            /* 15 目标转角 */
        ch[16] = (float)kart_steer_get_output();           /* 16 转向输出 */
        /* CH17/18/19 + CH9 就是【全局位姿】,科目二返回全靠它们,不用新增通道。
         * 2026-07-29 起门洞命令不再 kart_odom_reset(),故科目二全程 CH17/18 是
         * 【发车区坐标系】里的绝对位置:回到发车区时应回到 (0,0) 附近。
         * 【场地要量的两个数(阶段0)】
         *   ① 转 2 圈后 CH10(展开yaw)与 720° 之差 → 陀螺标度误差;
         *   ② 跑完一套随机动作后 CH17/18 与卷尺真值之差 → 位置误差总预算。
         *     <0.25m 稳;0.25~0.6m 靠加长门洞前直线引入段救;>1m 现有传感器救不回来
         *     (全车只有 IMU+编码器,没有任何能看见门洞的传感器)。 */
        ch[17] = kart_odom_get_x();                        /* 17 odom x(发车区系,米) */
        ch[18] = kart_odom_get_y();                        /* 18 odom y(发车区系,米) */
        ch[19] = kart_odom_get_dist();                     /* 19 odom 里程(标量,倒车也增) */
        /* CH20~23 是复用通道,含义随当前跑的分支变(省 4 个通道,不新增协议字段):
         *   方案B正向复现     : 20/21=投影当前x/y      22/23=瞄准点x/y
         *   科目三倒车 OLMode=0: 20=航向误差(度) 21=纠偏量(计数) 22=索引k 23=最终打角
         *   科目三倒车 OLMode=1: 20=航向误差(度) 21=【横向偏差e_lat(米)】22=索引k 23=最终打角
         * 调方案1的 Ke 就看 CH21:应被压向 0;若发散或换向震荡,先把 Ke 减半或取负。 */
        /* 【CH20/21 在科目三跟随段被占用】复用通道原本给方案B复现和科三倒车用,
         * 跟随段它们全程为 0。跟随段改成:20=色相合格像素占比,21=视觉帧序号。
         * 离开跟随段自动还给原语义,协议字段数和帧长都不变。 */
        if((MISSION_SUBJECT_3 == kart_mission_get_mode())
           && (S3_PHASE1_FOLLOW == kart_mission_get_subject3_stage()))
        {
            ch[20] = (float)kart_vision_get()->hue_pct;    /* 20 色相合格像素占比(%) */
            ch[21] = (float)kart_mission_get_s3_vision_seq(); /* 21 视觉帧序号:不涨=没出帧 */
        }
        else
        {
            ch[20] = kart_playback_get_cur_x();            /* 20 诊断:投影当前x / 航向误差 */
            ch[21] = kart_playback_get_cur_y();            /* 21 诊断:投影当前y / 纠偏量 / e_lat */
        }
        /* 【CH22/23 跟随段占用】22=core3 单帧识别耗时(ms),23=投递被拒次数。
         * 22 是这次搬核的直接验收量:它现在应该 300ms 上下,但【不再进 CH27】。
         * 23 涨得快说明相机出帧比 core3 算得快,那是正常的(丢帧优于卡环);
         * 23 完全不涨而 CH21 也不涨,说明 core3 根本没在服务,查 core3_main。 */
        if((MISSION_SUBJECT_3 == kart_mission_get_mode())
           && (S3_PHASE1_FOLLOW == kart_mission_get_subject3_stage()))
        {
            ch[22] = (float)kart_multicore_vision_last_us() * 0.001f; /* 22 core3 识别耗时(ms) */
            ch[23] = (float)kart_multicore_vision_reject();  /* 23 投递被拒累计 */
        }
        else
        {
            ch[22] = kart_playback_get_aim_x();            /* 22 诊断:瞄准点x / 索引k */
            ch[23] = kart_playback_get_aim_y();            /* 23 诊断:瞄准点y / 最终打角 */
        }
        ch[24] = kart_imu_get_dt_us();                     /* 24 IMU积分步长(us):稳定应≈5000 */
        ch[25] = (float)g_sched_last_exec_us;              /* 25 调度上拍分发耗时(us) */
        ch[26] = (float)g_sched_max_exec_us;               /* 26 调度历史最大耗时(us):应<5000 */
        ch[27] = (float)g_sched_overrun_count;             /* 27 漏周期累计:稳态应长期为0 */
        ch[28] = kart_steer_get_meas_delta();              /* 28 转向内环实测转角(center_delta):与CH15目标对比判内环跟踪/回中 */
        ch[29] = (float)kart_encoder_get_left_delta();     /* 29 左轮编码器原始delta(未滤波,脉冲/5ms):与CH5滤波值对比看滞后/抖动 */
        ch[30] = (float)kart_encoder_get_right_delta();    /* 30 右轮编码器原始delta(未滤波,脉冲/5ms):与CH6滤波值对比看滞后/抖动 */
        /* 【CH31/32 换过】原遥控油门/方向 raw:科三视觉源全程不看遥控,
         * 而且 CH2/CH13 已经能反映遥控是否介入。现改成逐像素曝光统计,
         * 用来把 "认不到黄板" 拆成三种病因(见 kart_vision.h 的字段注释):
         *   CH31 高 → 欠曝,像素过不了亮度门
         *   CH32 高 → 过曝,分量被压平过不了饱和度门
         *   两者都低而 CH20 仍为 0 → 曝光正常,是色相窗口或白平衡漂了
         * 三路都是纯诊断,不参与任何检测判据。 */
        {
            const kart_vision_result_t *vs = kart_vision_get();
            ch[31] = (float)vs->dark_pct;                  /* 31 欠曝像素占比(%) */
            ch[32] = (float)vs->over_pct;                  /* 32 过曝像素占比(%) */
        }
        ch[33] = kart_control_get_target_cmd();            /* 33 斜坡前的请求目标(上层写入):与CH4(斜坡后)对比即斜坡曲线,CH4追不上CH33就是在爬坡 */

        /* 34-38 摄像头采集链路(KART_CAMERA_ENABLE=0 时恒为 0/OFF)。
         * 判读方法:
         *   CH34 实测帧率:应≈配置的 FPS(默认60)。明显偏低=PCLK分频或曝光时间问题。
         *   CH35 状态:0=未启用 1=init失败 2=有init无VSYNC(查DVP排线/走线长度) 3=正常出帧。
         *   CH36 错位帧累计:稳态应长期不变。持续增长=PCLK走线太长或3.3V纹波,先查硬件别改代码。
         *   CH37 丢帧累计:上层处理来不及。只要不是持续暴涨就可接受,阶段0无消费者应为0。
         *   CH38 摄像头中断最长耗时(us):关键指标。DMA中断优先级70、VSYNC 62 都高于5ms控制PIT的50,
         *        会抢占控制环。此值 + CH26(调度最大耗时) 若逼近5000us,就必须调优先级或降分辨率/帧率。 */
        ch[34] = (float)g_kart_cam_fps;
        ch[35] = (float)kart_camera_state();
        /* 【CH36 换过】原错位帧累计:那是走线/电源的硬件指标,不随这次调参变。
         * 现改成固定白平衡下发结果。原因:kart_camera.c 在 init 后下发
         * scc8660_set_white_balance(),失败时【静默】沿用基础初始化 —— 也就是
         * 退回自动白平衡。而白平衡一动,黄色就会漂出色相窗口,整条识别链全废,
         * 表现和 "认不到板子" 完全一样。这一路必须能看见:0=已锁定,0xFF=没执行。 */
        ch[36] = (float)g_kart_cam_wb_ret;                 /* 36 白平衡锁定结果:0成功,255未执行 */
        ch[37] = (float)g_kart_cam_drop_count;
        ch[38] = (float)g_kart_cam_isr_max_us;

        /* 39-44 科目三跟随。两套语义二选一，由 KART_PERSON_LINK_ENABLE 切，
         * 这6路的分支切换不改变当前50通道总数。
         * 2026-08-12 新增第二套：TC4D7 人体视觉链路诊断。
         * 为何共用这6个通道：两套不可能同时在跑
         * （387 本地视觉靠 KART_VISION_ENABLE，它和 4D7 链路是两条路），
         * 而每加一通道都要改 VOFA 配置 + 帧长，上车时极容易忘。 */
#if KART_WIFI_ENABLE
        /* WiFi 联调临时诊断：不改菜单，直接用 VOFA CH39-CH44
         * 分开 SPI 版本读取、热点入网和 TCP 建链三层。
         * CH39 link:0未启用 1=SPI失败 2=WiFi失败 3=TCP失败 4=全通
         * CH40 init_ret；CH41 version_ok；CH42 INT高；CH43 IP字符串非空；CH44重试数。 */
        ch[39] = (float)g_kart_wifi_link;
        ch[40] = (float)g_kart_wifi_init_ret;
        ch[41] = ('\0' != wifi_spi_version[0]) ? 1.0f : 0.0f;
        ch[42] = (0 != gpio_get_level(WIFI_SPI_INT_PIN)) ? 1.0f : 0.0f;
        ch[43] = ('\0' != wifi_spi_ip_addr_port[0]) ? 1.0f : 0.0f;
        ch[44] = (float)g_kart_wifi_reconnect_cnt;
#elif KART_PERSON_LINK_ENABLE
        /* 【人体视觉链路诊断。上车先看 CH39，它能一下子分开四种毛病】
         *   CH39 link 0=一个字节都没收到(接线/共地/4D7 未发/中断未使能)
         *             1=有字节但从未成帧(波特率不对 / 帧头或 CRC 与对端不一致)
         *             2=曾经通过但现在失联(>200ms 无 VALID 帧：人丢了或链路断)
         *             3=在线
         *   CH40 frame_ok 累计好帧。四十五通道帧是 50Hz 发的，若 4D7 推理 20Hz，
         *        这个数应约 0.4/帧 地涨；阶段性停涨 = 4D7 那边卡了
         *   CH41 bad = crc_err + hdr_err + seq_stale + resync 之和。开机头几帧涨 1-2 正常
         *        (上电时刻可能切到帧中间)，持续涨就是真有问题。
         *        为何求和不分开：6 个通道不够，而定位时先只需知道"脏不脏"；
         *        真要细分拿调试器 watch kart_person_link_get_stat() 的七个字段
         *   CH42 height_norm 目标框高/图高 (0..1)。【用它标 KART_PLINK_NEAR_HEIGHT_STOP】
         *        把人站到想让车停住的距离，读此值，减 0.05 当 STOP，再减 0.07 当 RESUME
         *   CH43 bearing_deg >0=人在车右。【符号看它，但别用它标 HALF_FOV】
         *        符号：人站车右前方 → 此值应为正。反了就是人往右、车往左。
         *        【为何不能拿它标 KART_PLINK_HALF_FOV_DEG】本值 = err_norm × HALF_FOV，
         *        人走到画面边缘时 err_norm 恒为 1，所以 CH43 恒等于当前假设值
         *        —— 拿它标自己是个恒等式，标不出东西。
         *        正确做法是拿几何真值比：人站车前 X 米、横偏 Y 米，
         *        真实角度 = atan(Y/X)；修正系数 = 真实角度 / CH43 读数，
         *        把 HALF_FOV 乘上去。例：2m 处横偏 0.7m → 真值 19.3°，
         *        CH43 读到 15° → HALF_FOV 从 30 改成 30×19.3/15 ≈ 38.6
         *   CH44 flags 位集 bit0 VALID bit1 TRACKING bit2 FILTER_READY
         *        bit3 FAR bit4 MID bit5 NEAR。VOFA 看十进制：1=只检到 3=稳定跟 35=跟上且太近 */
        {
            const kart_vtrack_result_t *vt = kart_person_link_vtrack();
            kart_person_link_stat_t     st;
            kart_person_frame_t         pf;
            uint8                       got;
            float                       link;

            kart_person_link_get_stat(&st);
            got = kart_person_link_get_frame(&pf);

            if(0U == st.byte_count)                     { link = 0.0f; }
            else if(0U == st.frame_ok)                  { link = 1.0f; }
            else if(0U == kart_person_link_is_online()) { link = 2.0f; }
            else                                        { link = 3.0f; }

            ch[39] = link;
            ch[40] = (float)st.frame_ok;
            ch[41] = (float)(st.crc_err + st.hdr_err + st.seq_stale + st.resync);
            ch[42] = got ? ((float)pf.height * 0.001f) : 0.0f;
            ch[43] = rad_to_degree(vt->bearing_rad);
            ch[44] = got ? (float)pf.flags : 0.0f;
        }
#else
        /* 2026-08-11 从 WiFi 图传诊断切回来:图传方案暂停(CH39 恒为 1 = SPI 读不到
         * 模块固件版本号,数据通路不通),改回屏 + 菜单。WiFi 那 6 行赋值原样注释在
         * 下面,KART_WIFI_ENABLE 改回1时把两段对调即可,通道总数仍是50。
         *
         *   CH39 valid  1=本帧认到引导板      CH40 reject 0通过 1面积不足 2太窄 3宽高比 4填充率
         *   CH41 width_px 标定 f_px 用          CH42 dist_m 视觉反算距离
         *   CH43 bearing_deg >0=目标在右      CH44 follow state 0=IDLE 1=TRACKING 2=HOLD 3=LOST
         *
         * 【KART_VISION_ENABLE / KART_FOLLOW_ENABLE 都是 0 时这 6 条恒为 0】
         * 两个 get() 返回的是模块内部静态快照,禁用时也有定义、可安全取,只是不更新。 */
        {
            const kart_vision_result_t *v = kart_vision_get();
            const kart_follow_out_t    *f = kart_follow_get();

            ch[39] = (float)v->valid;
            ch[40] = (float)v->reject;
            ch[41] = (float)v->width_px;
            ch[42] = v->dist_m;
            ch[43] = f->bearing_deg;
            ch[44] = (float)f->state;
        }
#endif

        ch[45] = (float)kart_event_id;
        ch[46] = (float)kart_event_level;
        ch[47] = (float)kart_event_tick;
        {
            const kart_follow_out_t *fo = kart_follow_get();
            ch[48] = fo->target_v_pulse;
            ch[49] = fo->target_delta;
        }
        ch[50] = (float)kart_vision_get()->confidence;
#endif
        kart_log_send_justfloat(ch, KART_LOG_CHANNELS);
    }
    kart_log_sequence++;
}
