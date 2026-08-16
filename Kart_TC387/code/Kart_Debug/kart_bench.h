/*********************************************************************************************************************
 * 文件名称  kart_bench
 * 所属分层  Kart_Debug（只读观测层：不参与任何控制决策）
 * 功能设计  Kart_TC387 实时性能基准测试矩阵：12 项基准（B1-B12），单帧耗时 + 统计量
 *
 * 【为什么必须有它：先有基准，才能判断"可行"】
 *   架构讨论中反复出现"2-3ms可行""单趟扫描几十us"这类未经验证的性能数字。
 *   Kart_TC387 有硬件 FPU + DSP，但 300MHz 主频在嵌入式里不算高；160×120 图像处理
 *   + 多特征 LK 光流 + 姿态解算 + 双 PI 闭环，每个模块单独看都"应该不慢"，
 *   叠起来是否真的能在 5ms 控制窗口内跑完，必须实测，不能靠估。
 *
 * 【基准矩阵设计原则】
 *   1) 分层覆盖：从最底层（RGB→灰度、memcpy）到最顶层（end-to-end 单帧）；
 *   2) 独立可测：每个基准可单独运行，不依赖硬件状态（用假图/假数据）；
 *   3) 统计完备：不只要均值，还要最大值（worst-case）、标准差、P95/P99；
 *   4) 在线观测：通过 VOFA 示波器通道实时看，与 bearing/scale 在同一时间轴对齐。
 *
 * 【12 项基准（B1-B12）】
 *   B1:  RGB565 → 灰度（160×120 单帧，19200 像素）
 *   B2:  金字塔下采样（L0 → L1，双线性 2×2→1，4800 像素写）
 *   B3:  单点双线性插值 × 1000（LK 内层循环热点）
 *   B4:  Shi-Tomasi 角点响应（3×3 窗口，单点）
 *   B5:  LK 单层单点迭代（11×11 窗口，20 次迭代上限）
 *   B6:  LK 金字塔单点（2 层，forward + backward = 双向）
 *   B7:  32 点 LK 金字塔（模拟 kart_vtrack 满载）
 *   B8:  Detector 单帧（kart_vision_process，160×120 单趟扫描 + 统计）
 *   B9:  Tracker 单帧（kart_vtrack_update，含金字塔 + 32 点 LK + MAD + scale）
 *   B10: 姿态解算单拍（Madgwick 6DOF，200Hz 典型）
 *   B11: 控制环单拍（双 PI：速度环 + 转向串级环，5ms 典型）
 *   B12: End-to-end 单帧（B9 + B10 + B11，最坏情况：视觉帧到达时刻恰好在控制拍开头）
 *
 * 【通过/不通过判据】
 *   - B9（Tracker 单帧）< 8ms：留 2ms 给 Detector、控制环、日志排空
 *   - B11（控制环）< 1ms：g_sched_max_exec_us 已在跑，这里独立测一遍确认
 *   - B12（End-to-end）< 10ms：最坏情况不能超过两个 5ms 拍，否则必漏拍
 *   以上任一超标，整套方案"在 Kart_TC387 上实时可行"这个前提就不成立，必须换方案。
 *
 * 【如何用】
 *   1) 首次接硬件：在 main loop 里调 bench_run_once()，跑一遍 B1-B12，
 *      结果通过 kart_wifi 示波器通道或 VOFA 串口看，记录到文档。
 *   2) 优化后复测：改了算法（如 LK 窗口大小、金字塔层数、MAX_POINTS）后重跑，
 *      对比前后耗时，判断优化是否有效、是否引入退化。
 *   3) 持续监控：把 B9（Tracker）和 B12（End-to-end）加到示波器通道，
 *      跑车时实时看，确认"跑起来和静态测一样快"（排除 cache miss、
 *      中断抢占等只有真实负载下才出现的问题）。
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-08-10        Kart                首版：12 项基准矩阵，未接硬件
 ********************************************************************************************************************/

#ifndef KART_BENCH_H_
#define KART_BENCH_H_
#include "zf_common_headfile.h"

/* 总开关 */
#ifndef BENCH_ENABLE
#define BENCH_ENABLE               (0)
#endif

/*=========================== 基准项枚举 ===========================*/
typedef enum
{
    KART_BENCH_B1_RGB_TO_GRAY = 0,      /* RGB565 → 灰度，160×120 */
    KART_BENCH_B2_PYRAMID_L1,           /* 金字塔下采样 L0→L1 */
    KART_BENCH_B3_INTERP_1K,            /* 双线性插值 × 1000 */
    KART_BENCH_B4_CORNER_RESP,          /* Shi-Tomasi 角点响应，单点 */
    KART_BENCH_B5_LK_ONE_LEVEL,         /* LK 单层单点 */
    KART_BENCH_B6_LK_PYRAMID_1PT,       /* LK 金字塔单点（双向）*/
    KART_BENCH_B7_LK_PYRAMID_32PT,      /* 32 点 LK 金字塔 */
    KART_BENCH_B8_DETECTOR,             /* Detector 单帧 */
    KART_BENCH_B9_TRACKER,              /* Tracker 单帧 */
    KART_BENCH_B10_IMU,                 /* 姿态解算单拍 */
    KART_BENCH_B11_CONTROL,             /* 控制环单拍 */
    KART_BENCH_B12_END_TO_END,          /* End-to-end 单帧 */
    KART_BENCH_COUNT
}kart_bench_item_enum;

/*=========================== 统计量 ===========================*/
typedef struct
{
    uint32 count;           /* 采样次数 */
    uint32 sum_us;          /* 累积耗时(us)，用于算均值 */
    uint32 min_us;          /* 最小值(us) */
    uint32 max_us;          /* 最大值(us) */
    uint32 last_us;         /* 最近一次(us) */
    float  mean_us;         /* 均值(us)，每 N 次刷新 */
    float  std_dev_us;      /* 标准差(us)，每 N 次刷新 */
}kart_bench_stat_t;

/*=========================== 对外接口 ===========================*/

/* 初始化。清零统计量，分配测试用假数据（cpu0_dsram）。
 * 必须在 kart_vtrack_init() 之后调用（B7/B9 会调 kart_vtrack 内部函数）。 */
void kart_bench_init (void);

/* 运行单项基准。item 指定 B1-B12 中的一项，返回本次耗时(us)。
 * 纯计算、不阻塞、不改全局控制状态（用的是内部假数据）。
 * 内部同时更新该项的统计量（count/sum/min/max/mean/std_dev）。 */
uint32 kart_bench_run_one (kart_bench_item_enum item);

/* 运行所有基准（B1-B12），每项跑 1 次。
 * 用于首次接硬件时摸底、或优化后复测。耗时约 50-100ms，只能在 IDLE 页或初始化阶段调。 */
void kart_bench_run_all (void);

/* 取某项基准的统计量（只读）。供 VOFA 日志、示波器通道、菜单页读取。 */
const kart_bench_stat_t *kart_bench_get_stat (kart_bench_item_enum item);

/* 清零某项基准的统计量。用于"改了参数、重新开始采样"。 */
void kart_bench_reset_stat (kart_bench_item_enum item);

/* 清零所有基准的统计量 */
void kart_bench_reset_all (void);

/* 通过 kart_wifi 示波器通道输出指定基准的最近一次耗时。
 * ch_base 指定起始通道号（0-7），依次输出 B9/B11/B12 三项（各占 1 通道）。
 * 这三项是最关键的 go/no-go 指标，实时看它们是否超标。
 * 【注意】必须先 #include "kart_wifi.h" 且 WIFI_ENABLE=1。 */
void kart_bench_publish_to_wifi_osc (uint8 ch_base);

#endif
