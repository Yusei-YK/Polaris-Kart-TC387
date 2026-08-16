/*********************************************************************************************************************
 * 文件名称  kart_bench
 * 功能说明  Kart_TC387 实时性能基准测试实现。设计说明见 kart_bench.h，此处只写实现骨架。
 *
 * 【实现状态：骨架 + 关键接口，具体测试代码待硬件验证时填充】
 *   首版只提供初始化、统计量管理、接口框架。B1-B12 的具体测试代码留作 TODO，
 *   理由：1) 无硬件无法验证；2) 部分测试需要调用 kart_vtrack 内部 static 函数，
 *   需要先把那些函数改 extern 或提供测试桩；3) token 预算。
 *   接硬件时按需填充，优先填 B9（Tracker）、B11（控制环）、B12（End-to-end）。
 ********************************************************************************************************************/
#include "kart_bench.h"
#include "kart_vtrack.h"
#include "kart_vision.h"
#include "kart_wifi.h"       /* kart_wifi_osc_set, WIFI_ENABLE */
#include <math.h>
#include <string.h>

#pragma section all "cpu0_dsram"

/*=========================== 统计量数组 ===========================*/
static kart_bench_stat_t bench_stats[KART_BENCH_COUNT];

/*=========================== 测试用假数据 ===========================*/
/* 160x120 RGB565 假图，填充棋盘格或渐变，供 B1/B8/B9 用。
 * BENCH_ENABLE=0 时不实例化：这 38400 字节在 cpu0 DSRAM，
 * 要与 scc8660_image、preprocess_buf、kart_vtrack 金字塔、kart_vision 掩膜
 * 抢同一块 240K，白占会让链接器报 ltc E112。 */
#if BENCH_ENABLE
static uint16 fake_img_rgb565[120][160];
#endif

/* 初始化假数据：简单棋盘格，便于目视验证算法跑了 */
static void kart_bench_init_fake_data(void)
{
#if BENCH_ENABLE
    uint16 y, x;
    for(y = 0; y < 120; y++)
    {
        for(x = 0; x < 160; x++)
        {
            /* 棋盘格：8x8 块，黑白交替 */
            uint8 bx = (uint8)(x / 8);
            uint8 by = (uint8)(y / 8);
            uint16 color = (uint16)(((bx + by) & 1) ? 0xFFFFU : 0x0000U);
            fake_img_rgb565[y][x] = color;
        }
    }
#endif
}

void kart_bench_init(void)
{
    memset(bench_stats, 0, sizeof(bench_stats));

    uint16 i;
    for(i = 0; i < KART_BENCH_COUNT; i++)
    {
        bench_stats[i].min_us = 0xFFFFFFFFU;    /* 初始化为最大值 */
    }

    kart_bench_init_fake_data();
}

/*-------------------------------------------------------------------------------------------------------------------
 * 更新统计量：每次 run_one 后调用
 *-----------------------------------------------------------------------------------------------------------------*/
static void kart_bench_update_stat(kart_bench_item_enum item, uint32 dt_us)
{
    kart_bench_stat_t *s = &bench_stats[item];

    s->count++;
    s->sum_us += dt_us;
    s->last_us = dt_us;

    if(dt_us < s->min_us)
    {
        s->min_us = dt_us;
    }
    if(dt_us > s->max_us)
    {
        s->max_us = dt_us;
    }

    /* 每 100 次刷新一次均值（避免每次除法）*/
    if((s->count % 100) == 0)
    {
        s->mean_us = (float)s->sum_us / (float)s->count;
        /* 标准差计算需要二阶矩，简化实现暂不做，留作 TODO */
        s->std_dev_us = 0.0f;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 运行单项基准
 *
 * 【TODO】B1-B12 的具体实现待硬件验证时填充。当前返回 0，统计量不会更新。
 *-----------------------------------------------------------------------------------------------------------------*/
uint32 kart_bench_run_one(kart_bench_item_enum item)
{
    uint32 t0 = system_getval();
    uint32 t1;
    uint32 dt_us;

    switch(item)
    {
        case KART_BENCH_B1_RGB_TO_GRAY:
        {
            /* TODO: 调用 kart_vtrack 的 RGB→灰度转换，或复制其代码片段 */
            break;
        }

        case KART_BENCH_B2_PYRAMID_L1:
        {
            /* TODO: 调用金字塔下采样 */
            break;
        }

        case KART_BENCH_B3_INTERP_1K:
        {
            /* TODO: 双线性插值 × 1000 */
            break;
        }

        case KART_BENCH_B4_CORNER_RESP:
        {
            /* TODO: Shi-Tomasi 单点 */
            break;
        }

        case KART_BENCH_B5_LK_ONE_LEVEL:
        {
            /* TODO: LK 单层单点 */
            break;
        }

        case KART_BENCH_B6_LK_PYRAMID_1PT:
        {
            /* TODO: LK 金字塔单点（双向）*/
            break;
        }

        case KART_BENCH_B7_LK_PYRAMID_32PT:
        {
            /* TODO: 32 点 LK 金字塔 */
            break;
        }

        case KART_BENCH_B8_DETECTOR:
        {
#if BENCH_ENABLE
            (void)kart_vision_process((const uint16 *)fake_img_rgb565, 160, 120);
#endif
            break;
        }

        case KART_BENCH_B9_TRACKER:
        {
#if BENCH_ENABLE
            (void)kart_vtrack_update((const uint16 *)fake_img_rgb565, 160, 120);
#endif
            break;
        }

        case KART_BENCH_B10_IMU:
        {
            /* TODO: 调用 kart_imu 的 Madgwick 单拍 */
            break;
        }

        case KART_BENCH_B11_CONTROL:
        {
            /* B11: 速度环+转向串级单拍。测控制算法本身耗时(输入来自实时传感器状态)。*/
            kart_control_speed_update();
            kart_steer_ctrl_update();
            break;
        }

        case KART_BENCH_B12_END_TO_END:
        {
#if BENCH_ENABLE
            (void)kart_vtrack_update((const uint16 *)fake_img_rgb565, 160, 120);
#endif
            kart_control_speed_update();
            kart_steer_ctrl_update();
            break;
        }

        default:
            return 0;
    }

    t1 = system_getval();

    /* 原始计数差分再换 us，避开 system_getval_us() 先除再差的回绕坑 */
    dt_us = (t1 - t0) / 100u;

    kart_bench_update_stat(item, dt_us);

    return dt_us;
}

void kart_bench_run_all(void)
{
    uint16 i;
    for(i = 0; i < KART_BENCH_COUNT; i++)
    {
        (void)kart_bench_run_one((kart_bench_item_enum)i);
    }
}

const kart_bench_stat_t *kart_bench_get_stat(kart_bench_item_enum item)
{
    if(item < KART_BENCH_COUNT)
    {
        return &bench_stats[item];
    }
    return NULL;
}

void kart_bench_reset_stat(kart_bench_item_enum item)
{
    if(item < KART_BENCH_COUNT)
    {
        memset(&bench_stats[item], 0, sizeof(kart_bench_stat_t));
        bench_stats[item].min_us = 0xFFFFFFFFU;
    }
}

void kart_bench_reset_all(void)
{
    uint16 i;
    for(i = 0; i < KART_BENCH_COUNT; i++)
    {
        kart_bench_reset_stat((kart_bench_item_enum)i);
    }
}

void kart_bench_publish_to_wifi_osc(uint8 ch_base)
{
#if WIFI_ENABLE
    /* 输出三项关键指标：B9（Tracker）、B11（控制环）、B12（End-to-end）*/
    if(ch_base + 2 < 8)     /* 确保不超出 8 通道 */
    {
        kart_wifi_osc_set(ch_base + 0, (float)bench_stats[KART_BENCH_B9_TRACKER].last_us);
        kart_wifi_osc_set(ch_base + 1, (float)bench_stats[KART_BENCH_B11_CONTROL].last_us);
        kart_wifi_osc_set(ch_base + 2, (float)bench_stats[KART_BENCH_B12_END_TO_END].last_us);
    }
#else
    (void)ch_base;      /* 图传关闭时避免 unused warning */
#endif
}

#pragma section all restore
