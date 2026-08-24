/*********************************************************************************************************************
 * 文件名称  kart_preprocess
 * 功能说明  图像预处理模块实现
 ********************************************************************************************************************/
#include "kart_preprocess.h"
#include <string.h>

#pragma section all "cpu0_dsram"

/* ======================== 内部缓冲区 ======================== */

#if PREPROCESS_ENABLE

/* 预处理后的图像缓冲（160×120 RGB565 = 38400字节）
 * 注意：若启用ROI裁剪，实际只用到下半部分，但仍分配全图以简化索引 */
static uint16 preprocess_buf[PREPROCESS_HEIGHT][PREPROCESS_WIDTH];

/* 统计量 */
static kart_preprocess_stat_t preprocess_stat;

#if PREPROCESS_HIST_EQ_ENABLE
/* 直方图均衡查找表：R5/G6/B5 各自的映射表 */
static uint8 hist_lut_r[32];    /* R: 0-31 */
static uint8 hist_lut_g[64];    /* G: 0-63 */
static uint8 hist_lut_b[32];    /* B: 0-31 */
#endif

/* ======================== 辅助函数 ======================== */

#if PREPROCESS_DENOISE_ENABLE && PREPROCESS_DENOISE_MEDIAN
/* 3x3 中值滤波：对9个像素排序取中值（针对RGB565整体） */
static uint16 median_filter_3x3(const uint16 *img, int16 w, int16 h, int16 x, int16 y)
{
    uint16 buf[9];
    uint8 cnt = 0;
    int16 i, j;

    /* 收集3×3窗口内的像素（边界处窗口缩小） */
    for(j = -1; j <= 1; j++)
    {
        for(i = -1; i <= 1; i++)
        {
            int16 nx = x + i;
            int16 ny = y + j;

            if((nx >= 0) && (nx < w) && (ny >= 0) && (ny < h))
            {
                buf[cnt++] = img[(int32)ny * (int32)w + (int32)nx];
            }
        }
    }

    if(cnt == 0) return 0;

    /* 冒泡排序取中值（9个元素，足够快） */
    for(i = 0; i < cnt - 1; i++)
    {
        for(j = i + 1; j < cnt; j++)
        {
            if(buf[i] > buf[j])
            {
                uint16 tmp = buf[i];
                buf[i] = buf[j];
                buf[j] = tmp;
            }
        }
    }

    return buf[cnt / 2];
}
#endif

#if PREPROCESS_DENOISE_ENABLE && (!PREPROCESS_DENOISE_MEDIAN)
/* 3x3 均值滤波（RGB565各通道分别平均） */
static uint16 mean_filter_3x3(const uint16 *img, int16 w, int16 h, int16 x, int16 y)
{
    uint32 sum_r = 0;
    uint32 sum_g = 0;
    uint32 sum_b = 0;
    uint8 cnt = 0;
    int16 i, j;

    for(j = -1; j <= 1; j++)
    {
        for(i = -1; i <= 1; i++)
        {
            int16 nx = x + i;
            int16 ny = y + j;

            if((nx >= 0) && (nx < w) && (ny >= 0) && (ny < h))
            {
                uint16 pix = img[(int32)ny * (int32)w + (int32)nx];
                sum_r += (pix >> 11) & 0x1FU;
                sum_g += (pix >> 5) & 0x3FU;
                sum_b += pix & 0x1FU;
                cnt++;
            }
        }
    }

    if(cnt == 0) return 0;

    uint8 avg_r = (uint8)(sum_r / cnt);
    uint8 avg_g = (uint8)(sum_g / cnt);
    uint8 avg_b = (uint8)(sum_b / cnt);

    return (uint16)(((uint16)avg_r << 11) | ((uint16)avg_g << 5) | avg_b);
}
#endif

#if PREPROCESS_HIST_EQ_ENABLE
/* 直方图均衡：对RGB565各通道分别计算累积分布函数并建立映射表 */
static void hist_equalize_build_lut(const uint16 *img, int16 w, int16 h,
                                     int16 y_start, int16 y_end)
{
    uint32 hist_r[32] = {0};
    uint32 hist_g[64] = {0};
    uint32 hist_b[32] = {0};
    uint32 cdf_r[32];
    uint32 cdf_g[64];
    uint32 cdf_b[32];
    int16 x, y;
    uint32 total = 0;
    uint16 i;

    /* 统计直方图 */
    for(y = y_start; y < y_end; y++)
    {
        const uint16 *row = &img[(int32)y * (int32)w];
        for(x = 0; x < w; x++)
        {
            uint16 pix = row[x];
            uint8 r5 = (uint8)((pix >> 11) & 0x1FU);
            uint8 g6 = (uint8)((pix >> 5) & 0x3FU);
            uint8 b5 = (uint8)(pix & 0x1FU);

            hist_r[r5]++;
            hist_g[g6]++;
            hist_b[b5]++;
            total++;
        }
    }

    if(total == 0) return;

    /* 计算累积分布函数 CDF */
    cdf_r[0] = hist_r[0];
    for(i = 1; i < 32; i++) cdf_r[i] = cdf_r[i - 1] + hist_r[i];

    cdf_g[0] = hist_g[0];
    for(i = 1; i < 64; i++) cdf_g[i] = cdf_g[i - 1] + hist_g[i];

    cdf_b[0] = hist_b[0];
    for(i = 1; i < 32; i++) cdf_b[i] = cdf_b[i - 1] + hist_b[i];

    /* 建立映射表：new_val = (cdf[val] - cdf_min) * (max_val - 1) / (total - cdf_min) */
    uint32 cdf_min_r = cdf_r[0];
    uint32 cdf_min_g = cdf_g[0];
    uint32 cdf_min_b = cdf_b[0];

    for(i = 0; i < 32; i++)
    {
        hist_lut_r[i] = (uint8)(((cdf_r[i] - cdf_min_r) * 31U) / (total - cdf_min_r));
        hist_lut_b[i] = (uint8)(((cdf_b[i] - cdf_min_b) * 31U) / (total - cdf_min_b));
    }

    for(i = 0; i < 64; i++)
    {
        hist_lut_g[i] = (uint8)(((cdf_g[i] - cdf_min_g) * 63U) / (total - cdf_min_g));
    }
}

/* 应用直方图均衡映射 */
static uint16 hist_equalize_apply(uint16 pix)
{
    uint8 r5 = (uint8)((pix >> 11) & 0x1FU);
    uint8 g6 = (uint8)((pix >> 5) & 0x3FU);
    uint8 b5 = (uint8)(pix & 0x1FU);

    uint8 r_new = hist_lut_r[r5];
    uint8 g_new = hist_lut_g[g6];
    uint8 b_new = hist_lut_b[b5];

    return (uint16)(((uint16)r_new << 11) | ((uint16)g_new << 5) | b_new);
}
#endif

#endif  /* PREPROCESS_ENABLE */

/* ======================== 对外接口实现 ======================== */

void kart_preprocess_init(void)
{
#if PREPROCESS_ENABLE
    memset(&preprocess_stat, 0, sizeof(preprocess_stat));
    memset(preprocess_buf, 0, sizeof(preprocess_buf));

#if PREPROCESS_HIST_EQ_ENABLE
    /* 初始化映射表为恒等映射 */
    uint16 i;
    for(i = 0; i < 32; i++)
    {
        hist_lut_r[i] = (uint8)i;
        hist_lut_b[i] = (uint8)i;
    }
    for(i = 0; i < 64; i++)
    {
        hist_lut_g[i] = (uint8)i;
    }
#endif
#endif
}

const uint16 *kart_preprocess_frame(const uint16 *img, int16 w, int16 h)
{
#if PREPROCESS_ENABLE
    uint32 t0 = system_getval();
    int16 y_start = 0;
    int16 y_end = h;
    int16 x, y;

    if((img == NULL) ||
       (w != PREPROCESS_WIDTH) ||
       (h != PREPROCESS_HEIGHT))
    {
        return img;
    }

#if PREPROCESS_ROI_ENABLE
    /* ROI裁剪：只处理 [ROI_Y_START, h) 范围 */
    y_start = PREPROCESS_ROI_Y_START;
    if(y_start >= h) y_start = 0;   /* 防止配置错误 */
#endif

    /* ROI 上方清黑：识别算法仍扫完整 160x120，不清黑就只是“不滤波”，
     * 上方的黄色干扰仍然会被当成目标。 */
    if(y_start > 0)
    {
        memset(preprocess_buf, 0, (size_t)y_start * (size_t)w * sizeof(uint16));
    }

#if PREPROCESS_HIST_EQ_ENABLE
    /* 直方图均衡：先建立映射表（需要扫描一遍ROI） */
    hist_equalize_build_lut(img, w, h, y_start, y_end);
#endif

    /* 主处理循环：ROI区域应用滤波 + 均衡 */
    for(y = y_start; y < y_end; y++)
    {
        for(x = 0; x < w; x++)
        {
            uint16 pix = img[(int32)y * (int32)w + (int32)x];

#if PREPROCESS_DENOISE_ENABLE
    #if PREPROCESS_DENOISE_MEDIAN
            pix = median_filter_3x3(img, w, h, x, y);
    #else
            pix = mean_filter_3x3(img, w, h, x, y);
    #endif
#endif

#if PREPROCESS_HIST_EQ_ENABLE
            pix = hist_equalize_apply(pix);
#endif

            preprocess_buf[y][x] = pix;
        }
    }

    /* 更新统计 */
    uint32 t1 = system_getval();
    uint32 dt_us = (t1 - t0) / 100u;

    preprocess_stat.last_us = dt_us;
    if(dt_us > preprocess_stat.max_us)
    {
        preprocess_stat.max_us = dt_us;
    }
    preprocess_stat.count++;

    return (const uint16 *)preprocess_buf;
#else
    /* 预处理关闭：直接返回原图 */
    (void)w;
    (void)h;
    return img;
#endif
}

const kart_preprocess_stat_t *kart_preprocess_get_stat(void)
{
#if PREPROCESS_ENABLE
    return &preprocess_stat;
#else
    static const kart_preprocess_stat_t dummy = {0};
    return &dummy;
#endif
}

void kart_preprocess_reset_stat(void)
{
#if PREPROCESS_ENABLE
    memset(&preprocess_stat, 0, sizeof(preprocess_stat));
#endif
}

#pragma section all restore
