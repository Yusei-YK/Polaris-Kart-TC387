/*********************************************************************************************************************
 * 文件名称  kart_vision_calibrate
 * 功能说明  视觉参数标定辅助工具实现
 *
 * 【本模块目前是孤立的】.h 没有任何文件 #include,update/reset/print_report
 * 也没有任何调用点。CALIB_MODE 默认 CALIB_DISABLED,整份实现编译成文件末尾
 * 那几个空壳。想用它得先照 .h 开头"第 0 步"把调用接进视觉链路。
 ********************************************************************************************************************/
#include "kart_vision_calibrate.h"
#include <stdio.h>

#if (CALIB_MODE != CALIB_DISABLED)

/* 全局统计量 */
static kart_calib_stat_t g_calib_stat = {0};

/* 颜色验证模式需要缓存图像指针 */
#if (CALIB_MODE == CALIB_COLOR)
const uint16 *g_calib_img_ptr = NULL;
int16 g_calib_img_w = 0;
#endif

/* 焦距标定用的滑动窗口。定长 50 帧的环形缓冲,每帧覆盖最老的一格,
 * 取的是窗口内非零项的算术平均,不是中位数。
 * 【别拿它测太远的板子】宽度门 VISION_MIN_WIDTH=10 决定了可标距离上限是 2.76 m,
 * 再远 result->valid 恒为 0,窗口一格都填不上。 */
#if (CALIB_MODE == CALIB_FOCAL)
static float g_width_window[CALIB_WINDOW_SIZE] = {0};
static uint32 g_window_idx = 0;
#endif

void kart_calib_init(void)
{
    g_calib_stat.total_frames = 0;
    g_calib_stat.valid_frames = 0;
    g_calib_stat.reject_area = 0;
    g_calib_stat.reject_width = 0;
    g_calib_stat.reject_aspect = 0;
    g_calib_stat.reject_fill = 0;
    g_calib_stat.width_sum = 0.0f;
    g_calib_stat.width_count = 0;
    g_calib_stat.width_avg = 0.0f;
    g_calib_stat.focal_estimate = 0.0f;

#if (CALIB_MODE == CALIB_FOCAL)
    g_window_idx = 0;
    for(uint32 i = 0; i < CALIB_WINDOW_SIZE; i++)
    {
        g_width_window[i] = 0.0f;
    }
#endif
}

void kart_calib_update(const uint16 *img, int16 w, int16 h, const kart_vision_result_t *result)
{
    if(result == NULL)
    {
        return;
    }

#if (CALIB_MODE == CALIB_COLOR)
    /* 缓存图像指针供VOFA宏使用 */
    g_calib_img_ptr = img;
    g_calib_img_w = w;
#else
    (void)img;
    (void)w;
    (void)h;
#endif

    /* 统计帧数 */
    g_calib_stat.total_frames++;

    if(result->valid)
    {
        g_calib_stat.valid_frames++;

#if (CALIB_MODE == CALIB_FOCAL)
        /* 焦距标定: 累加像素宽度 */
        if(result->width_px > 0)
        {
            /* 滑动窗口平滑 */
            g_width_window[g_window_idx] = (float)result->width_px;
            g_window_idx = (g_window_idx + 1) % CALIB_WINDOW_SIZE;

            /* 计算窗口平均值 */
            float sum = 0.0f;
            uint32 count = 0;
            for(uint32 i = 0; i < CALIB_WINDOW_SIZE; i++)
            {
                if(g_width_window[i] > 0.1f)  /* 跳过未填充的位置 */
                {
                    sum += g_width_window[i];
                    count++;
                }
            }

            if(count > 0)
            {
                g_calib_stat.width_avg = sum / (float)count;
                /* f_px = (width_px × dist_m) / board_w_m
                 * 【按板宽解,且没有做离轴校正】kart_vision.c 测距那行乘了
                 * lens_range_scale = theta/sin(theta) 来补 130 度鱼眼的
                 * 边缘压缩,这里没有。所以标定时板子必须放在画面横向正中,
                 * 偏到两侧解出来的 f_px 会系统性偏大。 */
                g_calib_stat.focal_estimate =
                    (g_calib_stat.width_avg * CALIB_DIST_M) / VISION_BOARD_W_M;
            }

            g_calib_stat.width_sum += (float)result->width_px;
            g_calib_stat.width_count++;
        }
#endif

#if (CALIB_MODE == CALIB_STATS)
        /* 统计模式: 累加宽度用于计算平均值 */
        if(result->width_px > 0)
        {
            g_calib_stat.width_sum += (float)result->width_px;
            g_calib_stat.width_count++;
            g_calib_stat.width_avg = g_calib_stat.width_sum / (float)g_calib_stat.width_count;
        }
#endif
    }
    else
    {
        /* 统计拒绝原因。
         * 【这四格不是四个独立原因】REJ_AREA 是 kart_vision_process 的
         * 缺省值,连"图像指针为空""图比掩膜大"都算进面积一格;REJ_ASPECT
         * 则同时是真宽高比越界和连续性门跳目标两件事(它们共用一个取值)。
         * 报告里的百分比按这个口径读,别当成门限调参的依据。
         * default 分支吞不掉任何东西:每条失败路径都写了明确的 reject 值。 */
        switch(result->reject)
        {
            case VISION_REJ_AREA:
                g_calib_stat.reject_area++;
                break;
            case VISION_REJ_WIDTH:
                g_calib_stat.reject_width++;
                break;
            case VISION_REJ_ASPECT:
                g_calib_stat.reject_aspect++;
                break;
            case VISION_REJ_FILL:
                g_calib_stat.reject_fill++;
                break;
            default:
                break;
        }
    }
}

const kart_calib_stat_t *kart_calib_get_stat(void)
{
    return &g_calib_stat;
}

void kart_calib_reset(void)
{
    kart_calib_init();
}

void kart_calib_print_report(void)
{
    const kart_calib_stat_t *s = &g_calib_stat;

    printf("\n========== 视觉标定报告 ==========\n");
    printf("总帧数: %u\n", (unsigned int)s->total_frames);
    printf("有效帧数: %u (%.1f%%)\n",
        (unsigned int)s->valid_frames,
        (s->total_frames > 0) ? (100.0f * s->valid_frames / s->total_frames) : 0.0f);

    printf("\n拒绝原因分布:\n");
    printf("  面积不足: %u (%.1f%%)\n",
        (unsigned int)s->reject_area,
        (s->total_frames > 0) ? (100.0f * s->reject_area / s->total_frames) : 0.0f);
    printf("  宽度不足: %u (%.1f%%)\n",
        (unsigned int)s->reject_width,
        (s->total_frames > 0) ? (100.0f * s->reject_width / s->total_frames) : 0.0f);
    printf("  宽高比: %u (%.1f%%)\n",
        (unsigned int)s->reject_aspect,
        (s->total_frames > 0) ? (100.0f * s->reject_aspect / s->total_frames) : 0.0f);
    printf("  填充率: %u (%.1f%%)\n",
        (unsigned int)s->reject_fill,
        (s->total_frames > 0) ? (100.0f * s->reject_fill / s->total_frames) : 0.0f);

#if (CALIB_MODE == CALIB_FOCAL)
    printf("\n焦距标定结果:\n");
    printf("  标定距离: %.2f m\n", CALIB_DIST_M);
    printf("  板子宽度: %.2f m\n", VISION_BOARD_W_M);
    printf("  平均像素宽度: %.1f px\n", s->width_avg);
    printf("  估算焦距: %.1f px\n", s->focal_estimate);
    printf("\n下一步操作:\n");
    printf("  1. 在5个距离点(1.0/1.5/2.0/2.5/3.0m)重复标定\n");
    printf("  2. 取5个 focal_estimate 的平均值\n");
    printf("  3. 更新 kart_vision.h 中的 VISION_FPX\n");
#endif

#if (CALIB_MODE == CALIB_STATS)
    printf("\n识别统计:\n");
    printf("  平均检测宽度: %.1f px\n", s->width_avg);
    printf("  识别率: %.1f%%\n",
        (s->total_frames > 0) ? (100.0f * s->valid_frames / s->total_frames) : 0.0f);
#endif

    printf("==================================\n\n");
}

#else

/* 关闭模式: 空壳实现 */
void kart_calib_init(void) {}
void kart_calib_update(const uint16 *img, int16 w, int16 h, const kart_vision_result_t *result)
{
    (void)img; (void)w; (void)h; (void)result;
}
const kart_calib_stat_t *kart_calib_get_stat(void) { return NULL; }
void kart_calib_reset(void) {}
void kart_calib_print_report(void) {}

#endif  /* CALIB_MODE != CALIB_DISABLED */
