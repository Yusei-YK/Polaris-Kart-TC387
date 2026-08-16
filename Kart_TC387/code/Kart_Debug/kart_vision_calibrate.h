/*********************************************************************************************************************
 * 文件名称  kart_vision_calibrate
 * 功能说明  视觉参数标定辅助工具 - 用于实车标定焦距和验证颜色判别式
 *
 * 设计目标:
 *   1. 焦距标定: 通过已知距离/板宽反推 f_px
 *   2. 颜色验证: 实时输出判别中间量,调试阈值
 *   3. 检测统计: 记录识别成功率、误检情况
 *   4. VOFA输出: 格式化输出便于波形查看
 *
 * 使用流程:
 *   [焦距标定]
 *   1. 设置 CALIB_MODE = CALIB_FOCAL
 *   2. 板子放在已知距离(1.0m/1.5m/2.0m/2.5m/3.0m)
 *   3. 静止10秒,VOFA记录 width_px
 *   4. 取中位值,用公式 f_px = (width_px × dist_m) / BOARD_W_M
 *   5. 重复5个距离点,取平均值更新 VISION_FPX
 *
 *   [颜色验证]
 *   1. 设置 CALIB_MODE = CALIB_COLOR
 *   2. 板子放在视野中央
 *   3. VOFA查看 r5/g5/b5/m_lhs/m_rhs,确认判别式是否通过
 *   4. 在3种光照下重复(室内日光/暖光/窗边自然光)
 *
 *   [识别统计]
 *   1. 设置 CALIB_MODE = CALIB_STATS
 *   2. 手持板子在视野内移动10秒
 *   3. 读取统计量: 总帧数/有效帧数/拒绝原因分布
 *
 * 作者:Claude Code  日期:2026-08-10
 ********************************************************************************************************************/

#ifndef KART_VISION_CALIBRATE_H_
#define KART_VISION_CALIBRATE_H_
#include "zf_common_headfile.h"
#include "kart_vision.h"

/* ======================== 标定模式选择 ======================== */

#define CALIB_DISABLED     (0)     /* 关闭标定功能,零开销 */
#define CALIB_FOCAL        (1)     /* 焦距标定模式 */
#define CALIB_COLOR        (2)     /* 颜色判别验证模式 */
#define CALIB_STATS        (3)     /* 识别统计模式 */

/* 当前标定模式(编译期选择) */
#ifndef CALIB_MODE
#define CALIB_MODE         (CALIB_DISABLED)
#endif

/* ======================== 焦距标定参数 ======================== */

/* 标定时板子的实际距离(米),需要用米尺精确测量 */
#define CALIB_DIST_M       (1.50f)

/* 采样窗口大小(帧数),用于滤波抖动 */
#define CALIB_WINDOW_SIZE  (50)

/* ======================== 颜色验证参数 ======================== */

/* 取样像素位置(图像中心) */
#define CALIB_SAMPLE_X     (80)    /* 160/2 */
#define CALIB_SAMPLE_Y     (60)    /* 120/2 */

/* ======================== 统计信息 ======================== */

typedef struct
{
    uint32 total_frames;        /* 累计处理帧数 */
    uint32 valid_frames;        /* 检测到有效目标的帧数 */
    uint32 reject_area;         /* 因面积不足拒绝的帧数 */
    uint32 reject_width;        /* 因宽度不足拒绝的帧数 */
    uint32 reject_aspect;       /* 因宽高比拒绝的帧数 */
    uint32 reject_fill;         /* 因填充率拒绝的帧数 */

    /* 焦距标定用 */
    float  width_sum;           /* 像素宽度累加(用于计算平均值) */
    uint32 width_count;         /* 有效宽度采样数 */
    float  width_avg;           /* 平均像素宽度 */
    float  focal_estimate;      /* 估算焦距 = (width_avg × dist) / board_w */
} kart_calib_stat_t;

/* ======================== 对外接口 ======================== */

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称  kart_calib_init
 * 功能说明  标定模块初始化(清零统计量)
 * 参数说明  void
 * 返回参数  void
 *-----------------------------------------------------------------------------------------------------------------*/
void kart_calib_init(void);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称  kart_calib_update
 * 功能说明  更新标定数据(每帧调用一次)
 * 参数说明  img    - 图像数据指针(RGB565)
 *           w      - 图像宽度
 *           h      - 图像高度
 *           result - 视觉检测结果(kart_vision_process 的返回值)
 * 返回参数  void
 * 注意事项  必须在 kart_vision_process() 之后调用
 *-----------------------------------------------------------------------------------------------------------------*/
void kart_calib_update(const uint16 *img, int16 w, int16 h, const kart_vision_result_t *result);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称  kart_calib_get_stat
 * 功能说明  获取标定统计信息
 * 参数说明  void
 * 返回参数  统计量结构体指针(只读)
 *-----------------------------------------------------------------------------------------------------------------*/
const kart_calib_stat_t *kart_calib_get_stat(void);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称  kart_calib_reset
 * 功能说明  清零统计量(重新开始标定)
 * 参数说明  void
 * 返回参数  void
 *-----------------------------------------------------------------------------------------------------------------*/
void kart_calib_reset(void);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称  kart_calib_print_report
 * 功能说明  通过串口打印标定报告(阻塞式)
 * 参数说明  void
 * 返回参数  void
 * 注意事项  仅在标定结束后手动调用,会阻塞数百ms
 *-----------------------------------------------------------------------------------------------------------------*/
void kart_calib_print_report(void);

/* ======================== VOFA输出宏 ======================== */

#if (CALIB_MODE == CALIB_FOCAL)
    /* 焦距标定模式: 输出 [width_px, dist_est, focal_est, valid] */
    #define CALIB_VOFA_OUTPUT(result) \
        do { \
            float w_px = (float)(result)->width_px; \
            float d_est = (result)->dist_m; \
            const kart_calib_stat_t *stat = kart_calib_get_stat(); \
            float f_est = stat->focal_estimate; \
            float valid = (float)(result)->valid; \
            printf("%.2f,%.2f,%.2f,%.1f\n", w_px, d_est, f_est, valid); \
        } while(0)

#elif (CALIB_MODE == CALIB_COLOR)
    /* 颜色验证模式: 输出 [r5, g5, b5, m_lhs, m_rhs, is_target] */
    #define CALIB_VOFA_OUTPUT(result) \
        do { \
            extern const uint16 *g_calib_img_ptr; \
            extern int16 g_calib_img_w; \
            if(g_calib_img_ptr != NULL && g_calib_img_w > 0) { \
                int16 x = CALIB_SAMPLE_X; \
                int16 y = CALIB_SAMPLE_Y; \
                uint16 pix = g_calib_img_ptr[y * g_calib_img_w + x]; \
                int16 r5, g5, b5; \
                int32 m_lhs, m_rhs; \
                int16 sum; \
                uint8 is_target = kart_vision_probe_pixel(pix, &r5, &g5, &b5, &m_lhs, &m_rhs, &sum); \
                printf("%d,%d,%d,%d,%d,%d\n", r5, g5, b5, (int)m_lhs, (int)m_rhs, is_target); \
            } \
        } while(0)

#elif (CALIB_MODE == CALIB_STATS)
    /* 统计模式: 输出 [valid_rate, reject_area%, reject_aspect%, avg_width] */
    #define CALIB_VOFA_OUTPUT(result) \
        do { \
            const kart_calib_stat_t *stat = kart_calib_get_stat(); \
            float rate = (stat->total_frames > 0) ? \
                (100.0f * stat->valid_frames / stat->total_frames) : 0.0f; \
            float r_area = (stat->total_frames > 0) ? \
                (100.0f * stat->reject_area / stat->total_frames) : 0.0f; \
            float r_asp = (stat->total_frames > 0) ? \
                (100.0f * stat->reject_aspect / stat->total_frames) : 0.0f; \
            float w_avg = stat->width_avg; \
            printf("%.1f,%.1f,%.1f,%.1f\n", rate, r_area, r_asp, w_avg); \
        } while(0)

#else
    /* 关闭模式: 空操作 */
    #define CALIB_VOFA_OUTPUT(result)  ((void)0)
#endif

#endif  /* KART_VISION_CALIBRATE_H_ */
