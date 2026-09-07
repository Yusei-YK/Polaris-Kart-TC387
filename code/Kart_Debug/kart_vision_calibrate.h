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
 *   【第 0 步:先把它接进去】本模块当前没有任何调用点 —— kart_vision_calibrate.h
 *   全工程没人 #include(kart_include.h 里也没有),kart_calib_update() /
 *   kart_calib_reset() / kart_calib_print_report() 三个函数一个调用点都没有。
 *   只改 CALIB_MODE 是采不到样本的,必须先:
 *     a) 在跑视觉的那个文件里 #include "kart_vision_calibrate.h";
 *     b) 紧跟 kart_vision_process() 之后调 kart_calib_update(img, w, h, result);
 *     c) 要看波形再加 CALIB_VOFA_OUTPUT(result);标定结束手动调 print_report()。
 *
 *   [焦距标定]
 *   1. 设置 CALIB_MODE = CALIB_FOCAL
 *   2. 板子放在已知距离,并且要正对镜头轴心(理由见下面第 5 条)
 *      【别照抄 1.0/1.5/2.0/2.5/3.0 这五个点】板宽的像素数 = BOARD_W_M×FPX/d,
 *      这五个距离分别只有 27.6 / 18.4 / 13.8 / 11.0 / 9.2 px,而 VISION_MIN_WIDTH=10,
 *      所以 3.0m 处的 9.2px 会被宽度门直接判 REJ_WIDTH,一个样本都进不来;
 *      2.5m 的 11.0px 只剩一像素余量。这套板子/镜头的上限是 2.76 m,
 *      实际可用的点取 1.0 / 1.5 / 2.0 / 2.5 就够了。
 *   3. 静止十几秒,让 50 帧的滑动窗口填满(见 .c 里 g_width_window)
 *   4. 【不用手算,也不用取中位值】stat->focal_estimate 就是
 *      (窗口内 width_px 的算术平均 × CALIB_DIST_M) / VISION_BOARD_W_M,
 *      print_report() 会把它打出来,直接读那个数。工具里没有中位数。
 *   5. 【换距离点必须改 CALIB_DIST_M 并重新编译下载】它是编译期常量,不是自动测的。
 *      忘了改,站在 2.0m 却还是 1.50 的话,估算焦距会偏低到真值的 75%,
 *      而报告里"标定距离"一栏照样印 1.50,看不出错。
 *   6. 取各点 focal_estimate 的平均值更新 kart_vision.h 的 VISION_FPX
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
 * 成稿日期:2026-08-10
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

/* 标定时板子的实际距离(米),需要用米尺精确测量。
 * 【这是编译期常量,换距离点要改这里再重新编译下载】焦距估算式直接乘它,
 * 站在别的距离上却没改,估算焦距就按两个距离的比例整体偏掉,而且报告里
 * "标定距离"一栏印的也是这个宏,数字自洽,肉眼查不出来。 */
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
    uint32 reject_area;         /* 因面积不足拒绝的帧数。
                                 * 【这一路混了四件事】REJ_AREA 是
                                 * kart_vision_process 一进函数就预置的
                                 * 缺省值,成功前不清,所以 img==NULL、
                                 * w/h<=0、图比 160x120 掩膜大,
                                 * 都跟"真的团块太小"记在同一格里。
                                 * 板子太远到连种子都不成形时,这里涨的
                                 * 是面积而不是宽度 —— 别据此判断门限。 */
    uint32 reject_width;        /* 因宽度不足拒绝的帧数(bw < VISION_MIN_WIDTH)。
                                 * 距离超过 2.76 m 就只会落在这一格。 */
    uint32 reject_aspect;       /* 因宽高比拒绝的帧数。
                                 * 【这一路混了两件事】真的宽高比越界,和
                                 * 连续性门拦掉"跳到另一个同色物体" —— 后者
                                 * 因为 reject 取值不能新增而借用了同一个值。
                                 * 本工具不读 cont_reject,所以这两者在报告里
                                 * 分不开。要分就自己把 cont_reject 也记一路。 */
    uint32 reject_fill;         /* 因填充率拒绝的帧数(斜置的细长反光带)。 */

    /* 焦距标定用 */
    float  width_sum;           /* 像素宽度累加(用于计算平均值) */
    uint32 width_count;         /* 有效宽度采样数 */
    float  width_avg;           /* 平均像素宽度 */
    float  focal_estimate;      /* 估算焦距 = (width_avg × dist) / board_w。
                                 * 【这是按板宽解的,跑车时测距按板高解】
                                 * kart_vision.c 里 dist_m = BOARD_H_M×FPX/bh,
                                 * kart_vision.h 的 VISION_FPX 也是按竖板高度
                                 * 实测反算的。同一个 f_px 两边都成立的前提是
                                 * 外接框的宽高比正好等于 0.73,
                                 * 而宽高比门放行的是 0.3~1.8 这么宽一段,
                                 * 板子稍歪或被挡一角,这里解出来的数就和按高度
                                 * 标出来的对不上。两个都测,不一致就是板子没正对。 */
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
 * 返回参数  统计量结构体指针(只读)。
 *           【CALIB_MODE = CALIB_DISABLED 时返回 NULL】.c 末尾的空壳实现是
 *           return NULL,不是返回一个清零的结构体。默认就是 DISABLED,
 *           所以任何不带 #if 判断就解引用它的新代码都会当场空指针。
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

/* ======================== 调试输出宏 ======================== */
/* 【名字叫 VOFA,走的却不是本工程那条 VOFA 链路】下面三个宏都是 printf 出
 * 逗号分隔的 ASCII(VOFA 里要选 FireWater 模式才认),而 kart_debug_uart.c
 * 发的是 JustFloat 二进制。两者连的口也不是一个:printf 走 debug_init()
 * 配的 UART_0(user/cpu0_main.c 开头),日志在 UART_10(LOG_ON_UART0=0)。
 * 所以这些宏不会打乱日志波形,但 UART_0 正是逐飞助手图传用的口
 * (kart_assist_img.h 的 AIMG_UART),图传进行中 printf 会把 ASCII 插进
 * 那一帧二进制图里,整张图错乱。要同时看图和标定量,就分两次跑。 */

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
