/*********************************************************************************************************************
 * 文件名称  kart_camera
 * 功能说明  SCC8660 接入 + UART1 分时 + 采集诊断。详见 kart_camera.h 顶部说明。
 *
 * 【本文件的定位】
 *   阶段 0 只做一件事：确认摄像头能稳定出图，并且出图这件事没有把 5ms 控制环啃掉。
 *   不做任何图像识别 —— 识别方案（引导板颜色/图案、跟随方向）还没定，先出图再谈算法。
 ********************************************************************************************************************/
#include "kart_camera.h"
#include "kart_include.h"

#pragma section all "cpu0_dsram"

/* 编译期护栏。为什么要在编译期拦，而不是运行时判：
 *   dma_init() 里有 zf_assert(!(dma_count % 8))，而 debug_assert_handler() 断言失败后是
 *   while(TRUE) 死循环 —— 单片机直接卡死，车在场上就是一动不动。这种错误绝不能留到运行时。
 *   另外 dma_init 的链表拆分在段数 > 10 时也会 zf_assert(FALSE)。
 *   当前 160×120 RGB565 = 38400 字节：38400%8=0，拆 3 段，两条都过。
 *   若以后改分辨率，改完编译就会在这里报错，而不是烧进去才发现。
 *   顺带一提 320×240 = 153600 恰好需要 10 段，正卡在上限，不留余量，慎用。 */
typedef char cam_size_mod8_check[(0 == (SCC8660_IMAGE_SIZE % 8)) ? 1 : -1];
typedef char cam_size_max_check[(SCC8660_IMAGE_SIZE <= (16384 * 10)) ? 1 : -1];

/* 时基选型说明：
 *   不用 g_tick_5ms —— 它定义在 user/isr.c，Kart_Driver 层往 user/ 依赖是反向依赖，
 *   违反 kart_include.h 定的 Kart_Decision→Kart_App→Kart_Algo/Kart_Driver→Kart_Config 单向规则。
 *   改用库定时器 system_getval()：返回 10ns 为单位的 uint32，约 42.9s 回绕一次。
 *   必须在“原始计数”上做差分再换成毫秒 —— system_getval_ms() 先除再差，
 *   回绕那一瞬间差值不对。本模块最长的间隔是 1s，远小于 42.9s，uint32 差分安全。
 *   注：core0 调 system_getval() 用 STM0，本模块的 ISR 包装和 poll 都在 cpu0，时基一致。 */
#define CAM_RAW_PER_MS             (100000u)   /* 1ms = 100000 × 10ns */


volatile uint32 g_cam_vsync_count    = 0;
volatile uint32 g_cam_dma_count      = 0;
volatile uint32 g_cam_frame_count    = 0;
volatile uint32 g_cam_misalign_count = 0;
volatile uint32 g_cam_drop_count     = 0;
volatile uint32 g_cam_isr_max_us     = 0;
volatile uint16 g_cam_fps            = 0;
volatile uint8  g_cam_link_list_num  = 0;
volatile int    g_cam_init_ret       = -1;
volatile uint8  g_cam_init_try       = 0;
volatile uint32 g_cam_init_us        = 0;
volatile uint8  g_cam_wb_ret         = 0xFFu;

static kart_cam_state_enum  cam_state         = KART_CAM_STATE_OFF;

#if (CAMERA_ENABLE)

static volatile uint8       cam_frame_hold    = 0;    /* 1 = 上层正在用这一帧，不许被覆盖 */
static volatile uint32      cam_last_vsync_raw = 0;   /* 最后一个 VSYNC 的原始计数（10ns） */
static volatile uint8       cam_dma_in_frame  = 0;    /* 本帧已经来了几次 DMA 中断 */

/* 原始回调，包装函数里转调 */
static callback_function    cam_raw_vsync     = NULL;
static callback_function    cam_raw_dma       = NULL;

/* 帧率统计 */
static uint32 cam_fps_last_raw   = 0;
static uint32 cam_fps_last_count = 0;

/* 预览限帧 */
static uint32 cam_preview_last_raw = 0;
#define CAM_PREVIEW_PERIOD_MS      (200)     /* 最快 5Hz，够看清图像，又不至于把主循环拖垮 */

/*-------------------------------------------------------------------------------------------------------------------
 * VSYNC 包装。原函数负责重挂 DMA，这里只加计数和错位判定。
 * 注意顺序：先判上一帧的 DMA 中断数够不够，再转调原函数（原函数会清 dma_int_num）。
 -------------------------------------------------------------------------------------------------------------------*/
static void kart_camera_vsync_wrap(void)
{
    uint32 t0_raw = system_getval();
    uint32 dt;

    /* 上一帧收到的 DMA 中断数不等于链表段数 → 那一帧是残帧。
     * 逐飞驱动内部靠 TransactionRequestLost 自恢复，这里只做统计，不干预。 */
    if((g_cam_vsync_count != 0) && (cam_dma_in_frame != g_cam_link_list_num))
    {
        g_cam_misalign_count++;
    }
    cam_dma_in_frame = 0;

    g_cam_vsync_count++;
    cam_last_vsync_raw = t0_raw;

    if(cam_raw_vsync != NULL)
    {
        cam_raw_vsync();
    }

    /* 原始计数差分再换 us，避开 system_getval_us() “先除再差”在回绕处出错 */
    dt = (system_getval() - t0_raw) / 100u;
    if(dt > g_cam_isr_max_us)
    {
        g_cam_isr_max_us = dt;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * DMA 完成包装。
 * 关键点：原函数在最后一段完成时会把 scc8660_finish_flag 置 1。若上层还占着上一帧
 * （cam_frame_hold=1），说明来不及处理，记一次丢帧，并把 finish_flag 压回 0，
 * 让图像缓冲继续被下一帧覆盖 —— 宁可丢新帧也不能让上层读到写一半的图。
 -------------------------------------------------------------------------------------------------------------------*/
static void kart_camera_dma_wrap(void)
{
    uint32 t0_raw = system_getval();
    uint32 dt;

    if(cam_raw_dma != NULL)
    {
        cam_raw_dma();
    }

    g_cam_dma_count++;
    cam_dma_in_frame++;

    if(scc8660_finish_flag)
    {
        if(cam_frame_hold)
        {
            scc8660_finish_flag = 0;            /* 上层没腾出手，这帧作废 */
            g_cam_drop_count++;
        }
        else
        {
            g_cam_frame_count++;
        }
    }

    /* 原始计数差分再换 us，避开 system_getval_us() “先除再差”在回绕处出错 */
    dt = (system_getval() - t0_raw) / 100u;
    if(dt > g_cam_isr_max_us)
    {
        g_cam_isr_max_us = dt;
    }
}

/*------------------------------------------------------------------------------
 * 【2026-08-10 删除说明】这里原有一个 kart_camera_uart_give_back() —— 摄像头配置
 * 期把 UART1 借走,配完再改归属、重跑 tld7002_init、重开 1ms 点阵扫描。
 * 连同它一起删掉的还有运行期归属标志和 uart1_rx_isr 里的分派。
 * 删的理由(kart_camera.h 顶部详述):两者时间上不重叠,只要 kart_camera_init()
 * 早于 dot_matrix_screen_init(),UART1 就静态归灯板,不需要运行期仲裁。
 * 现在灯板的 UART1 是由 dot_matrix_screen_init() 内部的 tld7002_init() 按 2Mbps
 * 重配的,时序约束写在 cpu0_main.c:271 和本文件 init 的注释里。
 ------------------------------------------------------------------------------*/

#endif  /* CAMERA_ENABLE */

/*-------------------------------------------------------------------------------------------------------------------
 * 初始化。阻塞，只能上电阶段调，不能在 5ms 环开着的时候调。
 -------------------------------------------------------------------------------------------------------------------*/
uint8 kart_camera_init(void)
{
#if (CAMERA_ENABLE)
    uint32 t_start_raw = system_getval();
    uint8  ret     = 1;
    uint8  try_i;

    cam_state = KART_CAM_STATE_INIT_FAIL;
    g_cam_wb_ret = 0xFFu;

    /* 这里不需要停点阵扫描、也不需要改 UART1 归属:本函数被约束在
     * dot_matrix_screen_init() 之前调用(cpu0_main.c:265 / :275),那时灯板还没起来,
     * UART1 没人用。原先的动态仲裁已于 2026-08-10 删除,删除说明就在本函数上方
     * (CAMERA_ENABLE 段末那块注释)。 */

    for(try_i = 0; try_i < CAMERA_INIT_RETRY; try_i++)
    {
        g_cam_init_try = (uint8)(try_i + 1);

        /* scc8660_init 先试 SCCB（P02.3/P02.2 软 I2C），不通再退回 UART 9600。
         * 拆了板载 51 的摄像头走 SCCB，没拆的走 UART，驱动自己认。 */
        g_cam_init_ret = (int)scc8660_init();

        if(0 == g_cam_init_ret)
        {
            /* init 已按逐飞库配置 AUTO_EXP=1、目标亮度=100；但其默认
             * MANUAL_WB=0 会开启自动白平衡，所以在配置链路仍归摄像头
             * 所有时覆盖成固定中性白平衡。接口返回 0 表示成功；失败仍
             * 沿用基础初始化，不因调色参数通信失败而把采集链判死。 */
            g_cam_wb_ret = scc8660_set_white_balance(CAMERA_OUTDOOR_WB);
            ret = 0;
            break;
        }

        system_delay_ms(50);
    }

    if(0 == ret)
    {
        /* 套上诊断包装。必须在 init 成功之后做 —— init 内部自己会多次
         * set_camera_type，早套会被覆盖掉。 */
        cam_raw_vsync = camera_vsync_handler;
        cam_raw_dma   = camera_dma_handler;

        camera_vsync_handler = kart_camera_vsync_wrap;
        camera_dma_handler   = kart_camera_dma_wrap;

        /* 38400 字节 / 单段上限 16384 → 应当拆成 3 段，也就是每帧 3 次 DMA 中断。
         * 这个数只能从 dma 驱动的拆分规则推出来，驱动没导出，这里按同样规则算一遍，
         * 用于错位判定。算错了只会让 misalign 计数不准，不影响出图。 */
        {
            uint32 total = SCC8660_IMAGE_SIZE;
            uint8  n     = 1;
            while(((total / n) > 16384u) || (0 != (total % n)))
            {
                n++;
                if(n > 10)
                {
                    n = 1;
                    break;
                }
            }
            g_cam_link_list_num = n;
        }

        scc8660_finish_flag = 0;
        cam_frame_hold      = 0;
        cam_dma_in_frame    = 0;
        cam_state           = KART_CAM_STATE_NO_SIGNAL;   /* 等第一个 VSYNC 才算 RUNNING */
    }

    /* 这里也不需要把 UART1 还给灯板 —— 静态归属之后灯板自己在
     * dot_matrix_screen_init() 里把 UART1 配成 2Mbps。摄像头初始化失败不影响灯:
     * 本函数无论成败都只是设 cam_state 和几个诊断量,车照常能跑(只是没图),
     * 而灯是科目三终点语音联动要用的,属于计分项。 */

    g_cam_init_us = (system_getval() - t_start_raw) / 100u;

    cam_last_vsync_raw = system_getval();
    cam_fps_last_raw   = cam_last_vsync_raw;
    cam_fps_last_count = 0;

    return ret;
#else
    cam_state = KART_CAM_STATE_OFF;
    return 1;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 10ms 轮询。只算帧率和无信号判定，不碰图像。
 -------------------------------------------------------------------------------------------------------------------*/
void kart_camera_poll(void)
{
#if (CAMERA_ENABLE)
    uint32 now_raw;
    uint32 vsync_now;

    if(KART_CAM_STATE_INIT_FAIL == cam_state || KART_CAM_STATE_OFF == cam_state)
    {
        return;
    }

    now_raw   = system_getval();
    vsync_now = g_cam_vsync_count;

    /* 无信号判定：init 成功但收不到 VSYNC，基本是 DVP 排线没插好或 PCLK 走线太长。
     * 凌瞳 PCLK 最高 54MHz，总长超 30cm 就会花屏甚至完全不出。 */
    if((now_raw - cam_last_vsync_raw) > (CAMERA_NO_SIGNAL_MS * CAM_RAW_PER_MS))
    {
        cam_state = KART_CAM_STATE_NO_SIGNAL;
    }
    else
    {
        cam_state = KART_CAM_STATE_RUNNING;
    }

    if((now_raw - cam_fps_last_raw) >= (1000u * CAM_RAW_PER_MS))
    {
        g_cam_fps     = (uint16)(vsync_now - cam_fps_last_count);
        cam_fps_last_count = vsync_now;
        cam_fps_last_raw   = now_raw;
    }
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 取帧 / 放帧。中间夹的处理过程里 DMA 不会动这块缓冲。
 -------------------------------------------------------------------------------------------------------------------*/
uint8 kart_camera_frame_ready(void)
{
#if (CAMERA_ENABLE)
    uint32 primask;
    uint8  ready = 0;

    /* 与 kart_odom 取快照同一套写法：关全局中断读改一小段，避免和 DMA ISR 撞上 */
    primask = interrupt_global_disable();
    if(scc8660_finish_flag && (0 == cam_frame_hold))
    {
        cam_frame_hold = 1;
        ready          = 1;
    }
    interrupt_global_enable(primask);

    return ready;
#else
    return 0;
#endif
}

void kart_camera_frame_release(void)
{
#if (CAMERA_ENABLE)
    uint32 primask;

    primask = interrupt_global_disable();
    scc8660_finish_flag = 0;
    cam_frame_hold      = 0;
    interrupt_global_enable(primask);
#endif
}

kart_cam_state_enum kart_camera_state(void)
{
    return cam_state;
}

/*-------------------------------------------------------------------------------------------------------------------
 * IPS200 预览。逐行阻塞 SPI，120 行 → 120 次传输，耗时几十毫秒。
 * 只能在 IDLE/调试页调用。这里再加一层限帧，防止误接进快周期任务把主循环拖死。
 -------------------------------------------------------------------------------------------------------------------*/
void kart_camera_preview(void)
{
#if (CAMERA_ENABLE)
    uint32 now_raw = system_getval();

    if((now_raw - cam_preview_last_raw) < (CAM_PREVIEW_PERIOD_MS * CAM_RAW_PER_MS))
    {
        return;
    }

    if(0 == kart_camera_frame_ready())
    {
        return;
    }
    cam_preview_last_raw = now_raw;

    /* 等比铺满 SCC8660 原始分辨率区域。dis_width 是栈上变长数组的长度，
     * 传 SCC8660_W(160) 时占 320 字节栈，安全。 */
    ips200_show_rgb565_image(0, 0, (const uint16 *)scc8660_image[0],
                             SCC8660_W, SCC8660_H, SCC8660_W, SCC8660_H, 1);

    kart_camera_frame_release();
#endif
}

#pragma section all restore
