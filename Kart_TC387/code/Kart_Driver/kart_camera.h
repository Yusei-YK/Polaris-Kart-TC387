/*********************************************************************************************************************
 * 文件名称  kart_camera
 * 所属分层  Kart_Driver（直接摸硬件层）
 * 功能说明  SCC8660（凌瞳）彩色摄像头接入 + 采集链路诊断计数
 *
 * 【为什么要包一层，不直接调 scc8660_init()】
 *   1) 逐飞驱动的 vsync/dma 回调是 static 的，外部数不到。这里在 init 之后把
 *      camera_vsync_handler / camera_dma_handler 换成本模块的包装函数，
 *      先计数再转调原函数 —— 用日志判断出图是否正常，而不是靠感觉。
 *   2) 上电失败要能重试，且失败原因要留在全局量里给菜单看，而不是一个返回码丢掉。
 *   3) 帧的所有权要有明确交接（frame_ready / frame_release），否则上层处理到一半就被
 *      DMA 覆盖，出来的是上下半帧拼接的鬼图。
 *
 * 【UART1 的事：不需要仲裁，只需要初始化顺序】
 *   UART1(ASCLIN1) 上挂着灯板 TLD7002（2Mbps，P11.12/P11.10），摄像头上电配置也要
 *   UART1（9600，P02.2/P02.3）。但两者【时间上不重叠】：
 *     - scc8660 只在上电配置期用 UART1，而且只在 SCCB 软 I2C 探测失败、回退 UART
 *       时才用（见 zf_device_scc8660.c 的 scc8660_init）。kart_camera_init() 之后 UART1
 *       再也不碰，图像走 DVP + DMA。
 *     - 科三跑图期间点阵和灯板完全不动；菜单科二放语音/灯效时摄像头完全用不着。
 *   所以只要保证 kart_camera_init() 早于 dot_matrix_screen_init()，UART1 就静态
 *   归属灯板：不需要运行期归属标志、不需要停/开点阵 PIT、ISR 里也不需要分派。
 *   （2026-08-10 已按此删除原先的动态仲裁，见 .c 里的删除说明。）
 *
 * 【资源占用（只读，勿改，改了要同步 board_pins.h）】
 *   DVP 数据    P00.0 - P00.7        （TLD7002 的 GPIN0 是 P00.8，不冲突）
 *   PCLK        ERU_CH2_REQ14_P02_1  上升沿，中断服务源 = DMA，优先级 5
 *   VSYNC       ERU_CH3_REQ6_P02_0   下降沿，cpu0，优先级 62
 *   DMA         IfxDma_ChannelId_5   cpu0，优先级 70
 *   配置串口    UART_1 @9600  P02.2/P02.3（仅上电配置期，之后归灯板）
 *
 * 【已知风险，接上硬件后必须实测】
 *   A) DMA_INT_PRIO(70) 与 VSYNC EXTI(62) 都高于 5ms 控制 PIT 的 CCU6_0_CH0(50)，
 *      也就是说摄像头中断会抢占控制环。38400 字节 / link_list=3 → 每帧 3 次 DMA 中断，
 *      60FPS 时约 180 次/秒。先用 g_cam_isr_max_us 量，超预算再谈调优先级。
 *   B) 凌瞳 PCLK 最高 54MHz，约为总钻风的 7 倍。单根网线总长（FFC + 主板走线）要 < 30cm，
 *      25cm 排线时主板走线必须 ≤ 5cm，超了会花屏。
 *   C) 3.3V 纹波敏感。出现明暗条纹/噪点先查电源，不要先怀疑代码。
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-08-08        Kart                阶段0：出图与诊断，尚未接入图像处理
 * 2026-08-10        Kart                删除 UART1 动态仲裁，改为静态归属 + 初始化顺序约束
 ********************************************************************************************************************/

#ifndef KART_CAMERA_H_
#define KART_CAMERA_H_

/* 与同层 kart_power.h / kart_encoder.h 保持一致:直接吃逐飞总头文件。
 * 本模块需要其中的 scc8660(SCC8660_W/H/IMAGE_SIZE)、type(callback_function)、
 * timer(system_getval)、interrupt、ips200。 */
#include "zf_common_headfile.h"

/* 总开关。0 = 完全不碰摄像头，保持已验证的低速基线原样（编译期就不产生调用）。
 * 硬件接好、走线长度确认过之后再改 1。改 1 之前请先读上面的"已知风险"。 */
#ifndef CAMERA_ENABLE
#define CAMERA_ENABLE              (1)
#endif

/* 上电初始化失败时是否重试。凌瞳 set_config 单次超时 240ms，重试代价不低，
 * 上电阶段可接受；跑车途中不做任何重试。 */
#define CAMERA_INIT_RETRY          (2)

/* 连续多少毫秒没有 VSYNC 判为"无信号"。60FPS 一帧 16.7ms，25FPS 一帧 40ms，取 200ms 足够宽。 */
#define CAMERA_NO_SIGNAL_MS        (200)

/* 室外成像参数。逐飞驱动中 white_balance=0 的含义是启用自动白平衡，
 * 因此必须给 0x65~0xA0 内的非零值才能真正锁定白平衡；0x80 取中性档。
 * 曝光改在 zf_device_scc8660.h 中配置为 AUTO_EXP=1、目标亮度=100
 * （逐飞注明的自动曝光推荐值），适应室内外光照变化并避免整片过曝。 */
#define CAMERA_OUTDOOR_WB          (0x80u)

/* 采集链路状态。诊断用，菜单/日志直接显示。 */
typedef enum
{
    KART_CAM_STATE_OFF = 0,              /* 未启用（CAMERA_ENABLE=0 或没调 init） */
    KART_CAM_STATE_INIT_FAIL,            /* scc8660_init 返回非 0：配置串口不通或参数回读不符 */
    KART_CAM_STATE_NO_SIGNAL,            /* init 过了，但迟迟收不到 VSYNC：DVP 排线/PCLK 问题 */
    KART_CAM_STATE_RUNNING               /* 正常出帧 */
}kart_cam_state_enum;

/*=================================== 诊断计数，只读，供日志/菜单观察 =======================================*/
extern volatile uint32 g_cam_vsync_count;    /* VSYNC 次数，等于摄像头实际输出的帧数 */
extern volatile uint32 g_cam_dma_count;      /* DMA 完成中断次数，正常 = vsync × link_list_num */
extern volatile uint32 g_cam_frame_count;    /* 上层取走的完整帧数 */
extern volatile uint32 g_cam_misalign_count; /* 错位帧：上一帧 DMA 中断数 != link_list_num */
extern volatile uint32 g_cam_drop_count;     /* 丢帧：上层还没取走就被下一帧覆盖 */
extern volatile uint32 g_cam_isr_max_us;     /* VSYNC + DMA 单次 ISR 最长耗时，判是否啃控制环 */
extern volatile uint16 g_cam_fps;            /* 实测帧率，1s 统计一次 */
extern volatile uint8  g_cam_link_list_num;  /* dma_init 拆的链表段数，38400 字节应为 3 */
extern volatile int    g_cam_init_ret;       /* scc8660_init 返回码，0 = 成功 */
extern volatile uint8  g_cam_init_try;       /* 实际尝试次数 */
extern volatile uint32 g_cam_init_us;        /* init 总耗时 */
extern volatile uint8  g_cam_wb_ret;         /* 固定白平衡下发结果：0成功，0xFF未执行 */

/*======================================== 对外接口 ========================================================*/

/* 上电初始化。内部完成：scc8660_init（失败重试 CAMERA_INIT_RETRY 次）→ 挂计数包装回调。
 *
 * 【两条硬性调用顺序约束，改 cpu0_main 时不要动】
 *   1) 必须在 dot_matrix_screen_init() 之【前】调用。摄像头配置可能要用 UART1@9600
 *      (P02.2/P02.3)，而灯板 TLD7002 用 UART1@2Mbps；先配摄像头，再让灯板把 UART1
 *      按 2Mbps 重配走，此后 UART1 静态归灯板，互不干扰。反了则摄像头配置必然超时。
 *   2) 必须在 pit_ms_init(CCU60_CH0, ...) 打开 5ms 控制环之【前】调用。
 *      本函数阻塞约 0.5 - 1.5s（单次 set_config 超时 240ms × 重试），
 *      放在控制环之后会直接把控制周期撑爆。
 *
 * 返回 0 成功，非 0 失败；失败不影响其它功能，车照常能跑（只是没图）。 */
uint8  kart_camera_init                 (void);

/* 10ms 周期调用。刷新帧率/无信号判定，不做任何图像处理，不阻塞。 */
void   kart_camera_poll                 (void);

/* 是否有一帧可用。返回 1 时图像在 scc8660_image 里，处理完必须调 kart_camera_frame_release()。 */
uint8  kart_camera_frame_ready          (void);

/* 释放当前帧，允许 DMA 写下一帧。 */
void   kart_camera_frame_release        (void);

kart_cam_state_enum kart_camera_state   (void);

/* IPS200 预览。整屏 RGB565，逐行阻塞 SPI，耗时几十毫秒级 ——
 * 只能在 IDLE/调试页调用，绝对不能进 5ms 控制窗口。内部自带限帧。 */
void   kart_camera_preview              (void);

#endif
