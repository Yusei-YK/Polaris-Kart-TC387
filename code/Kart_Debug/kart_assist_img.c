/*********************************************************************************************************************
 * 文件名称  kart_assist_img
 * 功能说明  逐飞助手有线图传实现。设计取舍见 kart_assist_img.h 文件头。
 *
 * 【协议实现】按逐飞助手 V2 Kart_TC387 官方例程发送 12 字节 CAMERA 配置头，
 * 随后连续发送 RGB565 像素。原因与现场 HEX 证据见 .h 文件头。
 ********************************************************************************************************************/
#include "kart_assist_img.h"
#include "kart_include.h"
#include <string.h>

#pragma section all "cpu0_dsram"

/*=========================== 诊断量 ===========================*/
volatile uint32 g_aimg_img_sent      = 0;
volatile uint32 g_aimg_img_skip      = 0;
volatile uint32 g_aimg_img_ms        = 0;
volatile uint32 g_aimg_chunk_max_us  = 0;
volatile uint8  g_aimg_busy          = 0;
volatile uint32 g_aimg_progress      = 0;

#if AIMG_ENABLE

/* 【为什么要查这条】助手上位机按帧头里的 width*height*2 收字节。
 * 若 SCC8660_IMAGE_SIZE 与 SCC8660_W*2*SCC8660_H 不等（库改过定义），
 * 发出去的长度和声明的长度对不上，上位机会一直等下一帧，
 * 表现为“只出一帧然后卡死”。 */
typedef char aimg_size_check[(SCC8660_IMAGE_SIZE == (SCC8660_W * 2 * SCC8660_H)) ? 1 : -1];
/* 字节交换按 2 字节步进，帧长必须是偶数，否则最后一步越界。 */
typedef char aimg_even_check[((SCC8660_IMAGE_SIZE % 2) == 0) ? 1 : -1];

#define AIMG_RAW_PER_MS            (100000u)   /* 1ms = 100000 x 10ns，与 kart_camera/kart_wifi 同口径 */

/*=========================== 状态机 ===========================*/
typedef enum
{
    AIMG_TX_IDLE = 0,       /* 空闲，可接受新请求 */
    AIMG_TX_PENDING,        /* 已复制一帧，V2 头还没发 */
    AIMG_TX_SENDING         /* 头已发出，正在一块一块发图像数据 */
}aimg_tx_state_enum;

static aimg_tx_state_enum tx_state = AIMG_TX_IDLE;

/* 帧拷贝缓冲。见 .h “为什么要把帧拷出来”。
 * 38400 字节单独放 cpu1 DSRAM：cpu0 是 LCF_DEFAULT_HOST，
 * 除本文件外还要装 scc8660_image、preprocess_buf、kart_vtrack 金字塔、
 * kart_vision 掩膜以及库里所有普通全局，240K 装不下。
 * 这块只被 UART 逐字节读出，460800 波特率下跳核访问 SRI
 * 的额外延迟被波特率完全掩盖。 */
#pragma section all restore
#pragma section all "cpu1_dsram"
static uint8  img_buf[SCC8660_IMAGE_SIZE];
#pragma section all restore
#pragma section all "cpu0_dsram"

static uint32 img_start_raw;
static uint32 img_offset;       /* 已发出的图像字节数，分块发送的进度 */

/* 逐飞助手 V2 CAMERA 配置头（官方例程 seekfree_assistant_camera_struct.config）。
 * 全部使用 uint8 明确布局，避免编译器结构体对齐改变 12 字节协议长度。 */
static uint8  camera_v2_head[12];

/* V2 校验规则：head + 从 cmd 到 reserve 末尾的所有字节，结果取低 8 位。
 * check_sum 本身不参与计算。 */
static void aimg_build_v2_head(void)
{
    uint8 i;
    uint8 sum;

    camera_v2_head[0]  = 0xAAu;                     /* MCU -> 助手帧头 */
    camera_v2_head[1]  = 0u;                        /* check_sum，最后填写 */
    camera_v2_head[2]  = 0x02u;                     /* CAMERA 命令 */
    camera_v2_head[3]  = 0x03u;                     /* RGB565 / SCC8660 */
    camera_v2_head[4]  = (uint8)(SCC8660_W & 0xFFu);
    camera_v2_head[5]  = (uint8)(SCC8660_W >> 8);
    camera_v2_head[6]  = (uint8)(SCC8660_H & 0xFFu);
    camera_v2_head[7]  = (uint8)(SCC8660_H >> 8);
    camera_v2_head[8]  = 0x01u;                     /* 普通小端数据 */
    camera_v2_head[9]  = 0u;
    camera_v2_head[10] = 0u;
    camera_v2_head[11] = 0u;

    sum = camera_v2_head[0];
    for(i = 2u; i < sizeof(camera_v2_head); i++)
    {
        sum += camera_v2_head[i];
    }
    camera_v2_head[1] = sum;
}

#endif  /* AIMG_ENABLE */

/*-------------------------------------------------------------------------------------------------------------------
 * 初始化
 -----------------------------------------------------------------------------------------------------------------*/
void kart_assist_img_init(void)
{
#if AIMG_ENABLE
    tx_state   = AIMG_TX_IDLE;

    g_aimg_busy     = 0;
    g_aimg_progress = 0;

    /* 下载器虚拟串口就是 UART_0：P14.0=TX、P14.1=RX。图传只发送不接收，
     * 固定用 460800 8N1，把一帧 RGB565 的线速时间从约 3.33s 降到约 0.83s。
     * uart_init 会关闭 UART_0 RX 中断；这是有意的，因为日志在 UART_10，
     * 本功能也不需要从逐飞助手接收数据。 */
    uart_init(AIMG_UART, 460800u, UART0_TX_P14_0, UART0_RX_P14_1);

    /* 工程自带的是旧版 8B 图传库，而助手 V2.0.0.6 配套 Kart_TC387 例程使用
     * 12B 带校验头。这里构造 V2 头，不替换整套库，避免波及参数调试。 */
    aimg_build_v2_head();
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 请求发一帧
 -----------------------------------------------------------------------------------------------------------------*/
uint8 kart_assist_img_request(void)
{
#if AIMG_ENABLE
    /* 上一帧没发完就丢掉这次请求，不排队。
     * 排队会让上位机看到的图越来越滞后，而“图和手里板子的位置对不上”
     * 在标定时是致命的 —— 会以为阈值在飘。 */
    if(AIMG_TX_IDLE != tx_state)
    {
        g_aimg_img_skip++;
        return 0;
    }

    if(0 == kart_camera_frame_ready())
    {
        g_aimg_img_skip++;
        return 0;
    }

    /* 拷一份就放帧，别占着摄像头慢慢发（见 .h）。
     * 边拷边交换高低字节，理由见 .h 的 AIMG_BYTE_SWAP。
     * 合在这一趟里做：反正要逐字节搬 38400 个，不多一遍图像访问。 */
#if AIMG_BYTE_SWAP
    {
        const uint8 *src = (const uint8 *)scc8660_image[0];
        uint32 i;

        for(i = 0; i < (uint32)SCC8660_IMAGE_SIZE; i += 2u)
        {
            img_buf[i]      = src[i + 1u];
            img_buf[i + 1u] = src[i];
        }
    }
#else
    memcpy(img_buf, (const void *)scc8660_image[0], SCC8660_IMAGE_SIZE);
#endif
    kart_camera_frame_release();

    img_start_raw        = system_getval();
    tx_state             = AIMG_TX_PENDING;
    g_aimg_busy     = 1;
    g_aimg_progress = 0;
    img_offset           = 0;

    return 1;
#else
    return 0;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 后台推进。只能在主循环空转段调。
 -----------------------------------------------------------------------------------------------------------------*/
void kart_assist_img_background_poll(void)
{
#if AIMG_ENABLE
    uint32 send_t0;
    uint32 dt;

    if((AIMG_TX_PENDING != tx_state) && (AIMG_TX_SENDING != tx_state))
    {
        return;
    }

    /* 门禁只在请求处有是不够的:IDLE 里起的一帧会一路流穿整个
     * 跟随段。一旦离开 IDLE 就丢弃未发完的帧,阻塞写不得饿死控制环。 */
    if(MISSION_IDLE != kart_mission_get_mode())
    {
        tx_state         = AIMG_TX_IDLE;
        g_aimg_busy = 0;
        return;
    }

    send_t0 = system_getval();

    if(AIMG_TX_PENDING == tx_state)
    {
        /* 助手 V2：12B 配置头 + 38400B RGB565。
         * 160x120 时头应为 AA C8 02 03 A0 00 78 00 01 00 00 00。
         * 头很短，一次发完；图像数据在下面分块摊开（见 .h CHUNK_BYTES）。 */
        uart_write_buffer(AIMG_UART, camera_v2_head, sizeof(camera_v2_head));
        img_offset = 0;
        tx_state   = AIMG_TX_SENDING;
    }
    else
    {
        uint32 remain = (uint32)SCC8660_IMAGE_SIZE - img_offset;
        uint32 len    = (remain > AIMG_CHUNK_BYTES) ? (uint32)AIMG_CHUNK_BYTES : remain;

        uart_write_buffer(AIMG_UART, &img_buf[img_offset], len);
        img_offset          += len;
        g_aimg_progress = img_offset;

        if(img_offset >= (uint32)SCC8660_IMAGE_SIZE)
        {
            tx_state         = AIMG_TX_IDLE;
            g_aimg_busy = 0;
            g_aimg_img_sent++;
            g_aimg_img_ms = (system_getval() - img_start_raw) / AIMG_RAW_PER_MS;
        }
    }

    /* 取最大值而不是覆盖：要抓的是最坏那一块有没有超过 5ms 控制窗口。 */
    dt = (system_getval() - send_t0) / 100u;
    if(dt > g_aimg_chunk_max_us)
    {
        g_aimg_chunk_max_us = dt;
    }
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 查询忙闲
 -----------------------------------------------------------------------------------------------------------------*/
uint8 kart_assist_img_is_busy(void)
{
#if AIMG_ENABLE
    return (uint8)((AIMG_TX_IDLE != tx_state) ? 1 : 0);
#else
    return 0;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 放弃当前帧
 -----------------------------------------------------------------------------------------------------------------*/
void kart_assist_img_abort(void)
{
#if AIMG_ENABLE
    tx_state             = AIMG_TX_IDLE;
    g_aimg_busy     = 0;
    g_aimg_progress = 0;
#endif
}

#pragma section all restore
