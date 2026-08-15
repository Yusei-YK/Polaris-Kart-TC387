#include "kart_multicore.h"
#include "kart_imu.h"
#include "kart_odom.h"
#include "kart_record.h"
#include "kart_vision.h"
#include "kart_preprocess.h"
#include "zf_device_dot_matrix_screen.h"

typedef enum
{
    KART_MC_CMD_NONE = 0,
    KART_MC_CMD_IMU_UPDATE,
    KART_MC_CMD_ODOM_UPDATE,
    KART_MC_CMD_RECORD_POLL,
    KART_MC_CMD_DOT_SHOW_STRING,
    KART_MC_CMD_DOT_SCAN
} kart_mc_command_t;

typedef struct
{
    volatile uint32 request_seq;
    volatile uint32 done_seq;
    volatile uint32 command;
    volatile char   text[4];
} kart_mc_channel_t;

typedef struct
{
    volatile uint8  initialized;
    volatile uint8  runtime_enabled;
    volatile uint8  online[4];
    volatile uint32 heartbeat[4];
    kart_mc_channel_t core1;
    kart_mc_channel_t core2;
    kart_mc_channel_t core3;
} kart_mc_shared_t;

/* DSRAM0 is visible through the SRI bus and is not a cached LMU alias. */
#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
static kart_mc_shared_t kart_mc_shared;

#if KART_MC_VISION_ENABLE
/*==================== core3 异步视觉的共享块 ====================
 * 全放 cpu0_dsram:它走 SRI 总线,不是 LMU 别名,两个核看到的是同一份。
 *
 * 【为什么要自己拷一份图】DMA 会持续往 scc8660_image 里写。如果直接把那个
 * 指针交给 core3,core3 算 360ms 期间 DMA 会改了它读的行,算出来的团块是
 * 跨了好几帧的碎片 —— 比丢帧更坏,因为它会给出一个看着合法的错结果。
 * 所以 CPU0 在投递时把 38400 字节整帧拷进 vis_img,然后立刻 frame_release()
 * 把缓冲还给 DMA。拷贝约 38400/4 次 4 字节写,几十微秒量级,可以进控制窗口。
 *
 * 【同步协议】单生产者(CPU0)单消费者(core3),只用两个序号,不用锁:
 *   CPU0: busy==0 时填 vis_img → __dsync() → req++ → busy=1
 *   core3: req!=ack 时算 → 写 result → __dsync() → ack=req → busy=0
 * CPU0 只在 busy==0 时写 vis_img,core3 只在 req!=ack 时读它,两者不重叠。 */
typedef struct
{
    volatile uint32 req;              /* CPU0 投递计数 */
    volatile uint32 ack;              /* core3 完成计数 */
    volatile uint8  busy;             /* 1 = core3 持有 vis_img,CPU0 不得写 */
    volatile int16  w;
    volatile int16  h;
    volatile uint32 frames;           /* 算完的帧数 */
    volatile uint32 last_us;          /* 上一帧耗时 */
    volatile uint32 max_us;           /* 历史最大耗时 */
    volatile uint32 reject;           /* 投递被拒次数(core3 忙) */
    kart_vision_result_t result;      /* core3 写,CPU0 读 */
    uint16 vis_img[KART_PREPROCESS_HEIGHT][KART_PREPROCESS_WIDTH];
}kart_mc_vision_t;

static kart_mc_vision_t kart_mc_vis;
#endif
#if defined(__TASKING__)
#pragma section all restore
#endif

static void kart_mc_clear_channel(kart_mc_channel_t *channel)
{
    channel->request_seq = 0;
    channel->done_seq = 0;
    channel->command = KART_MC_CMD_NONE;
    channel->text[0] = '\0';
    channel->text[1] = '\0';
    channel->text[2] = '\0';
    channel->text[3] = '\0';
}

void kart_multicore_init(void)
{
    uint8 i;

    kart_mc_shared.initialized = 0;
    kart_mc_shared.runtime_enabled = 0;
    for(i = 0; i < 4; i++)
    {
        kart_mc_shared.online[i] = 0;
        kart_mc_shared.heartbeat[i] = 0;
    }

    kart_mc_clear_channel(&kart_mc_shared.core1);
    kart_mc_clear_channel(&kart_mc_shared.core2);
    kart_mc_clear_channel(&kart_mc_shared.core3);
    kart_mc_shared.online[0] = 1;
    __dsync();
    kart_mc_shared.initialized = 1;
    __dsync();
}

void kart_multicore_mark_online(uint8 core_id)
{
    if(core_id < 4)
    {
        while(!kart_mc_shared.initialized)
        {
            /* CPU0 initializes the shared block before releasing the barrier. */
        }
        kart_mc_shared.online[core_id] = 1;
        __dsync();
    }
}

void kart_multicore_enable_runtime(void)
{
#if KART_MULTICORE_COMPAT_ENABLE
    while(!kart_mc_shared.online[1] ||
          !kart_mc_shared.online[2] ||
          !kart_mc_shared.online[3])
    {
        /* Existing startup already waits indefinitely for every configured core. */
    }
    __dsync();
    kart_mc_shared.runtime_enabled = 1;
    __dsync();
#else
    kart_mc_shared.runtime_enabled = 0;
#endif
}

uint8 kart_multicore_is_runtime_enabled(void)
{
    return kart_mc_shared.runtime_enabled;
}

static void kart_mc_request(kart_mc_channel_t *channel, kart_mc_command_t command)
{
    uint32 request = channel->request_seq + 1u;

    channel->command = (uint32)command;
    __dsync();
    channel->request_seq = request;
    __dsync();

    while(channel->done_seq != request)
    {
        /* Compatibility mode deliberately preserves synchronous ordering. */
    }
    __dsync();
}

void kart_multicore_imu_update(void)
{
    if(kart_mc_shared.runtime_enabled)
    {
        kart_mc_request(&kart_mc_shared.core1, KART_MC_CMD_IMU_UPDATE);
    }
    else
    {
        kart_imu_update();
    }
}

void kart_multicore_odom_update(void)
{
    if(kart_mc_shared.runtime_enabled)
    {
        kart_mc_request(&kart_mc_shared.core1, KART_MC_CMD_ODOM_UPDATE);
    }
    else
    {
        kart_odom_update();
    }
}

void kart_multicore_record_poll(void)
{
    if(kart_mc_shared.runtime_enabled)
    {
        kart_mc_request(&kart_mc_shared.core2, KART_MC_CMD_RECORD_POLL);
    }
    else
    {
        kart_record_poll();
    }
}

void kart_multicore_dot_show_string(const char *str)
{
    if(!kart_mc_shared.runtime_enabled)
    {
        dot_matrix_screen_show_string(str);
        return;
    }

    kart_mc_shared.core3.text[0] = (str != NULL && str[0] != '\0') ? str[0] : ' ';
    kart_mc_shared.core3.text[1] = (str != NULL && str[0] != '\0' && str[1] != '\0') ? str[1] : ' ';
    kart_mc_shared.core3.text[2] = (str != NULL && str[0] != '\0' && str[1] != '\0' && str[2] != '\0') ? str[2] : ' ';
    kart_mc_shared.core3.text[3] = '\0';
    __dsync();
    kart_mc_request(&kart_mc_shared.core3, KART_MC_CMD_DOT_SHOW_STRING);
}

void kart_multicore_dot_scan(void)
{
    if(kart_mc_shared.runtime_enabled)
    {
        kart_mc_request(&kart_mc_shared.core3, KART_MC_CMD_DOT_SCAN);
    }
    else
    {
        dot_matrix_screen_scan();
    }
}

#if KART_MC_VISION_ENABLE
/*==================== core3 异步视觉:CPU0 侧 ====================*/

uint8 kart_multicore_vision_submit(const uint16 *img, int16 w, int16 h)
{
    int32 n;
    int32 i;
    uint16 *dst;

    if((img == NULL) || (w <= 0) || (h <= 0)
       || (w > KART_PREPROCESS_WIDTH) || (h > KART_PREPROCESS_HEIGHT))
    {
        return 0;
    }

    /* core3 还在算上一帧:直接拒,不等。丢这一帧远好过卡住控制环。 */
    if(kart_mc_vis.busy)
    {
        kart_mc_vis.reject++;
        return 0;
    }

    n   = (int32)w * (int32)h;
    dst = &kart_mc_vis.vis_img[0][0];
    for(i = 0; i < n; i++)
    {
        dst[i] = img[i];
    }

    kart_mc_vis.w = w;
    kart_mc_vis.h = h;
    __dsync();
    kart_mc_vis.req++;
    kart_mc_vis.busy = 1;
    __dsync();
    return 1;
}

uint8 kart_multicore_vision_ready(void)
{
    return (uint8)((kart_mc_vis.busy == 0) && (kart_mc_vis.ack == kart_mc_vis.req)
                   && (kart_mc_vis.frames > 0u));
}

const kart_vision_result_t *kart_multicore_vision_get(void)
{
    __dsync();
    return (const kart_vision_result_t *)&kart_mc_vis.result;
}

uint32 kart_multicore_vision_frames(void)  { return kart_mc_vis.frames;  }
uint32 kart_multicore_vision_last_us(void) { return kart_mc_vis.last_us; }
uint32 kart_multicore_vision_max_us(void)  { return kart_mc_vis.max_us;  }
uint32 kart_multicore_vision_reject(void)  { return kart_mc_vis.reject;  }
#else
uint8 kart_multicore_vision_submit(const uint16 *img, int16 w, int16 h)
{
    (void)img; (void)w; (void)h;
    return 0;
}
uint8 kart_multicore_vision_ready(void) { return 0; }
const kart_vision_result_t *kart_multicore_vision_get(void)
{
    return kart_vision_get();
}
uint32 kart_multicore_vision_frames(void)  { return 0; }
uint32 kart_multicore_vision_last_us(void) { return 0; }
uint32 kart_multicore_vision_max_us(void)  { return 0; }
uint32 kart_multicore_vision_reject(void)  { return 0; }
#endif

static uint8 kart_mc_take_request(kart_mc_channel_t *channel, uint32 *request, kart_mc_command_t *command)
{
    uint32 current = channel->request_seq;

    if(current == channel->done_seq)
    {
        return 0;
    }

    __dsync();
    *request = current;
    *command = (kart_mc_command_t)channel->command;
    return 1;
}

static void kart_mc_complete(kart_mc_channel_t *channel, uint32 request, uint8 core_id)
{
    kart_mc_shared.heartbeat[core_id]++;
    __dsync();
    channel->done_seq = request;
    __dsync();
}

uint8 kart_multicore_core1_service(void)
{
    uint32 request;
    kart_mc_command_t command;

    if(!kart_mc_take_request(&kart_mc_shared.core1, &request, &command))
    {
        return 0;
    }

    if(command == KART_MC_CMD_IMU_UPDATE)
    {
        kart_imu_update();
    }
    else if(command == KART_MC_CMD_ODOM_UPDATE)
    {
        kart_odom_update();
    }

    kart_mc_complete(&kart_mc_shared.core1, request, 1);
    return 1;
}

uint8 kart_multicore_core2_service(void)
{
    uint32 request;
    kart_mc_command_t command;

    if(!kart_mc_take_request(&kart_mc_shared.core2, &request, &command))
    {
        return 0;
    }

    if(command == KART_MC_CMD_RECORD_POLL)
    {
        kart_record_poll();
    }

    kart_mc_complete(&kart_mc_shared.core2, request, 2);
    return 1;
}

uint8 kart_multicore_core3_service(void)
{
    uint32 request;
    kart_mc_command_t command;

#if KART_MC_VISION_ENABLE
    /* 视觉优先,而且不看 runtime_enabled —— 这条通道跟 COMPAT 无关。
     * 整条流水线(预处理 + 识别)都在 core3 上跑,CPU0 一个周期都不出。 */
    if(kart_mc_vis.req != kart_mc_vis.ack)
    {
        uint32 t0 = system_getval();
        uint32 us;
        const uint16 *pp;
        int16 w = kart_mc_vis.w;
        int16 h = kart_mc_vis.h;
        const kart_vision_result_t *r;

        pp = kart_preprocess_frame((const uint16 *)&kart_mc_vis.vis_img[0][0], w, h);
        r  = kart_vision_process(pp, w, h);

        kart_mc_vis.result = *r;

        us = (system_getval() - t0) / 100u;
        kart_mc_vis.last_us = us;
        if((us < 1000000u) && (us > kart_mc_vis.max_us))
        {
            kart_mc_vis.max_us = us;
        }
        kart_mc_vis.frames++;

        __dsync();
        kart_mc_vis.ack  = kart_mc_vis.req;
        kart_mc_vis.busy = 0;
        __dsync();
        return 1;
    }
#endif

    if(!kart_mc_take_request(&kart_mc_shared.core3, &request, &command))
    {
        return 0;
    }

    if(command == KART_MC_CMD_DOT_SHOW_STRING)
    {
        char text[4];
        text[0] = kart_mc_shared.core3.text[0];
        text[1] = kart_mc_shared.core3.text[1];
        text[2] = kart_mc_shared.core3.text[2];
        text[3] = '\0';
        dot_matrix_screen_show_string(text);
    }
    else if(command == KART_MC_CMD_DOT_SCAN)
    {
        dot_matrix_screen_scan();
    }

    kart_mc_complete(&kart_mc_shared.core3, request, 3);
    return 1;
}

uint32 kart_multicore_get_heartbeat(uint8 core_id)
{
    if(core_id < 4)
    {
        return kart_mc_shared.heartbeat[core_id];
    }
    return 0;
}
