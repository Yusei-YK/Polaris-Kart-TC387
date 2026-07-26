#include "kart_multicore.h"
#include "kart_imu.h"
#include "kart_odom.h"
#include "kart_record.h"
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
