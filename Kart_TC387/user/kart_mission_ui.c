#include "kart_mission_ui.h"

#include "kart_control.h"
#include "kart_encoder.h"
#include "kart_imu.h"
#include "kart_light.h"
#include "kart_mission.h"
#include "kart_steer_abs.h"

#include <stdio.h>
#include <string.h>

#define KART_MISSION_UI_LINE_CHARS  (39U)
#define KART_MISSION_UI_LINE_HEIGHT (16U)

static kart_subject_t kart_mission_ui_cursor = KART_SUBJECT_1_AUTO_DRIVE;
static uint16 kart_mission_ui_elapsed_ms;
static uint8 kart_mission_ui_page;
static uint8 kart_mission_ui_dirty;

/* 用固定宽度空格覆盖上一帧残留字符，避免每 100ms 全屏清屏产生闪烁。 */
static void kart_mission_ui_show_line(uint8 row, const char *text)
{
    char line[KART_MISSION_UI_LINE_CHARS + 1U];
    uint8 index = 0U;

    while(index < KART_MISSION_UI_LINE_CHARS && text[index] != '\0')
    {
        line[index] = text[index];
        index++;
    }

    while(index < KART_MISSION_UI_LINE_CHARS)
    {
        line[index++] = ' ';
    }
    line[KART_MISSION_UI_LINE_CHARS] = '\0';

    ips200_show_string(0U, (uint16)row * KART_MISSION_UI_LINE_HEIGHT, line);
}

static void kart_mission_ui_draw_mission(void)
{
    kart_mission_status_t status;
    kart_mission_request_t request;
    char line[48];

    kart_mission_get_status(&status);
    kart_mission_get_request(&request);

    kart_mission_ui_show_line(0U, "KART MISSION / IPS200");

    (void)sprintf(line, "> %s", kart_mission_subject_name(kart_mission_ui_cursor));
    kart_mission_ui_show_line(1U, line);

    (void)sprintf(line, "SELECT: %s", kart_mission_subject_name(status.subject));
    kart_mission_ui_show_line(2U, line);

    (void)sprintf(line, "RUN:    %s", kart_mission_run_name(status.run_state));
    kart_mission_ui_show_line(3U, line);

    (void)sprintf(line, "STAGE:  %s", kart_mission_stage_name());
    kart_mission_ui_show_line(4U, line);

    (void)sprintf(line, "READY:  %08lX", (unsigned long)status.ready_mask);
    kart_mission_ui_show_line(5U, line);

    (void)sprintf(line, "MISSING:%08lX", (unsigned long)status.missing_mask);
    kart_mission_ui_show_line(6U, line);

    (void)sprintf(line, "S2 TASK:%u/%u  ZONE:%u",
                  (unsigned int)status.subject2_tasks_done,
                  (unsigned int)KART_MISSION_S2_TOTAL_TASK_COUNT,
                  (unsigned int)status.subject2_zone_tasks_done);
    kart_mission_ui_show_line(7U, line);

    (void)sprintf(line, "TIME:   %lu ms", (unsigned long)status.total_elapsed_ms);
    kart_mission_ui_show_line(8U, line);

    (void)sprintf(line, "ERROR:  %u", (unsigned int)status.last_error);
    kart_mission_ui_show_line(9U, line);

    (void)sprintf(line, "REQUEST:%u", (unsigned int)request.motion);
    kart_mission_ui_show_line(10U, line);
    kart_mission_ui_show_line(11U, "ROTATE=SELECT  ENTER=ARM");
    kart_mission_ui_show_line(12U, "START=RUN      STOP=ABORT");
    kart_mission_ui_show_line(13U, "PAGE=LIVE DATA");
}

static void kart_mission_ui_draw_live(void)
{
    kart_mission_request_t request;
    char line[48];
    int32 yaw_x100 = (int32)(kart_imu_get_yaw() * 100.0f);
    int32 speed_target_x100 = (int32)(kart_control_get_target() * 100.0f);
    int32 speed_meas_x100 = (int32)(kart_control_get_meas() * 100.0f);

    kart_mission_get_request(&request);

    kart_mission_ui_show_line(0U, "KART LIVE DATA / IPS200");

    (void)sprintf(line, "YAW x100:    %ld", (long)yaw_x100);
    kart_mission_ui_show_line(1U, line);

    (void)sprintf(line, "STEER RAW:   %u", (unsigned int)kart_steer_abs_get_raw());
    kart_mission_ui_show_line(2U, line);

    (void)sprintf(line, "STEER DELTA: %d", (int)kart_steer_abs_get_center_delta());
    kart_mission_ui_show_line(3U, line);

    (void)sprintf(line, "TARGET x100: %ld", (long)speed_target_x100);
    kart_mission_ui_show_line(4U, line);

    (void)sprintf(line, "SPEED x100:  %ld", (long)speed_meas_x100);
    kart_mission_ui_show_line(5U, line);

    (void)sprintf(line, "MOTOR DUTY:  %d", (int)kart_control_get_output());
    kart_mission_ui_show_line(6U, line);

    (void)sprintf(line, "ENC LEFT:    %ld", (long)kart_encoder_get_left_sum());
    kart_mission_ui_show_line(7U, line);

    (void)sprintf(line, "ENC RIGHT:   %ld", (long)kart_encoder_get_right_sum());
    kart_mission_ui_show_line(8U, line);

    (void)sprintf(line, "LIGHT TASK:  %u", (unsigned int)request.light);
    kart_mission_ui_show_line(9U, line);

    (void)sprintf(line, "HORN TASK:   %u", (unsigned int)request.horn);
    kart_mission_ui_show_line(10U, line);

    (void)sprintf(line, "GATE TASK:   %u", (unsigned int)request.gate);
    kart_mission_ui_show_line(11U, line);

    (void)sprintf(line, "MOTION TASK: %u", (unsigned int)request.motion_task);
    kart_mission_ui_show_line(12U, line);
    (void)sprintf(line, "HW IMU:%u ABS:%u  PAGE=MISSION",
                  (unsigned int)kart_imu_is_ready(),
                  (unsigned int)kart_steer_abs_is_ready());
    kart_mission_ui_show_line(13U, line);
}

static void kart_mission_ui_refresh(void)
{
    if(kart_mission_ui_dirty != 0U)
    {
        ips200_full(RGB565_BLACK);
        kart_mission_ui_dirty = 0U;
    }

    if(kart_mission_ui_page == 0U)
    {
        kart_mission_ui_draw_mission();
    }
    else
    {
        kart_mission_ui_draw_live();
    }
}

void kart_mission_ui_init(void)
{
    kart_mission_ui_cursor = KART_SUBJECT_1_AUTO_DRIVE;
    kart_mission_ui_elapsed_ms = 0U;
    kart_mission_ui_page = 0U;
    kart_mission_ui_dirty = 1U;

    ips200_init(IPS200_TYPE_SPI);
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    kart_mission_ui_refresh();
}

void kart_mission_ui_poll(uint16 elapsed_ms)
{
    uint32 total_ms = (uint32)kart_mission_ui_elapsed_ms + (uint32)elapsed_ms;

    if(total_ms < KART_MISSION_UI_REFRESH_MS && kart_mission_ui_dirty == 0U)
    {
        kart_mission_ui_elapsed_ms = (uint16)total_ms;
        return;
    }

    kart_mission_ui_elapsed_ms = 0U;
    kart_mission_ui_refresh();
}

void kart_mission_ui_event(kart_mission_ui_event_t event)
{
    kart_mission_status_t status;

    kart_mission_get_status(&status);

    switch(event)
    {
        case KART_MISSION_UI_PREVIOUS:
            if(status.run_state == KART_MISSION_IDLE)
            {
                kart_mission_ui_cursor = (kart_mission_ui_cursor <= KART_SUBJECT_1_AUTO_DRIVE)
                    ? KART_SUBJECT_3_PATH_RETURN
                    : (kart_subject_t)(kart_mission_ui_cursor - 1);
            }
            break;

        case KART_MISSION_UI_NEXT:
            if(status.run_state == KART_MISSION_IDLE)
            {
                kart_mission_ui_cursor = (kart_mission_ui_cursor >= KART_SUBJECT_3_PATH_RETURN)
                    ? KART_SUBJECT_1_AUTO_DRIVE
                    : (kart_subject_t)(kart_mission_ui_cursor + 1);
            }
            break;

        case KART_MISSION_UI_ENTER:
            if(status.run_state == KART_MISSION_IDLE)
            {
                if(kart_mission_select_subject(kart_mission_ui_cursor) != 0U)
                {
                    (void)kart_mission_arm();
                }
            }
            break;

        case KART_MISSION_UI_START:
            (void)kart_mission_start();
            break;

        case KART_MISSION_UI_BACK:
            if(status.run_state == KART_MISSION_RUNNING)
            {
                kart_mission_abort();
            }
            else
            {
                kart_mission_reset();
            }
            break;

        case KART_MISSION_UI_STOP:
            kart_mission_abort();
            break;

        case KART_MISSION_UI_PAGE:
            kart_mission_ui_page ^= 1U;
            break;

        default:
            break;
    }

    kart_mission_ui_dirty = 1U;
}
