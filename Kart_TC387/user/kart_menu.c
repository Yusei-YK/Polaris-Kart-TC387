#include "kart_menu.h"
#include "zf_device_ips200.h"
#include "kart_mission.h"
#include "kart_flash.h"
#include "kart_record.h"
#include "kart_remote.h"
#include "kart_imu.h"
#include "kart_odom.h"
#include "kart_playback.h"
#include "board_pins.h"
#include "kart_voice.h"
#include "kart_horn.h"

typedef enum
{
    MENU_LEVEL_MAIN = 0,
    MENU_LEVEL_SUBJECT1,
    MENU_LEVEL_SUBJECT2,
    MENU_LEVEL_S1_SLOT_SAVE,
    MENU_LEVEL_S1_SLOT_LOAD,
    MENU_LEVEL_S2_VOICE,
    MENU_LEVEL_S2_GATE,
    MENU_LEVEL_S2_GATE_SLOT_SAVE,
    MENU_LEVEL_S2_GATE_SLOT_LOAD,
} menu_level_t;

typedef enum
{
    MENU_MAIN_SUBJECT1 = 0,
    MENU_MAIN_SUBJECT2,
    MENU_MAIN_MAX
} menu_main_item_t;

typedef enum
{
    MENU_S1_RECORD = 0,
    MENU_S1_PLAYBACK,
    MENU_S1_ENTER_REMOTE,
    MENU_S1_EXIT_REMOTE,
    MENU_S1_MAX
} menu_s1_item_t;

typedef enum
{
    MENU_S2_VOICE = 0,
    MENU_S2_GATE,
    MENU_S2_BACK,
    MENU_S2_MAX
} menu_s2_item_t;

typedef enum
{
    MENU_S2_GATE_RECORD = 0,
    MENU_S2_GATE_PLAYBACK,
    MENU_S2_GATE_BACK,
    MENU_S2_GATE_MAX
} menu_s2_gate_item_t;

typedef enum
{
    REC_STATE_IDLE = 0,
    REC_STATE_WAIT_START,
    REC_STATE_RECORDING,
    REC_STATE_SAVE_PROMPT,
} recording_state_t;

static menu_level_t current_level = MENU_LEVEL_MAIN;
static uint8 cursor_main = 0;
static uint8 cursor_s1 = 0;
static uint8 cursor_s2 = 0;
static uint8 cursor_s2_gate = 0;
static uint8 cursor_slot = 0;
static recording_state_t rec_state = REC_STATE_IDLE;

static uint8 key_mid_last = 1;
static uint8 key_up_last = 1;
static uint8 key_down_last = 1;
static uint8 key_left_last = 1;

static uint8 need_redraw = 1;

static void menu_draw_status_bar(void)
{
    char buf[32];
    float yaw = kart_imu_get_yaw();
    uint8 rc_online = kart_remote_is_online();
    kart_remote_sw3_t sw = kart_remote_get_sw3();
    const char* sw_str = (sw == KART_REMOTE_SW3_L) ? "L" :
                         (sw == KART_REMOTE_SW3_M) ? "M" : "H";

    sprintf(buf, "%.1f", yaw);
    ips200_show_string(0, 0, "Y:");
    ips200_show_string(24, 0, buf);

    ips200_show_string(100, 0, "RC:");
    ips200_show_string(132, 0, rc_online ? "ON" : "OFF");

    ips200_show_string(180, 0, "SW:");
    ips200_show_string(212, 0, sw_str);
}

static void menu_draw_main(void)
{
    ips200_show_string(0, 32, cursor_main == MENU_MAIN_SUBJECT1 ? "> " : "  ");
    ips200_show_string(32, 32, "Subject 1");

    ips200_show_string(0, 64, cursor_main == MENU_MAIN_SUBJECT2 ? "> " : "  ");
    ips200_show_string(32, 64, "Subject 2");
}

static void menu_draw_subject1(void)
{
    ips200_show_string(0, 32, cursor_s1 == MENU_S1_RECORD ? "> " : "  ");
    ips200_show_string(32, 32, "Record Path");

    ips200_show_string(0, 64, cursor_s1 == MENU_S1_PLAYBACK ? "> " : "  ");
    ips200_show_string(32, 64, "Playback Path");

    ips200_show_string(0, 96, cursor_s1 == MENU_S1_ENTER_REMOTE ? "> " : "  ");
    ips200_show_string(32, 96, "Enter Remote");

    ips200_show_string(0, 128, cursor_s1 == MENU_S1_EXIT_REMOTE ? "> " : "  ");
    ips200_show_string(32, 128, "Exit Remote");
}

static void menu_draw_subject2(void)
{
    ips200_show_string(0, 32, cursor_s2 == MENU_S2_VOICE ? "> " : "  ");
    ips200_show_string(16, 32, "Voice Control");

    ips200_show_string(0, 64, cursor_s2 == MENU_S2_GATE ? "> " : "  ");
    ips200_show_string(16, 64, "Gate Recording");

    ips200_show_string(0, 96, cursor_s2 == MENU_S2_BACK ? "> " : "  ");
    ips200_show_string(16, 96, "Back");
}

static void menu_draw_s2_voice(void)
{
    ips200_show_string(32, 80, "Speak Command");
    ips200_show_string(16, 112, "Press MID to Exit");
}

static void menu_draw_s2_gate(void)
{
    ips200_show_string(0, 32, cursor_s2_gate == MENU_S2_GATE_RECORD ? "> " : "  ");
    ips200_show_string(16, 32, "Record");

    ips200_show_string(0, 64, cursor_s2_gate == MENU_S2_GATE_PLAYBACK ? "> " : "  ");
    ips200_show_string(16, 64, "Playback");

    ips200_show_string(0, 96, cursor_s2_gate == MENU_S2_GATE_BACK ? "> " : "  ");
    ips200_show_string(16, 96, "Back");
}

static void menu_draw_recording_wait(void)
{
    ips200_show_string(32, 64, "Ready to Record");
    ips200_show_string(16, 96, "Drive to Start Pt");
    ips200_show_string(16, 128, "Press START to Rec");
}

static void menu_draw_recording_active(void)
{
    ips200_show_string(64, 96, "Recording...");
    ips200_show_string(16, 128, "Press START to Stop");
}

static void menu_draw_slot_save(void)
{
    char buf[48];
    ips200_show_string(16, 32, "Save to Slot:");

    for(uint8 i = 0; i < KART_MENU_S1_SLOT_NUM; i++)
    {
        uint16 count = kart_flash_slot_count(i);
        ips200_show_string(0, 64 + i * 32, cursor_slot == i ? "> " : "  ");
        if(count > 0)
            sprintf(buf, "Slot%d [%dpts]", i + 1, count);
        else
            sprintf(buf, "Slot%d [Empty]", i + 1);
        ips200_show_string(32, 64 + i * 32, buf);
    }

    ips200_show_string(0, 64 + KART_MENU_S1_SLOT_NUM * 32,
                      cursor_slot == KART_MENU_S1_SLOT_NUM ? "> " : "  ");
    ips200_show_string(32, 64 + KART_MENU_S1_SLOT_NUM * 32, "Don't Save");
}

static void menu_draw_slot_load(void)
{
    char buf[48];
    ips200_show_string(16, 32, "Select Slot:");

    for(uint8 i = 0; i < KART_MENU_S1_SLOT_NUM; i++)
    {
        uint16 count = kart_flash_slot_count(i);
        ips200_show_string(0, 64 + i * 32, cursor_slot == i ? "> " : "  ");
        if(count > 0)
            sprintf(buf, "Slot%d [%dpts]", i + 1, count);
        else
            sprintf(buf, "Slot%d [Empty]", i + 1);
        ips200_show_string(32, 64 + i * 32, buf);
    }

    ips200_show_string(16, 180, "Press LEFT to Back");
}

static void menu_draw_s2_gate_slot_save(void)
{
    char buf[48];
    ips200_show_string(16, 32, "Save Gate Path:");

    const char* slot_names[] = {"Gate1 Left", "Gate1", "Gate2", "Gate3", "Gate3 Right"};
    for(uint8 i = 0; i < KART_MENU_S2_GATE_SLOT_NUM; i++)
    {
        uint16 count = kart_flash_slot_count(i + 1);
        ips200_show_string(0, 64 + i * 24, cursor_slot == i ? "> " : "  ");
        if(count > 0)
            sprintf(buf, "%s [%dpts]", slot_names[i], count);
        else
            sprintf(buf, "%s [Empty]", slot_names[i]);
        ips200_show_string(32, 64 + i * 24, buf);
    }

    ips200_show_string(0, 64 + KART_MENU_S2_GATE_SLOT_NUM * 24,
                      cursor_slot == KART_MENU_S2_GATE_SLOT_NUM ? "> " : "  ");
    ips200_show_string(32, 64 + KART_MENU_S2_GATE_SLOT_NUM * 24, "Don't Save");
}

static void menu_draw_s2_gate_slot_load(void)
{
    char buf[48];
    ips200_show_string(16, 32, "Select Gate:");

    const char* slot_names[] = {"Gate1 Left", "Gate1", "Gate2", "Gate3", "Gate3 Right"};
    for(uint8 i = 0; i < KART_MENU_S2_GATE_SLOT_NUM; i++)
    {
        uint16 count = kart_flash_slot_count(i + 1);
        ips200_show_string(0, 64 + i * 24, cursor_slot == i ? "> " : "  ");
        if(count > 0)
            sprintf(buf, "%s [%dpts]", slot_names[i], count);
        else
            sprintf(buf, "%s [Empty]", slot_names[i]);
        ips200_show_string(32, 64 + i * 24, buf);
    }

    ips200_show_string(16, 200, "Press LEFT to Back");
}

static void menu_handle_key_mid_press(void)
{
    need_redraw = 1;
    switch(current_level)
    {
        case MENU_LEVEL_MAIN:
            if(cursor_main == MENU_MAIN_SUBJECT1)
            {
                current_level = MENU_LEVEL_SUBJECT1;
                cursor_s1 = 0;
            }
            else if(cursor_main == MENU_MAIN_SUBJECT2)
            {
                current_level = MENU_LEVEL_SUBJECT2;
                cursor_s2 = 0;
            }
            break;

        case MENU_LEVEL_SUBJECT1:
            if(cursor_s1 == MENU_S1_RECORD)
            {
                kart_mission_set_mode(MISSION_REMOTE);
                rec_state = REC_STATE_WAIT_START;
            }
            else if(cursor_s1 == MENU_S1_PLAYBACK)
            {
                current_level = MENU_LEVEL_S1_SLOT_LOAD;
                cursor_slot = 0;
            }
            else if(cursor_s1 == MENU_S1_ENTER_REMOTE)
            {
                kart_mission_set_mode(MISSION_REMOTE);
            }
            else if(cursor_s1 == MENU_S1_EXIT_REMOTE)
            {
                kart_mission_set_mode(MISSION_IDLE);
            }
            break;

        case MENU_LEVEL_SUBJECT2:
            if(cursor_s2 == MENU_S2_VOICE)
            {
                kart_odom_reset();
                current_level = MENU_LEVEL_S2_VOICE;
            }
            else if(cursor_s2 == MENU_S2_GATE)
            {
                current_level = MENU_LEVEL_S2_GATE;
                cursor_s2_gate = 0;
            }
            else if(cursor_s2 == MENU_S2_BACK)
            {
                current_level = MENU_LEVEL_MAIN;
            }
            break;

        case MENU_LEVEL_S2_VOICE:
            current_level = MENU_LEVEL_SUBJECT2;
            break;

        case MENU_LEVEL_S2_GATE:
            if(cursor_s2_gate == MENU_S2_GATE_RECORD)
            {
                current_level = MENU_LEVEL_S2_GATE_SLOT_SAVE;
                cursor_slot = 0;
            }
            else if(cursor_s2_gate == MENU_S2_GATE_PLAYBACK)
            {
                current_level = MENU_LEVEL_S2_GATE_SLOT_LOAD;
                cursor_slot = 0;
            }
            else if(cursor_s2_gate == MENU_S2_GATE_BACK)
            {
                current_level = MENU_LEVEL_SUBJECT2;
            }
            break;

        case MENU_LEVEL_S2_GATE_SLOT_SAVE:
            if(cursor_slot < KART_MENU_S2_GATE_SLOT_NUM)
            {
                uint8 ret = kart_record_save_to_flash(cursor_slot + 1);
                if(ret == 0)
                {
                    ips200_clear();
                    menu_draw_status_bar();
                    ips200_show_string(48, 96, "Saved OK!");
                    system_delay_ms(1000);
                }
            }
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_S2_GATE;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_LOAD:
            if(cursor_slot < KART_MENU_S2_GATE_SLOT_NUM)
            {
                uint16 count = kart_flash_slot_count(cursor_slot + 1);
                if(count > 0)
                {
                    kart_odom_reset();
                    kart_record_load_from_flash(cursor_slot + 1);
                    if(kart_mission_get_mode() == MISSION_REMOTE)
                        kart_mission_set_mode(MISSION_IDLE);
                    kart_mission_set_mode(MISSION_SUBJECT_2);
                    kart_playback_start();

                    ips200_clear();
                    menu_draw_status_bar();
                    ips200_show_string(32, 96, "Playback Started!");
                    system_delay_ms(1000);
                }
            }
            current_level = MENU_LEVEL_S2_GATE;
            break;

        case MENU_LEVEL_S1_SLOT_SAVE:
            if(cursor_slot < KART_MENU_S1_SLOT_NUM)
            {
                uint8 ret = kart_record_save_to_flash(cursor_slot);
                if(ret == 0)
                {
                    ips200_clear();
                    menu_draw_status_bar();
                    ips200_show_string(48, 96, "Saved OK!");
                    system_delay_ms(1000);
                }
            }
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_SUBJECT1;
            break;

        case MENU_LEVEL_S1_SLOT_LOAD:
            if(cursor_slot < KART_MENU_S1_SLOT_NUM)
            {
                uint16 count = kart_flash_slot_count(cursor_slot);
                if(count > 0)
                {
                    kart_odom_reset();
                    kart_record_load_from_flash(cursor_slot);
                    if(kart_mission_get_mode() == MISSION_REMOTE)
                        kart_mission_set_mode(MISSION_IDLE);
                    kart_mission_set_mode(MISSION_SUBJECT_1);
                    kart_playback_start();

                    ips200_clear();
                    menu_draw_status_bar();
                    ips200_show_string(32, 96, "Playback Started!");
                    system_delay_ms(1000);
                }
            }
            current_level = MENU_LEVEL_SUBJECT1;
            break;

        default:
            break;
    }
}

static void menu_handle_key_left_press(void)
{
    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
        return;

    need_redraw = 1;
    switch(current_level)
    {
        case MENU_LEVEL_SUBJECT1:
        case MENU_LEVEL_SUBJECT2:
            current_level = MENU_LEVEL_MAIN;
            break;

        case MENU_LEVEL_S2_VOICE:
            current_level = MENU_LEVEL_SUBJECT2;
            break;

        case MENU_LEVEL_S2_GATE:
            current_level = MENU_LEVEL_SUBJECT2;
            break;

        case MENU_LEVEL_S1_SLOT_SAVE:
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_SUBJECT1;
            break;

        case MENU_LEVEL_S1_SLOT_LOAD:
            current_level = MENU_LEVEL_SUBJECT1;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_SAVE:
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_S2_GATE;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_LOAD:
            current_level = MENU_LEVEL_S2_GATE;
            break;

        default:
            break;
    }
}

static void menu_handle_key_up_press(void)
{
    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
        return;

    need_redraw = 1;
    switch(current_level)
    {
        case MENU_LEVEL_MAIN:
            if(cursor_main > 0) cursor_main--;
            break;

        case MENU_LEVEL_SUBJECT1:
            if(cursor_s1 > 0) cursor_s1--;
            break;

        case MENU_LEVEL_SUBJECT2:
            if(cursor_s2 > 0) cursor_s2--;
            break;

        case MENU_LEVEL_S2_GATE:
            if(cursor_s2_gate > 0) cursor_s2_gate--;
            break;

        case MENU_LEVEL_S1_SLOT_SAVE:
            if(cursor_slot > 0) cursor_slot--;
            break;

        case MENU_LEVEL_S1_SLOT_LOAD:
            if(cursor_slot > 0) cursor_slot--;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_SAVE:
            if(cursor_slot > 0) cursor_slot--;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_LOAD:
            if(cursor_slot > 0) cursor_slot--;
            break;

        default:
            break;
    }
}

static void menu_handle_key_down_press(void)
{
    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
        return;

    need_redraw = 1;
    switch(current_level)
    {
        case MENU_LEVEL_MAIN:
            if(cursor_main < MENU_MAIN_MAX - 1) cursor_main++;
            break;

        case MENU_LEVEL_SUBJECT1:
            if(cursor_s1 < MENU_S1_MAX - 1) cursor_s1++;
            break;

        case MENU_LEVEL_SUBJECT2:
            if(cursor_s2 < MENU_S2_MAX - 1) cursor_s2++;
            break;

        case MENU_LEVEL_S2_GATE:
            if(cursor_s2_gate < MENU_S2_GATE_MAX - 1) cursor_s2_gate++;
            break;

        case MENU_LEVEL_S1_SLOT_SAVE:
            if(cursor_slot < KART_MENU_S1_SLOT_NUM) cursor_slot++;
            break;

        case MENU_LEVEL_S1_SLOT_LOAD:
            if(cursor_slot < KART_MENU_S1_SLOT_NUM - 1) cursor_slot++;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_SAVE:
            if(cursor_slot < KART_MENU_S2_GATE_SLOT_NUM) cursor_slot++;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_LOAD:
            if(cursor_slot < KART_MENU_S2_GATE_SLOT_NUM - 1) cursor_slot++;
            break;

        default:
            break;
    }
}

static void menu_handle_recording_mid_press(void)
{
    switch(rec_state)
    {
        case REC_STATE_WAIT_START:
            kart_record_start();
            rec_state = REC_STATE_RECORDING;
            need_redraw = 1;
            break;

        case REC_STATE_RECORDING:
            kart_record_stop();
            rec_state = REC_STATE_SAVE_PROMPT;
            current_level = MENU_LEVEL_S1_SLOT_SAVE;
            cursor_slot = 0;
            need_redraw = 1;
            break;

        default:
            break;
    }
}

static void menu_scan_keys(void)
{
    uint8 key_mid = gpio_get_level(KART_MENU_KEY_MID);
    uint8 key_up = gpio_get_level(KART_MENU_KEY_UP);
    uint8 key_down = gpio_get_level(KART_MENU_KEY_DOWN);
    uint8 key_left = gpio_get_level(KART_MENU_KEY_LEFT);

    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
    {
        if(key_mid == 0 && key_mid_last == 1)
            menu_handle_recording_mid_press();
    }
    else
    {
        if(key_mid == 0 && key_mid_last == 1)
            menu_handle_key_mid_press();
    }

    if(key_up == 0 && key_up_last == 1)
        menu_handle_key_up_press();

    if(key_down == 0 && key_down_last == 1)
        menu_handle_key_down_press();

    if(key_left == 0 && key_left_last == 1)
        menu_handle_key_left_press();

    key_mid_last = key_mid;
    key_up_last = key_up;
    key_down_last = key_down;
    key_left_last = key_left;
}

void kart_menu_init(void)
{
    gpio_init(KART_MENU_KEY_UP, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_DOWN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_MID, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_LEFT, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_RIGHT, GPI, 0, GPI_FLOATING_IN);

    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();

    current_level = MENU_LEVEL_MAIN;
    cursor_main = 0;
    rec_state = REC_STATE_IDLE;
    need_redraw = 1;
}

void kart_menu_poll(void)
{
    kart_voice_poll();
    kart_voice_dispatch();

    menu_scan_keys();

    if(rec_state == REC_STATE_RECORDING)
    {
        if(need_redraw)
        {
            ips200_clear();
            menu_draw_status_bar();
            menu_draw_recording_active();
            need_redraw = 0;
        }
        return;
    }

    if(kart_playback_is_running())
    {
        return;
    }

    if(need_redraw)
    {
        ips200_clear();
        menu_draw_status_bar();

        if(rec_state == REC_STATE_WAIT_START)
        {
            menu_draw_recording_wait();
        }
        else if(rec_state == REC_STATE_RECORDING)
        {
            menu_draw_recording_active();
        }
        else
        {
            switch(current_level)
            {
                case MENU_LEVEL_MAIN:
                    menu_draw_main();
                    break;
                case MENU_LEVEL_SUBJECT1:
                    menu_draw_subject1();
                    break;
                case MENU_LEVEL_SUBJECT2:
                    menu_draw_subject2();
                    break;
                case MENU_LEVEL_S2_VOICE:
                    menu_draw_s2_voice();
                    break;
                case MENU_LEVEL_S2_GATE:
                    menu_draw_s2_gate();
                    break;
                case MENU_LEVEL_S2_GATE_SLOT_SAVE:
                    menu_draw_s2_gate_slot_save();
                    break;
                case MENU_LEVEL_S2_GATE_SLOT_LOAD:
                    menu_draw_s2_gate_slot_load();
                    break;
                case MENU_LEVEL_S1_SLOT_SAVE:
                    menu_draw_slot_save();
                    break;
                case MENU_LEVEL_S1_SLOT_LOAD:
                    menu_draw_slot_load();
                    break;
                default:
                    break;
            }
        }

        need_redraw = 0;
    }
    else
    {
    }
}
