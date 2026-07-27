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
#include "kart_params.h"

typedef enum
{
    MENU_LEVEL_MAIN = 0,
    MENU_LEVEL_SUBJECT1,
    MENU_LEVEL_SUBJECT2,
    MENU_LEVEL_S1_SLOT_SAVE,
    MENU_LEVEL_S1_SLOT_LOAD,
    MENU_LEVEL_S1_READY,
    MENU_LEVEL_S2_VOICE,
    MENU_LEVEL_S2_GATE,
    MENU_LEVEL_S2_GATE_SLOT_SAVE,
    MENU_LEVEL_S2_GATE_SLOT_LOAD,
    MENU_LEVEL_S4_RUN,
    MENU_LEVEL_SETTINGS,
} menu_level_t;

typedef enum
{
    MENU_MAIN_SUBJECT1 = 0,
    MENU_MAIN_SUBJECT2,
    MENU_MAIN_SUBJECT4,
    MENU_MAIN_SETTINGS,
    MENU_MAIN_MAX
} menu_main_item_t;

/* Settings 页一屏行数(参数比这多就滚屏)。
 * 页内项目 = KART_PARAM_MAX 个参数 + Save to Flash + Load Default。 */
#define MENU_SET_ROWS       (8)
#define MENU_SET_ITEM_SAVE  (KART_PARAM_MAX)
#define MENU_SET_ITEM_DEF   (KART_PARAM_MAX + 1)
#define MENU_SET_TOTAL      (KART_PARAM_MAX + 2)

/* 空闲自动落盘的等待拍数。kart_menu_poll 在 50ms 任务里跑 → 40 拍 ≈ 2s。
 * 给 2s 是为了让连续按 UP/DOWN 微调只在停手后擦写一次 DFlash。 */
#define MENU_PARAM_AUTOSAVE_TICKS   (40)

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
static uint8 cursor_set = 0;        /* Settings 页当前项 */
static uint8 set_top = 0;           /* Settings 页滚屏顶行下标 */
static uint8 set_edit = 0;          /* 1=编辑态,UP/DOWN 改值而不是移光标 */
static uint8 params_idle_cnt = 0;   /* 参数页无按键拍数,到阈值自动落盘 */
static recording_state_t rec_state = REC_STATE_IDLE;
static uint8 rec_target_is_gate = 0;    /* 0=科目一录制(存slot0), 1=门洞录制(停录后弹5槽菜单) */

static uint8 key_mid_last = 1;
static uint8 enc_sw_last = 1;       /* 旋钮按下键上一拍电平(与 MID 等价) */
static uint8 enc_a_last = 1;        /* 旋钮 A/B 相上一拍电平 */
static uint8 enc_b_last = 1;
static int8  enc_accum = 0;         /* 未攒满一格的边沿余数 */
static int8  enc_detent = 0;        /* 10ms 拍产出、50ms 拍取走的待处理格数 */
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

    /* 左对齐补空格:状态栏每拍刷新且不清屏,补空格覆盖上一帧残留字符(如 -180.0→5.6)。 */
    sprintf(buf, "%-8.1f", yaw);
    ips200_show_string(0, 0, "Y:");
    ips200_show_string(24, 0, buf);

    ips200_show_string(100, 0, "RC:");
    ips200_show_string(132, 0, rc_online ? "ON " : "OFF");

    ips200_show_string(180, 0, "SW:");
    ips200_show_string(212, 0, sw_str);
}

static void menu_draw_main(void)
{
    ips200_show_string(0, 32, cursor_main == MENU_MAIN_SUBJECT1 ? "> " : "  ");
    ips200_show_string(32, 32, "Subject 1");

    ips200_show_string(0, 64, cursor_main == MENU_MAIN_SUBJECT2 ? "> " : "  ");
    ips200_show_string(32, 64, "Subject 2");

    ips200_show_string(0, 96, cursor_main == MENU_MAIN_SUBJECT4 ? "> " : "  ");
    ips200_show_string(32, 96, "Subject 4");

    ips200_show_string(0, 128, cursor_main == MENU_MAIN_SETTINGS ? "> " : "  ");
    ips200_show_string(32, 128, "Settings");
}

/* 参数一行:"名字  值"。值按 meta.decimals 决定小数位,整数量(Vmax/Kp)不显示 .0。
 * 选中且在编辑态时用 "*" 光标,提示 UP/DOWN 此刻改的是值不是光标。 */
static void menu_draw_param_row(uint8 id, uint16 y, uint8 selected)
{
    const kart_param_meta_t *m = kart_params_meta(id);
    float v = kart_params_get(id);
    char buf[32];

    ips200_show_string(0, y, selected ? (set_edit ? "* " : "> ") : "  ");
    ips200_show_string(16, y, m->name);

    switch(m->decimals)
    {
        case 0:  sprintf(buf, "%-8.0f", v); break;
        case 1:  sprintf(buf, "%-8.1f", v); break;
        case 2:  sprintf(buf, "%-8.2f", v); break;
        default: sprintf(buf, "%-8.3f", v); break;
    }
    ips200_show_string(120, y, buf);
}

/* Settings 页:滚屏列表。MID 切"移光标/改值",LEFT 退出(编辑态先退编辑)。
 * 改完立即生效(存 RAM),Save to Flash 才写死;不存则下次上电回上次存的值。 */
static void menu_draw_settings(void)
{
    uint8 row, id;
    uint16 y;

    /* 提示里的"旋"=旋钮转动,与 UP/DOWN 等价;按旋钮=MID。 */
    ips200_show_string(0, 16, set_edit ? "SET(edit) knob=value"
                                       : "SET  push=edit LEFT=x");
    /* 行尾 * = 有改动还没进 Flash。按 LEFT 退出会自动存,存完这个星号消失。 */
    ips200_show_string(216, 16, kart_params_is_dirty() ? "*" : " ");

    for(row = 0; row < MENU_SET_ROWS; row++)
    {
        id = (uint8)(set_top + row);
        y  = (uint16)(40 + row * 20);

        if(id >= MENU_SET_TOTAL)
        {
            ips200_show_string(0, y, "                        ");
            continue;
        }

        if(id == MENU_SET_ITEM_SAVE)
        {
            ips200_show_string(0, y, cursor_set == id ? "> " : "  ");
            ips200_show_string(16, y, "Save to Flash       ");
        }
        else if(id == MENU_SET_ITEM_DEF)
        {
            ips200_show_string(0, y, cursor_set == id ? "> " : "  ");
            ips200_show_string(16, y, "Load Default        ");
        }
        else
        {
            menu_draw_param_row(id, y, (uint8)(cursor_set == id));
        }
    }
}

/* 光标移动后把它拉进可视窗口。 */
static void menu_settings_scroll(void)
{
    if(cursor_set < set_top) set_top = cursor_set;
    if(cursor_set >= set_top + MENU_SET_ROWS)
        set_top = (uint8)(cursor_set - MENU_SET_ROWS + 1);
}

/* 科目四运行界面:MID 让给 mission 当 START 键,只留 LEFT 退出。
 * 到停车区按一次 START 即直接开环倒车原路返回,车头不掉转、不搬车。
 * 按 mission 当前阶段显示,阶段跳变时由 kart_menu_poll 触发重绘。 */
static void menu_draw_s4_run(void)
{
    switch(kart_mission_get_subject4_stage())
    {
        case S4_PHASE1_RECORD:
            ips200_show_string(16, 48, "S4: Recording   ");
            ips200_show_string(16, 80, "RC drive the maze   ");
            ips200_show_string(16, 112, "At park:START back  ");
            ips200_show_string(16, 176, "Press LEFT to Exit  ");
            break;
        case S4_PHASE2_REVERSE:
            ips200_show_string(16, 48, "S4: Reversing...");
            ips200_show_string(16, 80, "Openloop back,no turn");
            ips200_show_string(16, 112, "                    ");
            ips200_show_string(16, 176, "                    ");
            break;
        case S4_FINISHED:
            ips200_show_string(16, 48, "S4: Finished    ");
            ips200_show_string(16, 80, "Back at start       ");
            ips200_show_string(16, 112, "                    ");
            ips200_show_string(16, 176, "Press LEFT to Exit  ");
            break;
        case S4_FAULT:
        default:
            ips200_show_string(16, 48, "S4: FAULT       ");
            ips200_show_string(16, 80, "Path invalid        ");
            ips200_show_string(16, 112, "                    ");
            ips200_show_string(16, 176, "Press LEFT to Exit  ");
            break;
    }
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

static void menu_draw_s1_ready(void)
{
    char buf[32];
    ips200_show_string(24, 64, "Loaded, at Start Pt");
    ips200_show_string(16, 96, "Press START to Go");
    ips200_show_string(16, 128, "Press LEFT to Back");

    /* 就地显示当前复现胆量,UP/DOWN 直接改。改完按 START 才生效。
     * Prof=OFF 时倍率不起作用(速度仍取录制速度),所以一并把开关状态显示出来,
     * 免得在场上狂按 UP 却毫无变化。 */
    /* 末尾 * = 改了还没进 Flash;按 LEFT 退出这一页会自动存(存完星号消失)。 */
    sprintf(buf, "Speed x%-5.2f UP/DN %c", kart_params_get(KART_PARAM_PB_SCALE),
            kart_params_is_dirty() ? '*' : ' ');
    ips200_show_string(16, 160, buf);
    ips200_show_string(16, 184, (kart_params_get(KART_PARAM_PB_PROF) > 0.5f)
                                ? "Profile: ON " : "Profile: OFF");
}

/* 离开可调参界面时自动落盘。只有改过才擦写 DFlash(数 ms 阻塞),没改一个字节不写。
 * 只在静止界面调用:车在跑时 kart_menu_poll 已提前 return,按键进不来。
 * 写了才闪提示,让人在按 reset 之前确认值已经进 Flash。 */
static void menu_params_autosave(void)
{
    if(kart_params_save_if_dirty())
    {
        ips200_clear();
        menu_draw_status_bar();
        ips200_show_string(48, 96, "Params Saved!");
        system_delay_ms(400);
    }
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
            else if(cursor_main == MENU_MAIN_SETTINGS)
            {
                current_level = MENU_LEVEL_SETTINGS;
                cursor_set = 0;
                set_top = 0;
                set_edit = 0;
            }
            else if(cursor_main == MENU_MAIN_SUBJECT4)
            {
                /* 进科目四即发车:同科目三录制入口(enter 清 odom+开录制+遥控接管)。
                 * 遥控开车走迷宫,到停车区按一次物理 START 键 → 停录并直接开环倒车返回,
                 * 车头不掉转。停 S4_RUN 屏:菜单不吃 MID(防运行中刷屏),发车用独立 START 键。 */
                kart_mission_set_mode(MISSION_SUBJECT_4);
                current_level = MENU_LEVEL_S4_RUN;
            }
            break;

        case MENU_LEVEL_SUBJECT1:
            if(cursor_s1 == MENU_S1_RECORD)
            {
                kart_mission_set_mode(MISSION_REMOTE);
                rec_target_is_gate = 0;
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
                /* 真正进科目二状态机:subject2_loop 才会每拍跑 voice_dispatch+motion_update,
                 * 运动指令的判停/deadman急停/蛇形翻打角靠它推进。只切菜单界面车会裸奔。 */
                kart_mission_set_mode(MISSION_SUBJECT_2);
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
            /* 退出语音:完整停机(mission_stop_all 停运动+关环+清语音队列+静音)。 */
            kart_mission_set_mode(MISSION_IDLE);
            current_level = MENU_LEVEL_SUBJECT2;
            break;

        case MENU_LEVEL_SETTINGS:
            if(cursor_set == MENU_SET_ITEM_SAVE)
            {
                /* 阻塞擦写一页 DFlash(数 ms)。Settings 页只在静止时进得来,
                 * 车在跑(playback_is_running)时 kart_menu_poll 早就 return 了,
                 * 按键根本进不到这里,不会在运行中插入 Flash 擦写。 */
                kart_params_save();
                ips200_clear();
                menu_draw_status_bar();
                ips200_show_string(48, 96, "Params Saved!");
                system_delay_ms(800);
            }
            else if(cursor_set == MENU_SET_ITEM_DEF)
            {
                kart_params_load_default();
                ips200_clear();
                menu_draw_status_bar();
                ips200_show_string(40, 96, "Default Loaded");
                system_delay_ms(800);
            }
            else
            {
                set_edit = set_edit ? 0 : 1;     /* MID 切换 移光标/改值 */
            }
            break;

        case MENU_LEVEL_S2_GATE:
            if(cursor_s2_gate == MENU_S2_GATE_RECORD)
            {
                kart_mission_set_mode(MISSION_REMOTE);
                rec_target_is_gate = 1;
                rec_state = REC_STATE_WAIT_START;
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
                uint16 loaded = kart_record_load_from_flash(cursor_slot);
                if(loaded >= 2)
                {
                    kart_odom_reset();
                    /* 强制重新进入科目一，确保阶段回到WAIT_START。 */
                    if(kart_mission_get_mode() != MISSION_IDLE)
                        kart_mission_set_mode(MISSION_IDLE);
                    kart_mission_set_mode(MISSION_SUBJECT_1);

                    ips200_clear();
                    menu_draw_status_bar();
                    ips200_show_string(24, 96, "Loaded, Press START");
                    system_delay_ms(1000);
                    /* 停在专用就绪界面:菜单不吃 MID,按独立 START 键发车。 */
                    current_level = MENU_LEVEL_S1_READY;
                }
                else
                {
                    ips200_clear();
                    menu_draw_status_bar();
                    ips200_show_string(40, 96, "Load Failed!");
                    system_delay_ms(1000);
                    current_level = MENU_LEVEL_SUBJECT1;
                }
            }
            else
            {
                current_level = MENU_LEVEL_SUBJECT1;
            }
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
            /* LEFT 退出语音同样完整停机。 */
            kart_mission_set_mode(MISSION_IDLE);
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

        case MENU_LEVEL_SETTINGS:
            /* 编辑态先退编辑,再按一次才退出页面(防误触直接跳走)。 */
            if(set_edit) set_edit = 0;
            else
            {
                /* 退出参数页自动落盘:改过才写。这样断电/按 reset 重置 IMU
                 * 都不会丢刚调的值,不必记得去点 Save to Flash。 */
                menu_params_autosave();
                current_level = MENU_LEVEL_MAIN;
            }
            break;

        case MENU_LEVEL_S1_READY:
            /* 就绪界面 LEFT 退回:一并退出科目一,防遥控/复现残留。 */
            if(kart_mission_get_mode() != MISSION_IDLE)
                kart_mission_set_mode(MISSION_IDLE);
            /* 这一页 UP/DOWN 能热调 PB Scale,退出时同样落盘。 */
            menu_params_autosave();
            current_level = MENU_LEVEL_SUBJECT1;
            break;

        case MENU_LEVEL_S4_RUN:
            /* 科目四运行界面 LEFT 退出:完整停机退回 IDLE,防遥控/录制/复现残留。 */
            if(kart_mission_get_mode() != MISSION_IDLE)
                kart_mission_set_mode(MISSION_IDLE);
            current_level = MENU_LEVEL_MAIN;
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

        case MENU_LEVEL_SETTINGS:
            if(set_edit) kart_params_step(cursor_set, +1);   /* 编辑态:加一步 */
            else if(cursor_set > 0) { cursor_set--; menu_settings_scroll(); }
            break;

        /* 科目一就绪界面:UP/DOWN 热调复现速度倍率(PB Scale)。
         * 场地上试速度不用退菜单:UP 加胆量、DOWN 减胆量,再按 START 跑一趟。
         * 只改 RAM 值,满意了再进 Settings 存 Flash。剖面在 playback_start 时重算,
         * 故改完必须重跑才生效(跑动中不会中途变速)。
         * 科目四界面不挂这个:它走开环倒车固定速,不用剖面,且流程已自动推进,
         * 不在这里插任何按键行为。 */
        case MENU_LEVEL_S1_READY:
            kart_params_step(KART_PARAM_PB_SCALE, +1);
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

        case MENU_LEVEL_SETTINGS:
            if(set_edit) kart_params_step(cursor_set, -1);   /* 编辑态:减一步 */
            else if(cursor_set < MENU_SET_TOTAL - 1) { cursor_set++; menu_settings_scroll(); }
            break;

        case MENU_LEVEL_S1_READY:
            kart_params_step(KART_PARAM_PB_SCALE, -1);       /* 见 UP 处注释 */
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
            current_level = rec_target_is_gate ? MENU_LEVEL_S2_GATE_SLOT_SAVE
                                               : MENU_LEVEL_S1_SLOT_SAVE;
            cursor_slot = 0;
            need_redraw = 1;
            break;

        default:
            break;
    }
}

/* 旋转编码器软件正交解码。由 10ms 拍调(kart_menu_enc_poll),不能放 50ms 拍:
 * EC11 一个机械档位 4 个正交边沿,手旋约 10 格/秒时单相约 25ms 一变,
 * 50ms 采样必漏边沿 → 计数丢一半甚至判错方向。10ms(100Hz)有 2 倍余量。
 * P11.2/11.3 不在硬件编码器定时器候选,只能软件读。
 *
 * 方向只由"哪一相先变 + 另一相当前电平"决定,漏采顶多少走一格,不会反向。
 * 攒够 KART_MENU_ENC_DIV 个边沿才产出一格,余数留在 enc_accum,慢旋不丢。 */
void kart_menu_enc_poll(void)
{
    uint8 a = gpio_get_level(KART_MENU_ENC_A);
    uint8 b = gpio_get_level(KART_MENU_ENC_B);

    if(a != enc_a_last)
    {
        /* A 相跳变:与 B 同电平算一个方向,异电平算另一个方向。 */
        enc_accum = (int8)(enc_accum + ((a == b) ? -1 : +1));
        enc_a_last = a;
    }

    if(b != enc_b_last)
    {
        enc_accum = (int8)(enc_accum + ((a == b) ? +1 : -1));
        enc_b_last = b;
    }

    while(enc_accum >= KART_MENU_ENC_DIV)
    {
        enc_accum = (int8)(enc_accum - KART_MENU_ENC_DIV);
        if(enc_detent < KART_MENU_ENC_PEND_MAX) enc_detent++;
    }
    while(enc_accum <= -KART_MENU_ENC_DIV)
    {
        enc_accum = (int8)(enc_accum + KART_MENU_ENC_DIV);
        if(enc_detent > -KART_MENU_ENC_PEND_MAX) enc_detent--;
    }
}

static void menu_scan_keys(void)
{
    uint8 key_mid = gpio_get_level(KART_MENU_KEY_MID);
    uint8 key_up = gpio_get_level(KART_MENU_KEY_UP);
    uint8 key_down = gpio_get_level(KART_MENU_KEY_DOWN);
    uint8 key_left = gpio_get_level(KART_MENU_KEY_LEFT);
    uint8 key_esw = gpio_get_level(KART_MENU_ENC_SW);
    uint8 mid_edge;
    /* 取走 10ms 拍攒下的格数。两者都在主循环上下文(10ms/50ms 拍同一个 for 循环里
     * 顺序分发),不是中断,不存在读改写竞争,不需要临界区。 */
    int8  enc = enc_detent;
    enc_detent = 0;

    /* 旋钮按下(P20.6)与五向 MID(P33.4)等价,取"任一下降沿"。
     * 两个脚独立,不会互相影响;哪个手顺用哪个。 */
    mid_edge = ((key_mid == 0 && key_mid_last == 1) ||
                (key_esw == 0 && enc_sw_last  == 1)) ? 1u : 0u;

    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
    {
        if(mid_edge)
            menu_handle_recording_mid_press();
    }
    else if(current_level == MENU_LEVEL_S1_READY || current_level == MENU_LEVEL_S4_RUN)
    {
        /* 这两个界面 MID 无对应菜单项,但 menu_handle_key_mid_press 一进去就
         * 置 need_redraw → 会在等发车/倒车推进期间插一次 IPS200 整屏刷新
         * (2026-07-27 已定过:运行中一律不刷屏)。所以直接吞掉。
         * 注意与旧板不同,现在吞的原因只是"防无谓刷屏",不再是引脚共用 ——
         * 新板 START 是独立的 P20.7,菜单 MID 是 P33.4。 */
    }
    else
    {
        if(mid_edge)
            menu_handle_key_mid_press();
    }

    if(key_up == 0 && key_up_last == 1)
        menu_handle_key_up_press();

    if(key_down == 0 && key_down_last == 1)
        menu_handle_key_down_press();

    if(key_left == 0 && key_left_last == 1)
        menu_handle_key_left_press();

    /* 旋钮转动 = 连按 UP/DOWN。走同一套处理函数,行为与按键完全一致
     * (含编辑态改值、S1_READY 热调速度倍率)。 */
    while(enc > 0) { menu_handle_key_up_press();   enc--; }
    while(enc < 0) { menu_handle_key_down_press(); enc++; }

    /* 任一输入活动都重新计时:连续调值期间不落盘,停手 2s 后才写一次 Flash。 */
    if(key_mid == 0 || key_up == 0 || key_down == 0 || key_left == 0 || key_esw == 0)
        params_idle_cnt = 0;

    key_mid_last = key_mid;
    key_up_last = key_up;
    key_down_last = key_down;
    key_left_last = key_left;
    enc_sw_last = key_esw;

    /* RIGHT(P21.7)刻意不绑动作:新板它可用,但"右=进入"与 MID 重复,
     * 而误触"进入"会直接执行菜单项(发车/擦写 Flash),不如留空。
     * 需要时把它接到 menu_handle_key_mid_press 即可(记得加 key_right_last)。 */
}

void kart_menu_init(void)
{
    /* 上拉在主板侧(按键板 R1 只是 LED1 限流,板上不带按键上拉),故这里用浮空输入,
     * 不再叠一层片内上拉。三个操作件公共端都接 GND → 按下/导通读 0。
     * 若换到没有外部上拉的主板,把这 8 个改成 GPI_PULL_UP 即可。 */
    gpio_init(KART_MENU_KEY_UP, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_DOWN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_MID, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_LEFT, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_KEY_RIGHT, GPI, 0, GPI_FLOATING_IN);

    gpio_init(KART_MENU_ENC_A,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_ENC_B,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_MENU_ENC_SW, GPI, 0, GPI_FLOATING_IN);

    /* 按当前真实电平初始化"上一拍"记录,防上电瞬间被当成一次按下/转动:
     * 若某键上电时正被按住,记 0 就不会产生下降沿。 */
    key_up_last   = gpio_get_level(KART_MENU_KEY_UP);
    key_down_last = gpio_get_level(KART_MENU_KEY_DOWN);
    key_mid_last  = gpio_get_level(KART_MENU_KEY_MID);
    key_left_last = gpio_get_level(KART_MENU_KEY_LEFT);
    enc_sw_last   = gpio_get_level(KART_MENU_ENC_SW);
    enc_a_last    = gpio_get_level(KART_MENU_ENC_A);
    enc_b_last    = gpio_get_level(KART_MENU_ENC_B);
    enc_accum     = 0;
    enc_detent    = 0;

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
    /* 语音收帧/分发已交给 subject2_loop 独占(进 Voice Control 会切 MISSION_SUBJECT_2)。
     * 此处不再调 voice_poll/dispatch,避免与 subject2_loop 双份分发抢同一队列。 */
    menu_scan_keys();

    /* 科目四阶段跳变不再触发重绘(2026-07-27):停录/倒车已改成自动推进,阶段会
     * 自己往下跳,屏幕跟着刷等于在录制和倒车过程中插入 IPS200 SPI 写(整屏 clear
     * + 多行字符),占住控制窗口。运行中一律不刷屏,进度看串口遥测。
     * 进 S4 运行界面时的首屏由 need_redraw 正常画一次,之后保持静止。 */

    if(rec_state == REC_STATE_RECORDING)
    {
        if(need_redraw)
        {
            ips200_clear();
            menu_draw_recording_active();
            need_redraw = 0;
        }
        menu_draw_status_bar();     /* 每拍刷状态栏,IMU yaw 实时更新,不靠按键 */
        return;
    }

    if(kart_playback_is_running())
    {
        return;
    }

    /* 空闲自动落盘:在 Settings / S1_READY 这两个静止页面上,最后一次按键后
     * 约 2s 无操作就把改动写进 DFlash。补的是"调完速度直接按 START 跑,没按
     * LEFT 退出"这条路径 —— 否则那次改动在 reset(重置 IMU)后就没了。
     * 这里已经在 RECORDING / playback_is_running 两个 return 之后,车必定静止;
     * 且只有 dirty=1 才真写,连续微调也只在停手后写一次。 */
    if(current_level == MENU_LEVEL_SETTINGS || current_level == MENU_LEVEL_S1_READY)
    {
        if(kart_params_is_dirty())
        {
            if(params_idle_cnt < MENU_PARAM_AUTOSAVE_TICKS) params_idle_cnt++;

            if(params_idle_cnt >= MENU_PARAM_AUTOSAVE_TICKS)
            {
                kart_params_save_if_dirty();
                need_redraw = 1;        /* 重画,把标脏的 '*' 抹掉 */
            }
        }
    }

    if(need_redraw)
    {
        ips200_clear();

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
                case MENU_LEVEL_S1_READY:
                    menu_draw_s1_ready();
                    break;
                case MENU_LEVEL_S4_RUN:
                    menu_draw_s4_run();
                    break;
                case MENU_LEVEL_SETTINGS:
                    menu_draw_settings();
                    break;
                default:
                    break;
            }
        }

        need_redraw = 0;
    }

    /* 状态栏每拍刷新(放 clear+主体之后,不会被 clear 擦掉);
     * 主体仍只在 need_redraw 时重绘,避免每拍全屏 clear 拖慢调度器。 */
    menu_draw_status_bar();
}
