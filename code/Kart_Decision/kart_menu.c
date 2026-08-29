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
#include "kart_camera.h"       /* 摄像头调试页:取帧/还帧/链路诊断 */
#include "kart_vision.h"       /* 摄像头调试页:黄色引导板检测结果 */
#include "kart_follow.h"       /* 科目三实时页:跟随下发的打角/速度(判符号用) */
#include "kart_person_link.h"  /* 科目三 PLINK 页:链路状态 + 原始帧（无图像）*/
#include "kart_traj_view.h"    /* 录制路径采样点可视化 */
#include "kart_boot_anim.h"    /* 白底开机帧动画 */
#include "kart_multicore.h"  /* core2 屏幕绘制代理 draw_* */

typedef enum
{
    MENU_LEVEL_MAIN = 0,
    MENU_LEVEL_SUBJECT1,
    MENU_LEVEL_SUBJECT2,
    MENU_LEVEL_S1_SLOT_SAVE,
    MENU_LEVEL_S1_SLOT_LOAD,
    MENU_LEVEL_S1_READY,
    MENU_LEVEL_S1_TRAJ,
    MENU_LEVEL_S2_VOICE,
    MENU_LEVEL_S2_GATE,
    MENU_LEVEL_S2_GATE_SLOT_SAVE,
    MENU_LEVEL_S2_GATE_SLOT_LOAD,
    MENU_LEVEL_S2_RET,
    MENU_LEVEL_S2_RET_SLOT_SAVE,
    MENU_LEVEL_S2_RET_SLOT_LOAD,
    MENU_LEVEL_S3_RUN,
    MENU_LEVEL_CAMERA,
    MENU_LEVEL_SETTINGS,
} menu_level_t;

typedef enum
{
    MENU_MAIN_SUBJECT1 = 0,
    MENU_MAIN_SUBJECT2,
    MENU_MAIN_SUBJECT3,
    MENU_MAIN_CAMERA,
    MENU_MAIN_SETTINGS,
    MENU_MAIN_MAX
} menu_main_item_t;

/* -------------------- Settings 页行表 --------------------
 * 改之前 Settings 是"按 param_id_t 顺序铺 24 行 + 两个动作项",
 * cursor_set 既是行号又是参数 id。问题是 enum 顺序 = 参数被加进来的时间顺序,
 * 同一类量散在表的两头(限幅项全挤在末尾),现场调一次参要来回滚屏。
 *
 * 现在改成一张显示顺序表:行号 → (参数 id | 分组标题 | 动作)。
 * 【关键】enum 值一个都没动 —— Flash 布局是 word[2+id],改 enum 会让旧存档错位
 * (kart_params.c 的 count 校验只挡加减参数,挡不住重排)。这里只动"怎么显示"。
 *
 * 分组标题行不可选中,UP/DOWN 会跳过(见 menu_settings_seek)。 */
#define SET_ROW_HEAD    (0xFFu)     /* 分组标题,不可选中 */
#define SET_ROW_SAVE    (0xFEu)     /* 动作:存 Flash */
#define SET_ROW_DEF     (0xFDu)     /* 动作:恢复出厂 */

typedef struct
{
    uint8       id;         /* 参数 id,或上面三个 SET_ROW_* 标记 */
    const char *text;       /* 仅标题/动作行用;参数行为 NULL(名字取 meta) */
} menu_set_row_t;

static const menu_set_row_t set_rows[] =
{
    { SET_ROW_HEAD,             "-- PLAYBACK --"  },
    { PARAM_PB_PROF,       NULL              },
    { PARAM_PB_SCALE,      NULL              },
    { PARAM_PB_VMAX,       NULL              },
    { PARAM_PB_VMIN,       NULL              },
    { PARAM_PB_ALAT,       NULL              },
    { PARAM_PB_CLAMP,      NULL              },
    { PARAM_PB_ABRAKE,     NULL              },
    { PARAM_PB_LDGAIN,     NULL              },
    { PARAM_PB_LDMAX,      NULL              },

    { SET_ROW_HEAD,             "-- LIMITS --"    },
    { PARAM_SPD_KP,        NULL              },
    { PARAM_SPD_IMAX,      NULL              },
    { PARAM_STR_OUTMAX,    NULL              },
    { PARAM_SLEW_REAR,     NULL              },
    { PARAM_RAMP,          NULL              },

    { SET_ROW_HEAD,             "-- S3 REVERSE --"},
    { PARAM_S3_OL_SPD,     NULL              },
    { PARAM_S3_OL_MODE,    NULL              },
    { PARAM_S3_OL_KH,      NULL              },
    { PARAM_S3_OL_KE,      NULL              },
    { PARAM_S3_LEAD,       NULL              },
    { PARAM_PB_REVSCL,     NULL              },
    { PARAM_PB_REVCORR,    NULL              },

    { SET_ROW_HEAD,             "-- MISC --"      },
    { PARAM_RC_VMAX,       NULL              },
    { PARAM_HEAD_KP,       NULL              },
    { PARAM_FLW_CRUZ,      NULL              },

    { SET_ROW_HEAD,             "-- ACTIONS --"   },
    { SET_ROW_SAVE,             "Save to Flash"   },
    { SET_ROW_DEF,              "Load Default"    },
};

#define MENU_SET_TOTAL  ((uint8)(sizeof(set_rows) / sizeof(set_rows[0])))

/* 一屏行数。UI_Y_ROW0=70,行距 20 → 第 11 行占 270..285,仍在 UI_Y_HINT(296) 之上。
 * 加了分组标题后表变长,多铺 3 行就少滚一屏。 */
#define MENU_SET_ROWS       (11)

/* 空闲自动落盘的等待拍数。kart_menu_poll 在 50ms 任务里跑 → 40 拍 ≈ 2s。
 * 给 2s 是为了让连续按 UP/DOWN 微调只在停手后擦写一次 DFlash。 */
#define MENU_PARAM_AUTOSAVE_TICKS   (40)

/* -------------------- 长按/快旋加速(单位 = 输入扫描拍 = 10ms) --------------------
 * kart_menu_input_poll 在 10ms 拍跑,所以这些常数都以 10ms 为单位。
 * 按住不放:先出一次(细调) → 300ms 后以 10 次/秒重复 → 1s 后每次走 10 个步长。
 * 这样"点一下=最小改动、按住=快速扫过量程"两件事用一个键都能干。 */
#define MENU_KEY_REPEAT_DELAY   (30)    /* 按住 300ms 才开始自动重复,防手抖连发 */
#define MENU_KEY_REPEAT_PERIOD  (10)    /* 重复间隔 100ms */
#define MENU_KEY_REPEAT_FAST    (100)   /* 按住 1s 之后切粗调 */
#define MENU_KEY_STEP_FAST      (10)    /* 粗调倍率:一次 10 个步长 */
#define MENU_KEY_HOLD_MAX       (30000) /* hold 计数封顶,防 uint16 回绕打乱重复相位 */

/* 旋钮:相邻两格间隔 ≤60ms 判为"快旋",编辑态直接上粗调倍率。
 * 慢慢拧一格一格微调、猛拧几圈快速到位,不用切档也不用长按。 */
#define MENU_ENC_FAST_GAP       (6)

typedef enum
{
    MENU_S1_RECORD = 0,
    MENU_S1_PLAYBACK,
    MENU_S1_VIEW_PATH,
    MENU_S1_ENTER_REMOTE,
    MENU_S1_EXIT_REMOTE,
    MENU_S1_MAX
} menu_s1_item_t;

typedef enum
{
    MENU_S2_VOICE = 0,
    MENU_S2_VOICE_B,
    MENU_S2_GATE,
    MENU_S2_RET,
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
    MENU_S2_RET_RECORD = 0,
    MENU_S2_RET_PLAYBACK,
    MENU_S2_RET_BACK,
    MENU_S2_RET_MAX
} menu_s2_ret_item_t;

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
/* 0=科目一(存slot0), 1=去程(槽1~5), 2=返程(槽6~10) */
static uint8 rec_target = 0;
static uint8 cursor_s2_ret = 0;

/* 科目一路径板上修正。每次按方向键移动中心 2cm，前后各 10 点平滑衰减。 */
#define TRAJ_EDIT_RADIUS       (10U)
#define TRAJ_EDIT_STEP_M       (0.02f)
#define TRAJ_SELECT_STEP       (5U)
static uint16 traj_cursor = 0;
static uint8  traj_edit = 0;
static uint8  traj_dirty = 0;

static uint8 key_mid_last = 1;
static uint8 enc_sw_last = 1;       /* 旋钮按下键上一拍电平(与 MID 等价) */
static uint8 enc_a_last = 1;        /* 旋钮 A/B 相上一拍电平 */
static uint8 enc_b_last = 1;
static int8  enc_accum = 0;         /* 未攒满一格的边沿余数 */
static int8  enc_detent = 0;        /* 解码产出、按键扫描取走的待处理格数 */
static uint8 key_left_last = 1;
static uint8 key_start_last = 1;

/* UP/DOWN 按住时长(10ms 拍)。0=没按住。长按自动重复+粗调靠它。 */
static uint16 key_up_hold = 0;
static uint16 key_down_hold = 0;
/* RIGHT 也走长按重复:它在摄像头页管准星的列,120 列靠单点太慢。 */
static uint16 key_right_hold = 0;
/* 旋钮上一格距今的拍数,用来判快旋(见 MENU_ENC_FAST_GAP)。 */
static uint16 enc_gap = 0xFFFFu;
static uint8  enc_fast = 0;         /* 1=本批格数按粗调倍率处理 */

/* -------------------- 刷屏标志:清屏与重绘分开 --------------------
 * 2026-07-28 前是一个 need_redraw:任何按键都 ips200_clear() + 整页重画。
 * 240x320x2B 整屏擦 SPI 上要十几 ms,又只在 50ms 拍执行 → "按一下、屏闪一下",
 * 旋钮再快也卡在这。
 *
 * 拆开之后:
 *   need_clear   —— 只有换页(current_level / rec_state 变了)才置,由 poll 自动判定;
 *   need_repaint —— 移光标、改值置这个,不清屏,直接把定宽文本盖上去。
 * 本文件所有绘制函数都补足了空格到定宽("%-8.2f"/"> "/"  "/"Save to Flash       "),
 * 所以不清屏也不会留残影 —— 加新界面时必须守住这条,否则会看到花字。
 *
 * repaint_row_only:编辑态改值时只重画光标那一行(24 字符,<1ms),连续调值最跟手。 */
static uint8 need_clear = 1;
static uint8 need_repaint = 1;
static uint8 repaint_row_only = 0;
static menu_level_t     last_drawn_level = MENU_LEVEL_MAIN;
static recording_state_t last_drawn_rec = REC_STATE_IDLE;
/* 科目三上一次画出的阶段。阶段跳变整体【不】触发重绘(见 kart_menu_poll 注释:
 * 会在控制窗口里插整屏 SPI 写),唯一例外是跳到 S3_SIGNAL/S3_FINISHED ——
 * 那两个阶段车已经彻底停机,没有控制窗口要保护,而 S3_SIGNAL 需要提示现场
 * "可以喊灯光/鸣笛口令了",不刷屏的话它会一直停在 "Reversing..." 那页。 */
static kart_subject3_stage_t last_drawn_s3 = S3_PHASE1_FOLLOW;

static void menu_draw_settings(void);
static void menu_draw_settings_row(uint8 row, uint16 y);

/* ==================== 日光可读白蓝配色 ====================
 * 【为什么换配色不花时间】ips200_show_char 画每个字时,笔画像素写 pencolor、
 * 其余像素写 bgcolor —— 一个字格 8x16=128 个像素本来就都要写一遍。所以
 * "换颜色"和"给整行铺高亮底"都是改写入的数值,SPI 字节数一个不多。
 * 这块屏是软件 SPI(IPS200_USE_SOFT_SPI=1,P02.8/P20.3 翻脚),真正贵的是
 * ips200_clear() 整屏 76800 像素 —— 那个已经改成只在换页时做了。
 *
 * 【硬约束:定宽】不清屏重绘,靠的是新文本把旧文本每个字格盖掉。所以所有
 * 行输出都走 ui_bar/ui_item 补空格到 UI_COLS,不要直接 ips200_show_string
 * 写不定长串,否则会看到上一帧的尾巴。 */
#define UI_BG        (0xFFFF)   /* #FFFFFF 日光下高反差白底 */
#define UI_FG        (0x194D)   /* 深海军蓝正文 */
#define UI_SEL_BG    (0xCE7F)   /* 浅蓝选中行底 */
#define UI_SEL_FG    (0x012E)   /* 深蓝选中行字 */
#define UI_BAR_BG    (0x03D9)   /* #007ACC 标题栏蓝 */
#define UI_BAR_FG    (0xFFFF)
#define UI_KEY       (0x045F)   /* 亮蓝关键字段 */
#define UI_NUM       (0x0471)   /* 蓝绿色数值 */
#define UI_OK        (0x2589)   /* 深绿正常态 */
#define UI_WARN      (0xC3E0)   /* 深琥珀提示 */
#define UI_ERR       (0xD9E7)   /* 深红故障 */
#define UI_DIM       (0x7BEF)   /* 中灰次要信息 */

/* 8x16 字体 + 240px 宽 = 一行正好 30 字。行距 20px 留 4px 行间。 */
#define UI_COLS      (30)
#define UI_ROW_H     (20)
#define UI_Y_TITLE   (0)        /* 标题栏(蓝底) */
#define UI_Y_STATUS  (22)       /* 状态行 Yaw/RC/SW */
#define UI_Y_HEAD    (46)       /* 页内副标题/操作提示 */
#define UI_Y_ROW0    (70)       /* 第一个菜单项 */
#define UI_Y_HINT    (296)      /* 底部按键提示 */

/* 换页清屏。必须先把颜色恢复成基色:ips200_clear 用的是当前 bgcolor,
 * 而 ui_* 会把 bgcolor 改成高亮蓝,不复位就会清出一屏蓝。 */
static void ui_clear(void)
{
    draw_clear(UI_FG, UI_BG);
}

/* 整行定宽输出:补空格到 30 字铺满一行,右侧残留一并盖掉。 */
static void ui_bar(uint16 y, const char *s, uint16 fg, uint16 bg)
{
    char line[UI_COLS + 1];
    uint8 i = 0;

    while(i < UI_COLS && s[i] != '\0') { line[i] = s[i]; i++; }
    while(i < UI_COLS)                 { line[i] = ' ';  i++; }
    line[UI_COLS] = '\0';

    draw_string(0, y, line, fg, bg);
}

/* 局部着色文本(状态行那种字段)。调用方自己保证定宽。 */
static void ui_text(uint16 x, uint16 y, const char *s, uint16 fg, uint16 bg)
{
    draw_string(x, y, s, fg, bg);
}

/* 菜单项一行。选中就整行铺选中蓝(不额外花 SPI,见上方说明)。 */
static void ui_item(uint16 y, const char *text, uint8 selected)
{
    char buf[64];
    sprintf(buf, " %c %s", selected ? '>' : ' ', text);
    ui_bar(y, buf, selected ? UI_SEL_FG : UI_FG,
                   selected ? UI_SEL_BG : UI_BG);
}

/* 蓝底标题栏:左边页名,右边角标(脏标记/阶段名之类,可传 NULL)。 */
static void ui_title(const char *page, const char *right)
{
    char line[UI_COLS + 1];
    uint8 i, n = 0;

    for(i = 0; i < UI_COLS; i++) line[i] = ' ';
    line[UI_COLS] = '\0';

    for(i = 0; i + 1 < UI_COLS && page[i] != '\0'; i++) line[i + 1] = page[i];

    if(right != NULL)
    {
        while(right[n] != '\0') n++;
        if(n + 1 < UI_COLS)
        {
            for(i = 0; i < n; i++) line[UI_COLS - 1 - n + i] = right[i];
        }
    }

    draw_string(0, UI_Y_TITLE, line, UI_BAR_FG, UI_BAR_BG);
}

/* 底部按键提示。这里写的键名必须跟真实按键一致:
 * knob=转旋钮, MID=五向中键或按下旋钮(两者等价), KART_LEFT=五向左,
 * START=独立的物理发车键(P20.7,由 kart_mission.c 检边沿,不是菜单 MID)。 */
static void ui_hint(const char *s)
{
    ui_bar(UI_Y_HINT, s, UI_DIM, UI_BG);
}

static void menu_draw_status_bar(void)
{
    char buf[16];
    float yaw = kart_imu_get_yaw();
    uint8 rc_online = kart_remote_is_online();
    kart_remote_sw3_t sw = kart_remote_get_sw3();
    const char* sw_str = (sw == KART_REMOTE_SW3_L) ? "L" :
                         (sw == KART_REMOTE_SW3_M) ? "M" : "H";

    /* 定宽补空格:这一行每拍刷、不清屏,补空格才能盖掉上一帧(-180.0→5.6)。 */
    sprintf(buf, "%-7.1f", yaw);
    ui_text(0,   UI_Y_STATUS, "Yaw", UI_DIM, UI_BG);
    ui_text(32,  UI_Y_STATUS, buf,   UI_NUM, UI_BG);

    ui_text(96,  UI_Y_STATUS, "RC", UI_DIM, UI_BG);
    ui_text(120, UI_Y_STATUS, rc_online ? "ON " : "OFF",
                              rc_online ? UI_OK : UI_ERR, UI_BG);

    ui_text(168, UI_Y_STATUS, "SW", UI_DIM, UI_BG);
    ui_text(192, UI_Y_STATUS, sw_str, UI_KEY, UI_BG);
}

static void menu_draw_main(void)
{
    ui_title("KART Kart_TC387", NULL);
    ui_bar(UI_Y_HEAD, " Select subject", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Subject 1   Slalom",
            (uint8)(cursor_main == MENU_MAIN_SUBJECT1));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Subject 2   Voice",
            (uint8)(cursor_main == MENU_MAIN_SUBJECT2));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "Subject 3   Follow",
            (uint8)(cursor_main == MENU_MAIN_SUBJECT3));
    ui_item(UI_Y_ROW0 + 3 * UI_ROW_H, "Camera      Kart_Debug",
            (uint8)(cursor_main == MENU_MAIN_CAMERA));
    ui_item(UI_Y_ROW0 + 4 * UI_ROW_H, "Settings",
            (uint8)(cursor_main == MENU_MAIN_SETTINGS));

    ui_hint(" knob/UP/DN move  MID enter");
}

/* 行号 → 屏幕 y。这里原先贴着一段"参数一行"的说明,与本宏无关:它讲的是
 * menu_draw_param_row(),那个函数头上还有一份逐字节相同的,重复的一份已删。 */
#define MENU_SET_ROW_Y(row)   ((uint16)(UI_Y_ROW0 + (row) * UI_ROW_H))

/* -------------------- 参数此刻是否真的起作用 --------------------
 * 返回 0 = 这一项现在调了也没用,菜单画成灰字。
 * 起因:菜单里有一批"看着能调、实际不进任何计算"的项,现场照着调会白费一趟。
 * 逐条都是查过代码的,改代码时记得同步这里:
 *
 *  PB Scale/Vmax/Vmin/Alat/ABrake —— 只在 kart_playback_build_profile() 里读,
 *      而该函数在 PB Prof<0.5 时开头就 return → 剖面关着时全灰。
 *      注意 Vmax 也在里面:剖面关着时限速只剩 PB Clamp 一道。
 *  PB RevScl —— 【这一行的灰显判据不准】它除了 build_profile 还有第二个读者
 *      kart_playback_ol_target_v()(科目三开环倒车的目标速),那里不受 PB Prof 管。
 *      所以剖面关着时它画成灰的,可科目三倒车速度仍被它缩放。判据没动(赛后
 *      不改运行时行为),现场记住:调科目三倒车速度别管这行是不是灰的。
 *  S3 OL Ke —— 只在 poll_closedloop() 里读,出厂 S3 OLMode=0 走 openloop → 灰。
 *  PB Clamp —— 它是下发前最后一道闸,PB Vmax 是剖面内的天花板。
 *      Clamp ≥ Vmax 时永远轮不到它裁到东西(录制速度回放仍走它,故只在
 *      剖面开着时才判灰)。
 *  PB LdMax —— 前视距离上限。它比"基础前视+增益×速度"能达到的最大值还大时,
 *      同样永远裁不到东西。 */
static uint8 menu_param_is_active(uint8 id)
{
    switch(id)
    {
        case PARAM_PB_SCALE:
        case PARAM_PB_VMAX:
        case PARAM_PB_VMIN:
        case PARAM_PB_ALAT:
        case PARAM_PB_ABRAKE:
        case PARAM_PB_REVSCL:
            return (uint8)(kart_params_get(PARAM_PB_PROF) > 0.5f);

        case PARAM_S3_OL_KE:
        case PARAM_S3_LEAD:        /* 提前完成量只有方案1 的完成判据读它 */
            return (uint8)(kart_params_get(PARAM_S3_OL_MODE) > 0.5f);

        case PARAM_S3_OL_KH:
            /* 方案1 且 Ke!=0 时 Kh 由 Ke 反解(见 kart_playback.h),这一行调了没用 → 转灰。
             * Ke==0 或 OLMode=0 时 Kh 仍是真旋钮(方案0 只有航向环),保持亮。 */
            if(kart_params_get(PARAM_S3_OL_MODE) <= 0.5f) return 1;
            return (uint8)(!(kart_params_get(PARAM_S3_OL_KE) >  PLAYBACK_OL_KE_EPS ||
                             kart_params_get(PARAM_S3_OL_KE) < -PLAYBACK_OL_KE_EPS));

        case PARAM_SPD_IMAX:
        case PARAM_STR_OUTMAX:
            /* OLMode=1 时这两项由 Flw Cruise / S3 OLSpd 反解(见 kart_params.c),
             * 菜单存的值运行时不再被读 -> 转灰。灰行显示的是算出来的值。 */
            return (uint8)(kart_params_get(PARAM_S3_OL_MODE) <= 0.5f);

        case PARAM_PB_CLAMP:
            /* 剖面关着时录制速度回放照样过这道闸 → 有效。开着时才比大小。 */
            if(kart_params_get(PARAM_PB_PROF) <= 0.5f) return 1;
            return (uint8)(kart_params_get(PARAM_PB_CLAMP)
                           < kart_params_get(PARAM_PB_VMAX));

        case PARAM_PB_LDMAX:
            /* 基础前视 + 增益×钳位速度 = 实际能达到的最大前视。够不到 LdMax 就裁不到。 */
            return (uint8)(kart_params_get(PARAM_PB_LDMAX)
                           < KART_PLAYBACK_LD_BASE
                             + kart_params_get(PARAM_PB_LDGAIN)
                               * kart_params_get(PARAM_PB_CLAMP));

        default:
            return 1;
    }
}

/* 改这个参数会不会改变【别的行】的灰/亮?会 → 编辑时必须整页重画。
 * 起因:编辑态默认只重画光标那一行(省 SPI),但灰显判据是跨参数的 ——
 * 比如把 PB Prof 从 0 拨到 1,同组六行该由灰转亮,只重画光标行的话
 * 屏上会留一片"已经生效了却还是灰的"行,比不做灰显更误导。
 * 对应关系照抄 menu_param_is_active,改那边记得同步这边。 */
static uint8 menu_param_gates_others(uint8 id)
{
    switch(id)
    {
        case PARAM_PB_PROF:        /* 管 Scale/Vmax/Vmin/Alat/ABrake/RevScl/Clamp */
        case PARAM_S3_OL_MODE:     /* 管 S3 OL Ke / S3 Lead / S3 OL Kh / Spd Imax / Str OutMax */
        case PARAM_S3_OL_KE:       /* 管 S3 OL Kh:Ke 一旦非 0,Kh 那行要转灰 */
        case PARAM_FLW_CRUZ:       /* 管 Spd Imax + Str OutMax 的显示值 */
        case PARAM_S3_OL_SPD:      /* 管 Spd Imax 的显示值 */
        case PARAM_PB_VMAX:        /* 参与 PB Clamp 的判据 */
        case PARAM_PB_CLAMP:       /* 参与 PB LdMax 的判据 */
        case PARAM_PB_LDGAIN:      /* 参与 PB LdMax 的判据 */
            return 1;
        default:
            return 0;
    }
}

/* 参数一行:"光标 名字 值"。值按 meta.decimals 决定小数位,整数量(Vmax/Kp)不显示 .0。
 * 选中且在编辑态时光标变 '*',提示此刻 knob/UP/DN 改的是值不是光标。
 *
 * 名字用 %-11s 补到定宽:滚屏会让同一行换成另一个参数,名字短一截就会留下
 * 上一个名字的尾巴("Ramp Step"→"PB Prof" 剩个 'e')。 */
static void menu_draw_param_row(uint8 id, uint16 y, uint8 selected)
{
    const kart_param_meta_t *m = kart_params_meta(id);
    float v = kart_params_derived(id);   /* 反解量显示算出来的值,不是存的值 */
    uint8 active = menu_param_is_active(id);
    char buf[64];

    switch(m->decimals)
    {
        case 0:  sprintf(buf, " %c %-11s %-8.0f", selected ? (set_edit ? '*' : '>') : ' ', m->name, v); break;
        case 1:  sprintf(buf, " %c %-11s %-8.1f", selected ? (set_edit ? '*' : '>') : ' ', m->name, v); break;
        case 2:  sprintf(buf, " %c %-11s %-8.2f", selected ? (set_edit ? '*' : '>') : ' ', m->name, v); break;
        default: sprintf(buf, " %c %-11s %-8.3f", selected ? (set_edit ? '*' : '>') : ' ', m->name, v); break;
    }

    /* 编辑态用黄字提示"这一行正在被改",非编辑态选中就是普通选中蓝。
     * 无效项(灰)选中时保留高亮底 —— 否则光标停在灰行上会像"光标消失了"。 */
    if(selected)
        ui_bar(y, buf, set_edit ? UI_WARN : (active ? UI_SEL_FG : UI_DIM), UI_SEL_BG);
    else
        ui_bar(y, buf, active ? UI_FG : UI_DIM, UI_BG);
}

/* Settings 页:滚屏列表。MID 切"移光标/改值",KART_LEFT 退出(编辑态先退编辑)。
 * 改完立即生效(存 RAM),停手 2s 或按 KART_LEFT 自动落盘,Save to Flash 是手动兜底。 */
static void menu_draw_settings(void)
{
    uint8 row;

    /* 标题栏右角标 * = 有改动还没进 Flash,存完就消失。 */
    ui_title("Settings", kart_params_is_dirty() ? "*" : " ");
    ui_bar(UI_Y_HEAD, set_edit ? " EDIT  value" : " BROWSE  parameter",
           set_edit ? UI_WARN : UI_KEY, UI_BG);

    for(row = 0; row < MENU_SET_ROWS; row++)
    {
        menu_draw_settings_row((uint8)(set_top + row), MENU_SET_ROW_Y(row));
    }

    /* 提示只写真实按键:knob=转旋钮,MID=五向中键或按下旋钮(两者等价)。 */
    ui_hint(set_edit ? " knob/UP/DN value  MID ok"
                     : " MID edit  KART_LEFT back");
}

/* 画 Settings 的一行。抽出来是为了编辑态改值时只重画光标那行(不清屏、不动其余行),
 * 连续调值/快旋时屏幕开销从"整页 11 行"降到"1 行"。
 * 入参是【行号】(set_rows 下标),不是参数 id —— 分组之后两者不再相等。 */
static void menu_draw_settings_row(uint8 row, uint16 y)
{
    uint8 id;

    if(row >= MENU_SET_TOTAL)
    {
        ui_bar(y, "", UI_FG, UI_BG);        /* 表尾之后的空行也要铺,盖掉滚屏残影 */
        return;
    }

    id = set_rows[row].id;

    if(id == SET_ROW_HEAD)
    {
        /* 分组标题:不可选中,用关键字蓝区分,一眼看出分段。 */
        ui_bar(y, set_rows[row].text, UI_KEY, UI_BG);
    }
    else if(id == SET_ROW_SAVE || id == SET_ROW_DEF)
    {
        ui_item(y, set_rows[row].text, (uint8)(cursor_set == row));
    }
    else
    {
        menu_draw_param_row(id, y, (uint8)(cursor_set == row));
    }
}

/* 只重画 Settings 页当前光标那一行(编辑态改值用)。光标不在可视窗口就整页画。 */
static void menu_draw_settings_cursor_row(void)
{
    if(cursor_set < set_top || cursor_set >= set_top + MENU_SET_ROWS)
    {
        menu_draw_settings();
        return;
    }
    menu_draw_settings_row(cursor_set, MENU_SET_ROW_Y(cursor_set - set_top));
}

/* 光标移动后把它拉进可视窗口。
 * 多留一手:光标落在某组第一个参数上时把它的分组标题一起带进屏内,
 * 否则会出现"标题刚滚出屏、只剩一堆参数名",看不出现在调的是哪一组。 */
static void menu_settings_scroll(void)
{
    if(cursor_set < set_top) set_top = cursor_set;
    if(cursor_set >= set_top + MENU_SET_ROWS)
        set_top = (uint8)(cursor_set - MENU_SET_ROWS + 1);

    if(set_top > 0 && set_top == cursor_set && set_rows[set_top - 1].id == SET_ROW_HEAD)
        set_top--;
}

/* 光标按 dir(+1 下 / -1 上)走一步,跳过不可选中的分组标题行。
 * 撞到表头/表尾原地不动(不回绕:回绕会让"按住 DOWN"在表两头来回窜)。
 * 用 int 中间量算:cursor_set 是 uint8,0 再减 1 会绕成 255。 */
static void menu_settings_seek(int8 dir)
{
    int next = (int)cursor_set;

    do
    {
        next += dir;
        if(next < 0 || next >= (int)MENU_SET_TOTAL) return;      /* 到头了 */
    } while(set_rows[next].id == SET_ROW_HEAD);

    cursor_set = (uint8)next;
    menu_settings_scroll();
}

/* 光标置到第一个可选中项。进 Settings 页时用:第 0 行是分组标题,不能停上面。 */
static void menu_settings_cursor_first(void)
{
    uint8 i;

    cursor_set = 0;
    set_top    = 0;

    for(i = 0; i < MENU_SET_TOTAL; i++)
    {
        if(set_rows[i].id != SET_ROW_HEAD) { cursor_set = i; break; }
    }
}

/* 光标那一行对应的参数 id;标题/动作行返回 PARAM_MAX(表示"不是参数")。
 * 编辑态改值前用它翻译:分组之后 cursor_set 是行号,不再等于参数 id。 */
static uint8 menu_settings_param_id(void)
{
    uint8 id;

    if(cursor_set >= MENU_SET_TOTAL) return PARAM_MAX;

    id = set_rows[cursor_set].id;
    return (id < PARAM_MAX) ? id : PARAM_MAX;
}

/* ======================== 摄像头调试页 ========================
 * 目的:上车调黄色引导板时,把"相机有没有出图"和"出的图认不认得出板子"
 * 这两件事在屏上一次看全,不用连 VOFA、不用把车推到电脑边。
 *
 * 【为什么不直接用 kart_camera_preview()】
 * 那个函数自己 frame_ready → 显示 → frame_release,帧在它手里进出。
 * 本页必须对【同一帧】既显示又跑 kart_vision,否则框会画在另一帧的位置上 ——
 * 人举着板子走的时候两帧能差出十几个像素,看着就像识别永远偏一点。
 * 所以这里自己取帧:frame_ready → kart_vision_process → 画图 → 画框 → frame_release。
 *
 * 【时序安全】本函数只在 50ms 拍(kart_menu_poll)里跑,那一拍本来就允许
 * 整屏 ui_clear()(软件 SPI,十几 ms),再加 160x120 出图约 4ms + kart_vision 约 3ms
 * 仍在同一量级,不进 5ms 控制窗口。且本页强制 MISSION_IDLE(见 MID 处),
 * 车不可能在动,慢一点也不会有后果。
 *
 * 【与真实检测器的一致性】判别式不在这里重写,一律走 kart_vision_probe_pixel()
 * 和 kart_vision_process()。调试页显示的通过/不通过就是跑车时的判定。 */

/* ---- 屏幕分区(240x320,8x16 字体) ----
 *   0..15    标题栏
 *   22..37   状态行(menu_draw_status_bar 每拍刷,所有页公用)
 *   46..61   相机链路状态 + 帧率
 *   66..185  图像 160x120,靠左
 *   x>=168   图像右侧空出 80px = 9 字,竖排放检测结果各字段
 *   190..285 底部 5 行满宽文字(取样点判别中间量 + 诊断计数)
 *   296      按键提示
 * 排布是按"不重叠"倒推的:图像 120 行高且必须整幅显示(缩放会让像素级
 * 取样失去意义),240 宽的屏放 160 宽的图必然剩一条 80px 的窄栏,那条栏
 * 只能放短字段,所以宽字段(判别式两边的整数)才放到图下面。 */
#define CAMDBG_IMG_X        (0)
#define CAMDBG_IMG_Y        (66)
#define CAMDBG_COL_X        (168)      /* 图右侧窄栏起始 x,= 第 21 字 */
#define CAMDBG_COL_W        (9)        /* 窄栏宽度(字),30-21=9 */
#define CAMDBG_TXT_Y0       (190)      /* 图像下沿(66+120=186)之后 */

/* 取样准星:默认画面正中。UP/DOWN 调行,RIGHT 调列(单向回卷);
 * 旋钮也能调列,但它实测不好用,别指望它(见按键处理)。
 * 这三个不放进 #if:按键处理函数里的 MENU_LEVEL_CAMERA 分支是无条件编译的
 * (那边只动数,不碰相机 API),关掉相机时那一页只显示"开关没打开",
 * 数怎么动都无所谓,但变量得在。 */
static int16 camdbg_probe_x = SCC8660_W / 2;
static int16 camdbg_probe_y = SCC8660_H / 2;
static uint8 camdbg_show_overlay = 1;   /* MID 切换:叠加框/准星 开关 */

/* 色相标定 ROI 的边长。15 是折中:板子在 2m 处宽约 14px(f_px 83.5、板宽 0.33m),
 * 取 15 保证近距离时整个方块都落在板面内,不会扫到黑边或背景。
 * 距离更远要减小它,否则中位数会被背景污染 —— 看 ROI% 那个数就知道有没有被污染。 */
static int16 camdbg_roi_side = 15;

/* 以下到 menu_draw_camera 结束都只在开了相机时才编译:
 * 关掉时这些函数/变量一个都用不上,留着就是一堆 defined but not used 警告。 */
#if (CAMERA_ENABLE)

/* 上一次成功出图的帧计数,用来判断"这一拍到底有没有新图"。 */
static uint32 camdbg_last_frame_cnt = 0;

/* 准星处的原始像素。做成静态是因为它必须在【还帧之前】取走(见取帧处注释),
 * 而显示它的那几行画在"有帧/无帧"分支之外 —— 没新帧时就显示上一帧取到的值,
 * 与旁边同样保留上一帧结论的 bear/cx 一致,不会出现半新半旧。 */
static uint16 camdbg_probe_pix = 0;

/* 画外接框(空心矩形)。坐标是图像像素,需加图像区偏移。
 * 越界钳制:kart_vision 给的框一定在图内,但准星是人调的,统一钳一次更省心。 */
static void camdbg_draw_box(int16 x0, int16 y0, int16 w, int16 h, uint16 color)
{
    int16 x1;
    int16 y1;

    if(w <= 0 || h <= 0) { return; }

    if(x0 < 0) { x0 = 0; }
    if(y0 < 0) { y0 = 0; }
    x1 = (int16)(x0 + w - 1);
    y1 = (int16)(y0 + h - 1);
    if(x1 > (int16)(SCC8660_W - 1)) { x1 = (int16)(SCC8660_W - 1); }
    if(y1 > (int16)(SCC8660_H - 1)) { y1 = (int16)(SCC8660_H - 1); }

    draw_line((uint16)(CAMDBG_IMG_X + x0), (uint16)(CAMDBG_IMG_Y + y0),
                   (uint16)(CAMDBG_IMG_X + x1), (uint16)(CAMDBG_IMG_Y + y0), color);
    draw_line((uint16)(CAMDBG_IMG_X + x0), (uint16)(CAMDBG_IMG_Y + y1),
                   (uint16)(CAMDBG_IMG_X + x1), (uint16)(CAMDBG_IMG_Y + y1), color);
    draw_line((uint16)(CAMDBG_IMG_X + x0), (uint16)(CAMDBG_IMG_Y + y0),
                   (uint16)(CAMDBG_IMG_X + x0), (uint16)(CAMDBG_IMG_Y + y1), color);
    draw_line((uint16)(CAMDBG_IMG_X + x1), (uint16)(CAMDBG_IMG_Y + y0),
                   (uint16)(CAMDBG_IMG_X + x1), (uint16)(CAMDBG_IMG_Y + y1), color);
}

/* 画取样准星(十字),中心留空一格,免得把被取样的那个像素自己盖掉。 */
static void camdbg_draw_cross(int16 cx, int16 cy, uint16 color)
{
    const int16 arm = 5;
    int16 a = (int16)(cx - arm);
    int16 b = (int16)(cx + arm);
    int16 c = (int16)(cy - arm);
    int16 d = (int16)(cy + arm);

    if(a < 0) { a = 0; }
    if(c < 0) { c = 0; }
    if(b > (int16)(SCC8660_W - 1)) { b = (int16)(SCC8660_W - 1); }
    if(d > (int16)(SCC8660_H - 1)) { d = (int16)(SCC8660_H - 1); }

    draw_line((uint16)(CAMDBG_IMG_X + a),        (uint16)(CAMDBG_IMG_Y + cy),
                   (uint16)(CAMDBG_IMG_X + cx - 1),   (uint16)(CAMDBG_IMG_Y + cy), color);
    draw_line((uint16)(CAMDBG_IMG_X + cx + 1),   (uint16)(CAMDBG_IMG_Y + cy),
                   (uint16)(CAMDBG_IMG_X + b),        (uint16)(CAMDBG_IMG_Y + cy), color);
    draw_line((uint16)(CAMDBG_IMG_X + cx),       (uint16)(CAMDBG_IMG_Y + c),
                   (uint16)(CAMDBG_IMG_X + cx),       (uint16)(CAMDBG_IMG_Y + cy - 1), color);
    draw_line((uint16)(CAMDBG_IMG_X + cx),       (uint16)(CAMDBG_IMG_Y + cy + 1),
                   (uint16)(CAMDBG_IMG_X + cx),       (uint16)(CAMDBG_IMG_Y + d), color);
}

/* 图右侧窄栏一行:定宽补到 9 字。
 * 必须补空格,理由和 ui_bar 一样 —— 这一页不清屏(清了图会闪),
 * 新字不铺满就会看到上一帧的数字尾巴,比如 "d 1.50m" 盖 "d 12.40m" 会读成 "d 1.50m0"。 */
static void camdbg_col(uint16 y, const char *s, uint16 fg)
{
    char line[CAMDBG_COL_W + 1];
    uint8 i = 0;

    while(i < CAMDBG_COL_W && s[i] != '\0') { line[i] = s[i]; i++; }
    while(i < CAMDBG_COL_W)                 { line[i] = ' ';  i++; }
    line[CAMDBG_COL_W] = '\0';

    draw_string(CAMDBG_COL_X, y, line, fg, UI_BG);
}

/* 相机链路状态 → 屏上文字 + 颜色。四个状态各自对应完全不同的排查方向,
 * 所以不合并成"OK/FAIL":INIT_FAIL 查配置串口(UART1 借还/波特率),
 * NO_SIGNAL 查 DVP 排线和 PCLK,OFF 是编译开关没打开。 */
static const char *camdbg_state_str(kart_cam_state_enum st, uint16 *color)
{
    switch(st)
    {
        case KART_CAM_STATE_RUNNING:   *color = UI_OK;   return "RUNNING";
        case KART_CAM_STATE_NO_SIGNAL: *color = UI_ERR;  return "NO SIGNAL";
        case KART_CAM_STATE_INIT_FAIL: *color = UI_ERR;  return "INIT FAIL";
        case KART_CAM_STATE_OFF:
        default:                       *color = UI_DIM;  return "OFF";
    }
}

static const char *camdbg_reject_str(uint8 rej)
{
    switch(rej)
    {
        case VISION_REJ_OK:     return "OK    ";
        case VISION_REJ_AREA:   return "AREA  ";     /* 太远/不在视野/曝光过暗 */
        case VISION_REJ_WIDTH:  return "WIDTH ";     /* 框太窄,测距没意义 */
        case VISION_REJ_ASPECT: return "ASPECT";     /* 细长条,多半是反光/衣服边 */
        case VISION_REJ_FILL:   return "FILL  ";     /* 填充率低,常见于两块黄色被并框 */
        default:                     return "?     ";
    }
}

static void menu_draw_camera(void)
{
    char  buf[40];
    uint16 st_color;
    kart_cam_state_enum st = kart_camera_state();
    const char *st_str = camdbg_state_str(st, &st_color);

    ui_title("Camera Kart_Debug", (VISION_BYTE_SWAP) ? "SWAP" : NULL);

    /* 出图 + 同帧跑视觉。frame_ready 为 0 就保持上一屏,不清图区 ——
     * 清了会闪,而且没有新信息可显示。 */
    if(kart_camera_frame_ready())
    {
        const kart_vision_result_t *v;

        /* 【顺序】先跑视觉再出图:kart_vision 扫的是 scc8660_image,
         * 出图函数也读同一块。两者都在还帧之前完成,DMA 不会中途改写。 */
        v = kart_vision_process((const uint16 *)scc8660_image[0],
                                (int16)SCC8660_W, (int16)SCC8660_H);

        /* 准星像素在【还帧之前】取走。放到还帧之后读就可能读到 DMA 正在写的
         * 下一帧,取样值与屏上这一帧对不上 —— 而这个数是用来判"这个颜色到底
         * 过不过判别式"的,对不上就等于在看另一张图的结论。 */
        camdbg_probe_pix = scc8660_image[camdbg_probe_y][camdbg_probe_x];

        draw_image(CAMDBG_IMG_X, CAMDBG_IMG_Y);

        if(camdbg_show_overlay)
        {
            /* 认到了画绿框,没认到画黄框(还是要画:黄框告诉你"黄色找到了但被门限拒了",
             * 配合下面的 REJ 字段就知道差在哪一项。完全没有黄色时 width=0,不画)。 */
            camdbg_draw_box(v->box_x0, v->box_y0, v->width_px, v->height_px,
                            v->valid ? UI_OK : UI_WARN);
            camdbg_draw_cross(camdbg_probe_x, camdbg_probe_y, UI_ERR);
        }

        camdbg_last_frame_cnt = g_cam_frame_count;
        kart_camera_frame_release();     /* 处理完立刻还帧 */

        /* ---- 图右侧窄栏:检测结果 ---- */
        sprintf(buf, "V%d", v->valid ? 1 : 0);
        camdbg_col(CAMDBG_IMG_Y + 0 * UI_ROW_H, buf, v->valid ? UI_OK : UI_ERR);

        camdbg_col(CAMDBG_IMG_Y + 1 * UI_ROW_H, camdbg_reject_str(v->reject),
                   (v->reject == VISION_REJ_OK) ? UI_OK : UI_WARN);

        sprintf(buf, "A%5u", (unsigned int)v->area_px);
        camdbg_col(CAMDBG_IMG_Y + 2 * UI_ROW_H, buf, UI_NUM);

        /* W 是标定 f_px 的唯一依据:卷尺量准 d,读 W,
         * f_px = W * d / 0.33,回填 VISION_FPX。0.33 是 VISION_BOARD_W_M
         * (kart_vision.h),原文写的 0.31 是换板之前的板宽,数已订正。 */
        sprintf(buf, "W%3d", v->width_px);
        camdbg_col(CAMDBG_IMG_Y + 3 * UI_ROW_H, buf, UI_NUM);

        sprintf(buf, "H%3d", v->height_px);
        camdbg_col(CAMDBG_IMG_Y + 4 * UI_ROW_H, buf, UI_NUM);

        sprintf(buf, "d%5.2f", (double)v->dist_m);
        camdbg_col(CAMDBG_IMG_Y + 5 * UI_ROW_H, buf, UI_NUM);
    }
    else
    {
        /* 没有新帧。区分"相机在跑只是本拍没赶上"和"根本不出图":
         * 前者 frame_count 在涨,后者不动。这一句省掉现场"是不是卡死了"的猜。
         * 写在窄栏而不是盖住图区:上一帧的图留着,断线瞬间的画面还能看。 */
        sprintf(buf, "no frame");
        camdbg_col(CAMDBG_IMG_Y + 0 * UI_ROW_H, buf,
                   (g_cam_frame_count != camdbg_last_frame_cnt) ? UI_WARN : UI_ERR);
        sprintf(buf, "c%lu", (unsigned long)(g_cam_frame_count % 100000u));
        camdbg_col(CAMDBG_IMG_Y + 1 * UI_ROW_H, buf, UI_DIM);
    }

    /* ---- 图下方满宽文字:方位角 + 准星处的判别中间量 ----
     * 这几行每拍都画,不放进 frame_ready 分支里:没新帧时它们显示的是上一帧的
     * 结论(kart_vision_get 返回的是上次保存的结果),照样是有效信息;
     * 而且行不重画就会被"有帧/无帧"来回切时的残留搞乱。 */
    {
        const kart_vision_result_t *vr = kart_vision_get();
        int16 r5 = 0, g5 = 0, b5 = 0, sum = 0;
        int32 lhs = 0, rhs = 0;
        uint8 hit;

        /* 方位角:>0 = 目标在右。上车验打角符号就看它和 CH15(target_delta) 是否反号。 */
        sprintf(buf, " bear%+6.1fdeg  cx%3d", (double)(vr->bearing_rad * 57.29578f),
                vr->cx_px);
        ui_bar(CAMDBG_TXT_Y0, buf, UI_NUM, UI_BG);

        /* 用还帧前抓下的那一个像素,保证与屏上这一帧、与上面 kart_vision 的结论同源。 */
        hit = kart_vision_probe_pixel(camdbg_probe_pix,
                                      &r5, &g5, &b5, &lhs, &rhs, &sum);

        sprintf(buf, " P(%3d,%3d) %s", camdbg_probe_x, camdbg_probe_y,
                hit ? "YELLOW " : "no     ");
        ui_bar(CAMDBG_TXT_Y0 + UI_ROW_H, buf, hit ? UI_OK : UI_DIM, UI_BG);

        sprintf(buf, " R%2d G%2d B%2d S%3d", r5, g5, b5, sum);
        ui_bar(CAMDBG_TXT_Y0 + 2 * UI_ROW_H, buf, UI_FG, UI_BG);

        /* 判别式两边都给:lhs>rhs 才算黄色。差得远就是颜色不对,
         * 差一点点就是曝光/白平衡问题,值得先固定曝光再动阈值。
         * sum < MIN_SUM 时直接被判暗部弃掉,单独标出来防误判成"颜色不对"。 */
        sprintf(buf, " M%6ld >%6ld %s", (long)lhs, (long)rhs,
                (sum < VISION_MIN_SUM) ? "DARK" : "    ");
        ui_bar(CAMDBG_TXT_Y0 + 3 * UI_ROW_H, buf,
               (sum < VISION_MIN_SUM) ? UI_ERR : UI_FG, UI_BG);

        /* ---- 色相标定读数（换板子/换光照后填宏就看这一行）----
         * HUE=中位数 -> 直接填 VISION_HUE_CENTER;
         * spread=p90-p10 -> HUE_TOL 至少要 spread/2 + 4;
         * S=饱和度中位数,离 MIN_SAT_PCT(25) 越远越安全;
         * ok=区域内有效像素占比,低于 80 说明准星没对准板面或者 ROI 太大扫到背景。
         * 每拍都算:15x15=225 像素,只在这一页跑,不进跑车链路。 */
        {
            vision_roi_stat_t rs;
            vision_roi_stat((const uint16 *)scc8660_image[0],
                                 (int16)SCC8660_W, (int16)SCC8660_H,
                                 camdbg_probe_x, camdbg_probe_y,
                                 camdbg_roi_side, &rs);

            if(rs.med_hue >= 0)
            {
                sprintf(buf, " HUE%3d sp%2d S%3d ok%3d",
                        rs.med_hue, (int)(rs.p90_hue - rs.p10_hue),
                        rs.med_sat_pct, rs.ok_pct);
            }
            else
            {
                /* 有效样本 <5：把三个占比摊出来,现场立刻知道是欠曝还是不够鲜艳。 */
                sprintf(buf, " HUE --- dk%3d ls%3d n%3d",
                        rs.dark_pct, rs.lowsat_pct, rs.n_total);
            }
            ui_bar(CAMDBG_TXT_Y0 + 4 * UI_ROW_H, buf,
                   (rs.med_hue >= 0) ? UI_OK : UI_WARN, UI_BG);
        }
    }

    /* 底部诊断行。misalign 非 0 = DMA 链表段数与实际不符;
     * init_ret 非 0 = scc8660_init 没过,图像肯定是黑的,先查 UART1 配置链路;
     * drp 本页必然一直涨(50ms 取一帧,相机 30fps),只有跑车时猛涨才是问题。 */
    /* 【行位挪过】原来在 TXT_Y0+4,那一行现在给色相标定读数用了。
     * 不能再往下放:TXT_Y0+5 = 290,会压到 UI_Y_HINT(296) 的提示行。
     * 这四个数只在"图像根本不对"时才需要看,并到副标题行足够。 */
    sprintf(buf, " %-9s fps%3u  m%lu d%lu i%d",
            st_str, (unsigned int)g_cam_fps,
            (unsigned long)(g_cam_misalign_count % 1000u),
            (unsigned long)(g_cam_drop_count % 1000u),
            g_cam_init_ret);
    ui_bar(UI_Y_HEAD, buf, (g_cam_init_ret != 0) ? UI_ERR : st_color, UI_BG);

    ui_hint(" knob X  UP/DN Y  MID box  L2 ROI");
}

#else   /* !CAMERA_ENABLE */

/* 编译开关没开:明确告诉现场要改哪个宏,别让人以为是硬件坏了。
 * 这一页照样进得去(菜单项不隐藏)—— 隐藏了就会有人以为版本不带这功能。 */
static void menu_draw_camera(void)
{
    ui_title("Camera Kart_Debug", "OFF");
    ui_bar(UI_Y_HEAD,            " CAMERA_ENABLE = 0", UI_ERR, UI_BG);
    ui_bar(UI_Y_ROW0,            " set it to 1 in",         UI_FG,  UI_BG);
    ui_bar(UI_Y_ROW0 + UI_ROW_H, " kart_camera.h, rebuild", UI_FG,  UI_BG);
    ui_hint(" KART_LEFT exit");
}

#endif  /* CAMERA_ENABLE */

/* ================== 科目三跟随实时画面(出厂用的就是这一页) ==================
 * 原标"临时,测完删" —— 但 PERSON_LINK_ENABLE 是 0,S3_FOLLOW_SRC 就取 VISION,
 * 下面这个 #if 出厂编译进去了,整场比赛看的都是它,一直没删。
 * 只干一件事:把跟随当前用的那一帧和它认出的框显示出来,看识别对不对。
 *
 * 【不走 frame_ready / frame_release】跟随那一拍已经在消费帧了,菜单再取一次
 * 就是跟控制环抢帧 —— 菜单取走并 release 的帧跟随就看不到,本来在查识别率低,
 * 加个观察窗口反而让它更低。所以直接读 scc8660_image,不碰帧协议。
 * 代价是偶尔一条撕裂缝(读到 DMA 正在写的半帧),不影响看框。
 * 框和数字取 kart_vision_get(),就是跟随真正用的结论,不另算一遍。
 *
 * 真要删:本函数 + menu_draw_s3_run 里的分支 + menu_poll_body 里那个每拍置
 * need_repaint 的分支,一共 3 处(同一个 #if 条件)。删了现场就没有观察窗口。 */
#if (S3_FOLLOW_SRC == S3_FOLLOW_SRC_VISION) && (CAMERA_ENABLE)
static void menu_draw_s3_live(void)
{
    const kart_vision_result_t *v = kart_vision_get();
    char buf[40];

    draw_image(CAMDBG_IMG_X, CAMDBG_IMG_Y);

    /* 认到绿框,没认到黄框(黄 = 找到黄色但被门限拒了,看下面 REJ 是哪一项)。 */
    camdbg_draw_box(v->box_x0, v->box_y0, v->width_px, v->height_px,
                    v->valid ? UI_OK : UI_WARN);

    /* 图右侧窄栏:认没认到 + 被哪个门限拒的 + 宽度/距离。 */
    camdbg_col(CAMDBG_IMG_Y + 0 * UI_ROW_H, v->valid ? "OK" : "NO",
               v->valid ? UI_OK : UI_ERR);
    camdbg_col(CAMDBG_IMG_Y + 1 * UI_ROW_H, camdbg_reject_str(v->reject),
               (v->reject == VISION_REJ_OK) ? UI_OK : UI_WARN);
    sprintf(buf, "W%3d", v->width_px);
    camdbg_col(CAMDBG_IMG_Y + 2 * UI_ROW_H, buf, UI_NUM);
    sprintf(buf, "d%5.2f", (double)v->dist_m);
    camdbg_col(CAMDBG_IMG_Y + 3 * UI_ROW_H, buf, UI_NUM);
    sprintf(buf, "A%5u", (unsigned int)v->area_px);
    camdbg_col(CAMDBG_IMG_Y + 4 * UI_ROW_H, buf, UI_NUM);

    /* 方位角和跟随下发的打角必须【反号】。同号就是 kart_follow_update() 里
     * delta = -(KART_STEER_R_TIMES_DELTA * kappa) 那个负号错了(一打就反向),
     * 立刻拨 SW3 到低挡停车。原注释指的 kart_follow.c:127 已不是那一行。 */
    sprintf(buf, " bear%+6.1f del%+6.0f",
            (double)(v->bearing_rad * 57.29578f),
            (double)kart_follow_get()->target_delta);
    ui_bar(CAMDBG_TXT_Y0, buf, UI_NUM, UI_BG);

    /* 提示文字必须 <=30 字(UI_COLS),超了 ui_bar 会直接截断成半个词。 */
    ui_hint(" hide board=back  KART_LEFT exit");
}
#endif

#if (S3_FOLLOW_SRC == S3_FOLLOW_SRC_PLINK)
/* ================== 科目三 PLINK 链路状态页 ==================
 * 与上面 menu_draw_s3_live() 的关系：两者互斥，同一个位置的两个分支。
 * 【为何不出图】PLINK 下图像在 TC4D7 侧，387 根本拿不到像素；scc8660_image 里
 * 要么是陈帧要么是黑的（CAMERA_ENABLE 可能还是 1，但摄头与本链路无关）。
 * 画一幅无关的图比不画更坏，所以这一页只出数字。
 *
 * 【开销】四行定宽文本，约 1~2ms/拍（跟其他菜单页同量级），不清屏。
 * 不像 menu_draw_s3_live() 那样要 4ms 出图，所以这一页不是“测完删”的临时物。
 * 但出厂 PERSON_LINK_ENABLE=0,本页整块没编译进去 —— 真正在跑的是上面那页。
 *
 * link 一列与 kart_debug_uart.c 里 PERSON_LINK 那档日志的 ch[39] 同义,但出厂档
 * LOG_PROFILE_S3=1 的通道表里 CH39 是 kart_playback_get_cur_y();ch[39]=link 在
 * 另一档的 #else 里,还要 PERSON_LINK_ENABLE=1 才编译,别对着出厂日志找它：
 *   0=一个字节没收到（线/波特率/4D7 没在发） 1=有字节但从未成帧（帧头或 CRC）
 *   2=曾通现失联（>200ms 无 VALID 帧）          3=在线
 * 现场先看这一位；0/1 是链路问题，2/3 才轮得到看跟随。 */
static void menu_draw_s3_plink(void)
{
    const kart_vtrack_result_t *vt = kart_person_link_vtrack();
    const kart_follow_out_t    *fo = kart_follow_get();
    kart_person_link_stat_t     st;
    kart_person_frame_t         pf;
    uint8  got;
    uint8  link;
    char   buf[40];

    kart_person_link_get_stat(&st);
    got = kart_person_link_get_frame(&pf);

    if(0U == st.byte_count)                     { link = 0U; }
    else if(0U == st.frame_ok)                  { link = 1U; }
    else if(0U == kart_person_link_is_online()) { link = 2U; }
    else                                        { link = 3U; }

    /* 不再调 ui_title：外层 menu_draw_s3_run 已经画过 ("Subject 3","REC")，
     * menu_draw_s3_live 也没再画一次。重复写标题栏白多一次 SPI。
     * REC 这个字在 PLINK 下也是对的：阶段1 确实在录轨。
     *
     * 第一行就是“能不能跑”。颜色分三档，不是两档：
     *   3 在线 → 绿；2 曾通现失联 → 黄（链路本身通了，人不在画面里也会是 2，
     *     但 4D7 重启/线松也是 2，所以不能当正常）；0/1 链路没通 → 红。
     * 先前写成 link>=2 就绿是错的：失联不能看着像正常。 */
    sprintf(buf, " link%1u  ok%5u  bad%5u", (unsigned int)link,
            (unsigned int)st.frame_ok,
            (unsigned int)(st.crc_err + st.hdr_err + st.seq_stale + st.resync));
    ui_bar(UI_Y_HEAD, buf,
           (3U == link) ? UI_OK : ((2U == link) ? UI_WARN : UI_ERR), UI_BG);

    /* 方位角与跟随下发的打角必须【反号】——同号就是旋转方向接反了
     * （人往右车往左），立即拨 SW3 到低档停车。判据与视觉页一致。 */
    sprintf(buf, " bear%+6.1f del%+6.0f",
            (double)(vt->bearing_rad * 57.29578f),
            (double)fo->target_delta);
    ui_bar(UI_Y_ROW0, buf, UI_NUM, UI_BG);

    /* h = 归一化高度×1000（标 NEAR_HEIGHT_STOP 用它）；sc = 合成的 scale_r；
     * st = kart_follow 状态 0IDLE 1TRACK 2HOLD 3LOST；ns = 近距联锁已触发。 */
    sprintf(buf, " h%4u sc%5.2f s%1u n%1u",
            (unsigned int)(got ? pf.height : 0U),
            (double)fo->scale_r,
            (unsigned int)fo->state,
            (unsigned int)fo->near_stop);
    ui_bar(UI_Y_ROW0 + UI_ROW_H, buf,
           vt->valid ? UI_FG : UI_WARN, UI_BG);

    /* 提示文字必须 <=30 字(UI_COLS)，超了 ui_bar 会直接截断成半个词。 */
    ui_hint(" hide 2s=back  KART_LEFT exit");
}
#endif

/* 科目三运行界面:菜单不吃 MID(防运行中刷屏),只留 KART_LEFT 退出。
 * 到停车区【把车停住即自动】停录并开环倒车原路返回,车头不掉转、不搬车。
 * 按 kart_mission 当前阶段显示,阶段跳变时由 kart_menu_poll 触发重绘。 */
static void menu_draw_s3_run(void)
{
    /* 这一页的 START 是独立的物理发车键(BOARD_START_KEY_PIN / P20.7),
     * 由 kart_mission.c 检下降沿 —— 不是菜单 MID,文字别改。 */
    switch(kart_mission_get_subject3_stage())
    {
        case S3_PHASE1_FOLLOW:
            /* 文案以自动判停为主:kart_mission.c subject3_loop 里,走够 1m 之后
             * 车速连续 150ms ≈0 就自己停录+开倒车,START 只是"不想等那 150ms"的
             * 手动提前触发。原来写成 "START to go back",现场会以为必须按键。
             * 两种控制源文案不同(S3_FOLLOW_SRC),别让现场看着遥控提示去举板子。 */
            ui_title("Subject 3", "REC");
/* 这里的条件必须跟 menu_draw_s3_live 的定义条件【逐字一致】,少一个
 * CAMERA_ENABLE 就会在关摄像头的版本里调到一个没定义的函数。 */
#if (S3_FOLLOW_SRC == S3_FOLLOW_SRC_VISION) && (CAMERA_ENABLE)
            /* 视觉源:整页让给实时画面(见 menu_draw_s3_live),看识别对不对。 */
            menu_draw_s3_live();
#elif (S3_FOLLOW_SRC == S3_FOLLOW_SRC_PLINK)
            /* PLINK 源：没图可出（图像在 4D7 侧），改出链路与跟随数字。
             * 【条件为何不带 CAMERA_ENABLE】它必须跟 menu_draw_s3_plink 的定义
             * 条件逐字一致，而那边没有——PLINK 不碰摄头，多卡一个条件反而会在
             * 关摄头的版本里把这一页静静落回下面那三行遥控文案，
             * 让现场以为要拿遥控开车。 */
            menu_draw_s3_plink();
#else
            ui_bar(UI_Y_HEAD,        " Recording",         UI_ERR,  UI_BG);
            ui_bar(UI_Y_ROW0,        " RC drive the maze", UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, " Stop car: auto reverse",  UI_WARN, UI_BG);
            ui_hint(" auto  START now  KART_LEFT exit");
#endif
            break;
        case S3_PHASE2_REVERSE:
            /* 原文 "Openloop, no turn" 已过期:倒车段会跟着录制打角走,
             * 且带航向 P 纠偏(PLAYBACK_OL_HEAD_EN=1),S3 OLMode=1 时
             * 再加横向位置 P。"不转向"是最早那版纯开环留下的说法。 */
            ui_title("Subject 3", "BACK");
            ui_bar(UI_Y_HEAD,        " Reversing...",      UI_WARN, UI_BG);
            ui_bar(UI_Y_ROW0,        " Retrace + heading corr", UI_FG, UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, "",                   UI_FG,   UI_BG);
            ui_hint("");
            break;
        case S3_FIXED_ACT:
            /* 倒车复现完成后自动跑的固定动作(出库 2.8m → 倒回 2.8m)。
             * 【不进 menu_poll 的重绘例外名单】这一段车正在动,整屏 SPI 写会挤掉
             * 控制拍;所以这一页只在换页/手动重绘时画得出来,正常跑动时屏上
             * 停留的还是 BACK 那页。这是故意的,别为了"屏上好看"去加重绘。 */
            ui_title("Subject 3", "AUTO");
            ui_bar(UI_Y_HEAD,        " Fixed action",      UI_WARN, UI_BG);
            ui_bar(UI_Y_ROW0,        " Out 2.8m / back 2.8m", UI_FG, UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, " no START needed",   UI_FG,   UI_BG);
            ui_hint("");
            break;
        case S3_SIGNAL:
            /* 已回发车区,等语音口令做灯光/鸣笛。这一阶段【没有 VOFA 日志】
             * (语音与日志共用 UART10,已切 115200),屏幕是唯一的现场反馈。 */
            ui_title("Subject 3", "VOICE");
            ui_bar(UI_Y_HEAD,        " Back at start",     UI_OK,   UI_BG);
            ui_bar(UI_Y_ROW0,        " Say kart_light / kart_horn",  UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, " log off (kart_voice uart)", UI_WARN, UI_BG);
            ui_hint(" KART_LEFT exit");
            break;
        case S3_FINISHED:
            ui_title("Subject 3", "DONE");
            ui_bar(UI_Y_HEAD,        " Finished",          UI_OK,   UI_BG);
            ui_bar(UI_Y_ROW0,        " Back at start",     UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, "",                   UI_FG,   UI_BG);
            ui_hint(" KART_LEFT exit");
            break;
        case S3_FAULT:
        default:
            ui_title("Subject 3", "FAULT");
            ui_bar(UI_Y_HEAD,        " FAULT",             UI_ERR,  UI_BG);
            ui_bar(UI_Y_ROW0,        " Path invalid",      UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, "",                   UI_FG,   UI_BG);
            ui_hint(" KART_LEFT exit");
            break;
    }
}

static void menu_draw_subject1(void)
{
    ui_title("Subject 1", "Slalom");
    ui_bar(UI_Y_HEAD, " Record / kart_playback path", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Record Path",
            (uint8)(cursor_s1 == MENU_S1_RECORD));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Playback Path",
            (uint8)(cursor_s1 == MENU_S1_PLAYBACK));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "View Sampled Path",
            (uint8)(cursor_s1 == MENU_S1_VIEW_PATH));
    ui_item(UI_Y_ROW0 + 3 * UI_ROW_H, "Enter Remote",
            (uint8)(cursor_s1 == MENU_S1_ENTER_REMOTE));
    ui_item(UI_Y_ROW0 + 4 * UI_ROW_H, "Exit Remote",
            (uint8)(cursor_s1 == MENU_S1_EXIT_REMOTE));

    ui_hint(" MID enter  KART_LEFT back");
}

static void menu_draw_subject2(void)
{
    ui_title("Subject 2", "Voice");
    ui_bar(UI_Y_HEAD, " Voice / gate path", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Voice A  auto ret",
            (uint8)(cursor_s2 == MENU_S2_VOICE));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Voice B  RC ret",
            (uint8)(cursor_s2 == MENU_S2_VOICE_B));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "Gate Recording",
            (uint8)(cursor_s2 == MENU_S2_GATE));
    ui_item(UI_Y_ROW0 + 3 * UI_ROW_H, "Return Path",
            (uint8)(cursor_s2 == MENU_S2_RET));
    ui_item(UI_Y_ROW0 + 4 * UI_ROW_H, "Back",
            (uint8)(cursor_s2 == MENU_S2_BACK));

    ui_hint(" MID enter  KART_LEFT back");
}

static void menu_draw_s2_voice(void)
{
    uint8 man = kart_mission_subject2_get_manual_return();

    ui_title(man ? "Voice B" : "Voice A", "LIVE");
    ui_bar(UI_Y_HEAD, " Speak command", UI_OK, UI_BG);
    ui_bar(UI_Y_ROW0, " Listening...",  UI_FG, UI_BG);
    ui_bar(UI_Y_ROW0 + UI_ROW_H,
           man ? " Return: RC + kart_voice" : " Return: auto GOTO",
           man ? UI_WARN : UI_KEY, UI_BG);
    ui_hint(" MID exit  KART_LEFT exit");
}

static void menu_draw_s2_gate(void)
{
    ui_title("Gate Recording", NULL);
    ui_bar(UI_Y_HEAD, " Gate path slots 1-5", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Record",
            (uint8)(cursor_s2_gate == MENU_S2_GATE_RECORD));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Playback",
            (uint8)(cursor_s2_gate == MENU_S2_GATE_PLAYBACK));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "Back",
            (uint8)(cursor_s2_gate == MENU_S2_GATE_BACK));

    ui_hint(" MID enter  KART_LEFT back");
}

/* 录制等待/录制中:开录和停录走的是 menu_scan_keys 里的 mid_edge —— 五向 MID
 * 或按下旋钮,跟独立 START 键无关。所以提示必须写 MID,不能写 START。 */
static void menu_draw_recording_wait(void)
{
    ui_title("Record", "READY");
    ui_bar(UI_Y_HEAD,      " Ready to kart_record",     UI_OK,   UI_BG);
    ui_bar(UI_Y_ROW0,           " RC drive to start pt", UI_FG,  UI_BG);
    ui_bar(UI_Y_ROW0 + UI_ROW_H, " Then press MID",      UI_WARN, UI_BG);
    ui_hint(" MID start rec");
}

/* 录制中只有点数在变。单独抽一行出来每拍刷,整页只在进入时画一次:
 * 录制时人在遥控开车,这时候整页重画(尤其 clear)会占住控制窗口。 */
static void menu_draw_rec_count(void)
{
    char buf[32];

    sprintf(buf, " points %-6u", (unsigned)kart_record_get_count());
    ui_bar(UI_Y_ROW0, buf, UI_NUM, UI_BG);
}

static void menu_draw_recording_active(void)
{
    ui_title("Record", "REC");
    ui_bar(UI_Y_HEAD, " Recording...", UI_ERR, UI_BG);

    /* 点数实时涨,一眼能看出录制真的在采样(以前要看串口才知道)。 */
    menu_draw_rec_count();

    ui_bar(UI_Y_ROW0 + UI_ROW_H, " Press MID to stop", UI_WARN, UI_BG);
    ui_hint(" MID stop rec");
}

/* 槽位一行:"名字 [n pts]" 或 "名字 [empty]"。空槽用灰字,一眼能挑出没录的。 */
static void menu_draw_slot_row(uint16 y, const char *name, uint16 count, uint8 selected)
{
    char buf[48];

    if(count > 0) sprintf(buf, "%-12s %-5u pts", name, (unsigned)count);
    else          sprintf(buf, "%-12s empty    ", name);

    if(selected)       ui_bar(y, buf, UI_SEL_FG, UI_SEL_BG);
    else if(count > 0) ui_bar(y, buf, UI_FG,  UI_BG);
    else               ui_bar(y, buf, UI_DIM, UI_BG);
}

static const char* menu_gate_slot_name(uint8 i)
{
    static const char* names[KART_MENU_S2_GATE_SLOT_NUM] =
        { "Gate1 Left", "Gate1", "Gate2", "Gate3", "Gate3 Right" };
    return (i < KART_MENU_S2_GATE_SLOT_NUM) ? names[i] : "?";
}

static void menu_draw_slot_save(void)
{
    uint8 i;
    char buf[24];

    ui_title("Save Path", "S1");
    ui_bar(UI_Y_HEAD, " Pick a slot to save", UI_KEY, UI_BG);

    for(i = 0; i < KART_MENU_S1_SLOT_NUM; i++)
    {
        sprintf(buf, "Slot%u", (unsigned)(i + 1));
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), buf,
                           kart_flash_slot_count(i), (uint8)(cursor_slot == i));
    }

    ui_item((uint16)(UI_Y_ROW0 + KART_MENU_S1_SLOT_NUM * UI_ROW_H), "Don't Save",
            (uint8)(cursor_slot == KART_MENU_S1_SLOT_NUM));

    ui_hint(" MID confirm  KART_LEFT discard");
}

static void menu_draw_slot_load(void)
{
    uint8 i;
    char buf[24];

    ui_title("Load Path", "S1");
    ui_bar(UI_Y_HEAD, " Pick a slot to load", UI_KEY, UI_BG);

    for(i = 0; i < KART_MENU_S1_SLOT_NUM; i++)
    {
        sprintf(buf, "Slot%u", (unsigned)(i + 1));
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), buf,
                           kart_flash_slot_count(i), (uint8)(cursor_slot == i));
    }

    ui_hint(" MID load  KART_LEFT back");
}

static void menu_draw_s2_gate_slot_save(void)
{
    uint8 i;

    ui_title("Save Gate Path", "S2");
    ui_bar(UI_Y_HEAD, " Pick a gate slot", UI_KEY, UI_BG);

    for(i = 0; i < KART_MENU_S2_GATE_SLOT_NUM; i++)
    {
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), menu_gate_slot_name(i),
                           kart_flash_slot_count((uint8)(i + 1)), (uint8)(cursor_slot == i));
    }

    ui_item((uint16)(UI_Y_ROW0 + KART_MENU_S2_GATE_SLOT_NUM * UI_ROW_H), "Don't Save",
            (uint8)(cursor_slot == KART_MENU_S2_GATE_SLOT_NUM));

    ui_hint(" MID confirm  KART_LEFT discard");
}

static void menu_draw_s2_gate_slot_load(void)
{
    uint8 i;

    ui_title("Load Gate Path", "S2");
    ui_bar(UI_Y_HEAD, " Pick a gate slot", UI_KEY, UI_BG);

    for(i = 0; i < KART_MENU_S2_GATE_SLOT_NUM; i++)
    {
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), menu_gate_slot_name(i),
                           kart_flash_slot_count((uint8)(i + 1)), (uint8)(cursor_slot == i));
    }

    ui_hint(" MID load  KART_LEFT back");
}

static const char* menu_ret_slot_name(uint8 i)
{
    static const char* names[MENU_S2_RET_SLOT_NUM] =
        { "Ret1 Right", "Ret1", "Ret2", "Ret3", "Ret3 Left" };
    return (i < MENU_S2_RET_SLOT_NUM) ? names[i] : "?";
}

static void menu_draw_s2_ret(void)
{
    ui_title("Return Path", NULL);
    ui_bar(UI_Y_HEAD, " Return slots 6-10", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Record",
            (uint8)(cursor_s2_ret == MENU_S2_RET_RECORD));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Playback",
            (uint8)(cursor_s2_ret == MENU_S2_RET_PLAYBACK));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "Back",
            (uint8)(cursor_s2_ret == MENU_S2_RET_BACK));

    ui_hint(" MID enter  KART_LEFT back");
}

static void menu_draw_s2_ret_slot_save(void)
{
    uint8 i;

    ui_title("Save Return Path", "S2");
    ui_bar(UI_Y_HEAD, " Pick a return slot", UI_KEY, UI_BG);

    for(i = 0; i < MENU_S2_RET_SLOT_NUM; i++)
    {
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), menu_ret_slot_name(i),
                           kart_flash_slot_count((uint8)(i + FLASH_S2R_FIRST_SLOT)),
                           (uint8)(cursor_slot == i));
    }

    ui_item((uint16)(UI_Y_ROW0 + MENU_S2_RET_SLOT_NUM * UI_ROW_H), "Don't Save",
            (uint8)(cursor_slot == MENU_S2_RET_SLOT_NUM));

    ui_hint(" MID confirm  KART_LEFT discard");
}

static void menu_draw_s2_ret_slot_load(void)
{
    uint8 i;

    ui_title("Load Return Path", "S2");
    ui_bar(UI_Y_HEAD, " Pick a return slot", UI_KEY, UI_BG);

    for(i = 0; i < MENU_S2_RET_SLOT_NUM; i++)
    {
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), menu_ret_slot_name(i),
                           kart_flash_slot_count((uint8)(i + FLASH_S2R_FIRST_SLOT)),
                           (uint8)(cursor_slot == i));
    }

    ui_hint(" MID run  KART_LEFT back");
}

static void menu_draw_s1_ready(void)
{
    char buf[40];
    uint8 prof_on = (uint8)(kart_params_get(PARAM_PB_PROF) > 0.5f);

    /* 标题右角标 * = 改了还没进 Flash;按 KART_LEFT 退出这一页会自动存(星号随之消失)。 */
    ui_title("Ready", kart_params_is_dirty() ? "*" : " ");
    ui_bar(UI_Y_HEAD, " Path loaded, at start pt", UI_OK, UI_BG);

    /* 就地显示复现胆量,knob/UP/DN 直接改。改完按 START 才生效(剖面在 start 时重算)。
     * Prof=OFF 时倍率不起作用(速度仍取录制速度),所以把开关状态一并显示出来,
     * 免得在场上狂拧旋钮却毫无变化。 */
    sprintf(buf, " Speed  x%-5.2f", kart_params_get(PARAM_PB_SCALE));
    ui_bar(UI_Y_ROW0, buf, UI_NUM, UI_BG);

    sprintf(buf, " Profile %s", prof_on ? "ON " : "OFF");
    ui_bar(UI_Y_ROW0 + UI_ROW_H, buf, prof_on ? UI_OK : UI_DIM, UI_BG);

    if(!prof_on)
    {
        ui_bar(UI_Y_ROW0 + 2 * UI_ROW_H, " Speed x has no effect", UI_WARN, UI_BG);
    }
    else
    {
        ui_bar(UI_Y_ROW0 + 2 * UI_ROW_H, "", UI_FG, UI_BG);
    }

    /* START 是独立的物理发车键(P20.7,kart_mission.c 检边沿),不是菜单 MID。 */
    ui_hint(" knob kart_speed  START go  KART_LEFT back");
}

/* 离开可调参界面时自动落盘。只有改过才擦写 DFlash(数 ms 阻塞),没改一个字节不写。
 * 只在静止界面调用:车在跑时 kart_menu_poll 已提前 return,按键进不来。
 * 写了才闪提示,让人在按 reset 之前确认值已经进 Flash。 */

/* 全屏闪一条提示(存 Flash 成功、载入成功之类的确认)。自己擦屏+阻塞延时,
 * 所以必须置 need_clear:下一拍整屏清掉再重画,否则提示字残留在菜单上。
 * 只在静止界面调:车在跑时 kart_menu_poll 早就 return 了,按键进不来。 */
static void menu_flash_notice(const char *msg, uint32 ms)
{
    char buf[UI_COLS + 1];
    uint8 n = 0, pad, i;

    while(msg[n] != '\0' && n < UI_COLS) n++;
    pad = (uint8)((UI_COLS - n) / 2u);       /* 在整行蓝底上居中,别贴左边 */
    for(i = 0; i < pad; i++) buf[i] = ' ';
    for(i = 0; i < n; i++)   buf[pad + i] = msg[i];
    buf[pad + n] = '\0';

    ui_clear();
    menu_draw_status_bar();
    ui_bar(UI_Y_ROW0 + UI_ROW_H, buf, UI_BAR_FG, UI_BAR_BG);
    system_delay_ms(ms);
    need_clear = 1;
}

static void menu_params_autosave(void)
{
    if(kart_params_save_if_dirty())
    {
        menu_flash_notice("Params Saved!", 400);
    }
}

static void menu_handle_key_mid_press(void)
{
    /* 清 row_only:MID/KART_LEFT 会改标题栏、副标题、底部提示(进出编辑态等),
     * 只重画光标那一行盖不住它们。同一个 50ms 画屏周期内先改值再按 MID 时会遇到。 */
    need_repaint = 1;
    repaint_row_only = 0;
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
                /* 不能直接置 0:第 0 行现在是分组标题,不可选中。 */
                menu_settings_cursor_first();
                set_edit = 0;
            }
            else if(cursor_main == MENU_MAIN_CAMERA)
            {
                /* 摄像头调试页。【强制 IDLE】:这一页每 50ms 要出一整幅
                 * 160x120 图 + 跑一遍视觉,比普通页面重得多;万一是从别的模式
                 * 误点进来的,车还带着输出就危险了。进来先把车完整停机。
                 * 这也让"这一页可以慢"这个前提成立(见 menu_draw_camera 头注释)。 */
                if(kart_mission_get_mode() != MISSION_IDLE)
                {
                    kart_mission_set_mode(MISSION_IDLE);
                }
                current_level = MENU_LEVEL_CAMERA;
            }
            else if(cursor_main == MENU_MAIN_SUBJECT3)
            {
                /* 进科目三即发车:同科目三录制入口(enter 清 kart_odom+开录制+遥控接管)。
                 * 遥控开车走迷宫,到停车区按一次物理 START 键 → 停录并直接开环倒车返回,
                 * 车头不掉转。停 S3_RUN 屏:菜单不吃 MID(防运行中刷屏),发车用独立 START 键。 */
                kart_mission_set_mode(MISSION_SUBJECT_3);
                current_level = MENU_LEVEL_S3_RUN;
            }
            break;

        case MENU_LEVEL_SUBJECT1:
            if(cursor_s1 == MENU_S1_RECORD)
            {
                kart_mission_set_mode(MISSION_REMOTE);
                rec_target = 0;
                rec_state = REC_STATE_WAIT_START;
            }
            else if(cursor_s1 == MENU_S1_PLAYBACK)
            {
                current_level = MENU_LEVEL_S1_SLOT_LOAD;
                cursor_slot = 0;
            }
            else if(cursor_s1 == MENU_S1_VIEW_PATH)
            {
                /* 可视化的权威数据源是科目一 Flash 槽 0。
                 * kart_traj_view_draw() 画的是 kart_record RAM 缓冲，所以进页前先把
                 * Flash 路径读回 RAM；否则冷启动后未走 Playback 载入时，页面
                 * 会误报 "No path in RAM"，即使槽 0 已经有存档。
                 * 进页前强制停车，避免 Flash 读取和整页画线挤进控制拍。 */
                if(kart_mission_get_mode() != MISSION_IDLE)
                    kart_mission_set_mode(MISSION_IDLE);
                (void)kart_record_load_from_flash(0);
                traj_cursor = 0;
                traj_edit = 0;
                traj_dirty = 0;
                kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
                current_level = MENU_LEVEL_S1_TRAJ;
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
            if(cursor_s2 == MENU_S2_VOICE || cursor_s2 == MENU_S2_VOICE_B)
            {
                kart_mission_subject2_set_manual_return(
                    (uint8)(cursor_s2 == MENU_S2_VOICE_B));
                /* 真正进科目二状态机:subject2_loop 才会每拍跑 kart_voice_dispatch+kart_motion_update,
                 * 运动指令的判停/deadman急停/蛇形翻打角靠它推进。只切菜单界面车会裸奔。
                 * 2026-07-29:原来这里先 kart_odom_reset() 再切模式,现在清零由
                 * mission_enter(MISSION_SUBJECT_2) 统一做(它在 set_mode 里、更靠后),
                 * 发车区原点只有一个来源,免得两处各清一次将来改漏一处。 */
                kart_mission_set_mode(MISSION_SUBJECT_2);
                current_level = MENU_LEVEL_S2_VOICE;
            }
            else if(cursor_s2 == MENU_S2_GATE)
            {
                current_level = MENU_LEVEL_S2_GATE;
                cursor_s2_gate = 0;
            }
            else if(cursor_s2 == MENU_S2_RET)
            {
                current_level = MENU_LEVEL_S2_RET;
                cursor_s2_ret = 0;
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
        {
            /* cursor_set 现在是显示行号,不再等于参数 id,先查表看这一行是什么。
             * 标题行落不到这里 —— menu_settings_seek/cursor_first 保证光标不停在标题上,
             * 但仍按"不是参数就什么都不做"处理,免得将来加行时误触发。 */
            uint8 row_id = (cursor_set < MENU_SET_TOTAL) ? set_rows[cursor_set].id
                                                        : SET_ROW_HEAD;

            if(row_id == SET_ROW_SAVE)
            {
                /* 阻塞擦写一页 DFlash(数 ms)。Settings 页只在静止时进得来,
                 * 车在跑(kart_playback_is_running)时 kart_menu_poll 早就 return 了,
                 * 按键根本进不到这里,不会在运行中插入 Flash 擦写。 */
                kart_params_save();
                menu_flash_notice("Params Saved!", 800);
            }
            else if(row_id == SET_ROW_DEF)
            {
                kart_params_load_default();
                menu_flash_notice("Default Loaded", 800);
            }
            else if(row_id < PARAM_MAX)
            {
                set_edit = set_edit ? 0 : 1;     /* MID 切换 移光标/改值 */
            }
            break;
        }

        case MENU_LEVEL_CAMERA:
            /* MID 切换叠加层。关掉是为了看清原图 —— 框和准星画在黄色上面,
             * 判断"这块颜色够不够黄色"时那几条线本身就是干扰。 */
            camdbg_show_overlay = camdbg_show_overlay ? 0 : 1;
            break;

        case MENU_LEVEL_S1_TRAJ:
            if(kart_record_get_count() >= 2U)
            {
                traj_edit = traj_edit ? 0U : 1U;
                kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
                need_clear = 1;
            }
            break;

        case MENU_LEVEL_S2_GATE:
            if(cursor_s2_gate == MENU_S2_GATE_RECORD)
            {
                kart_mission_set_mode(MISSION_REMOTE);
                rec_target = 1;
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
                    menu_flash_notice("Saved OK!", 1000);
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
                    menu_flash_notice("Playback Started!", 1000);
                }
            }
            current_level = MENU_LEVEL_S2_GATE;
            break;

        case MENU_LEVEL_S2_RET:
            if(cursor_s2_ret == MENU_S2_RET_RECORD)
            {
                kart_mission_set_mode(MISSION_REMOTE);
                rec_target = 2;
                rec_state = REC_STATE_WAIT_START;
            }
            else if(cursor_s2_ret == MENU_S2_RET_PLAYBACK)
            {
                current_level = MENU_LEVEL_S2_RET_SLOT_LOAD;
                cursor_slot = 0;
            }
            else if(cursor_s2_ret == MENU_S2_RET_BACK)
            {
                current_level = MENU_LEVEL_SUBJECT2;
            }
            break;

        case MENU_LEVEL_S2_RET_SLOT_SAVE:
            if(cursor_slot < MENU_S2_RET_SLOT_NUM)
            {
                uint8 ret = kart_record_save_to_flash(
                                (uint8)(cursor_slot + FLASH_S2R_FIRST_SLOT));
                if(ret == 0)
                {
                    menu_flash_notice("Saved OK!", 1000);
                }
            }
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_S2_RET;
            break;

        /* 台上试跑走方案B(就地起跑),跟场上人工摆位后的回退路径是同一份代码。 */
        case MENU_LEVEL_S2_RET_SLOT_LOAD:
            if(cursor_slot < MENU_S2_RET_SLOT_NUM)
            {
                if(kart_flash_slot_count((uint8)(cursor_slot + FLASH_S2R_FIRST_SLOT)) > 0)
                {
                    if(kart_mission_get_mode() == MISSION_REMOTE)
                        kart_mission_set_mode(MISSION_IDLE);
                    kart_mission_set_mode(MISSION_SUBJECT_2);
                    if(kart_mission_subject2_start_return_here(cursor_slot))
                        menu_flash_notice("Return Started!", 1000);
                }
            }
            current_level = MENU_LEVEL_S2_RET;
            break;

        case MENU_LEVEL_S1_SLOT_SAVE:
            if(cursor_slot < KART_MENU_S1_SLOT_NUM)
            {
                uint8 ret = kart_record_save_to_flash(cursor_slot);
                if(ret == 0)
                {
                    menu_flash_notice("Saved OK!", 1000);
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
                    /* 这里的 START 是独立的物理 START 键(BOARD_START_KEY_PIN),
                     * 由 kart_mission.c 检边沿,不是菜单 MID —— 文字别改成 MID。 */
                    menu_flash_notice("Loaded, Press START", 1000);
                    /* 停在专用就绪界面:菜单不吃 MID,按独立 START 键发车。 */
                    current_level = MENU_LEVEL_S1_READY;
                }
                else
                {
                    menu_flash_notice("Load Failed!", 1000);
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

    need_repaint = 1;
    repaint_row_only = 0;       /* 见 MID 处注释 */
    switch(current_level)
    {
        case MENU_LEVEL_SUBJECT1:
        case MENU_LEVEL_SUBJECT2:
            current_level = MENU_LEVEL_MAIN;
            break;

        case MENU_LEVEL_S2_VOICE:
            /* KART_LEFT 退出语音同样完整停机。 */
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
            /* 就绪界面 KART_LEFT 退回:一并退出科目一,防遥控/复现残留。 */
            if(kart_mission_get_mode() != MISSION_IDLE)
                kart_mission_set_mode(MISSION_IDLE);
            /* 这一页 UP/DOWN 能热调 PB Scale,退出时同样落盘。 */
            menu_params_autosave();
            current_level = MENU_LEVEL_SUBJECT1;
            break;

        case MENU_LEVEL_S1_TRAJ:
            if(traj_edit)
            {
                if(kart_record_adjust_segment(traj_cursor, TRAJ_EDIT_RADIUS,
                                              -TRAJ_EDIT_STEP_M, 0.0f))
                    traj_dirty = 1;
                kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
                need_clear = 1;
            }
            else
            {
                current_level = MENU_LEVEL_SUBJECT1;
            }
            break;

        case MENU_LEVEL_S3_RUN:
            /* 科目三运行界面 KART_LEFT 退出:完整停机退回 IDLE,防遥控/录制/复现残留。 */
            if(kart_mission_get_mode() != MISSION_IDLE)
                kart_mission_set_mode(MISSION_IDLE);
            current_level = MENU_LEVEL_MAIN;
            break;

        case MENU_LEVEL_CAMERA:
            /* 退出调试页。这里【必须还帧】:本页是"ready 就取、画完就还",
             * 正常路径每次都配平了;但按键是 10ms 拍处理的,和 50ms 的画屏
             * 不同拍 —— 万一在取到帧、还没画完的当口退出,那一帧就一直被
             * hold 住,相机再也取不到新帧,回到跟随时表现为"视觉全程丢目标"。
             * 没持帧时调它并非完全空操作:它会把 finish_flag 一并清掉,
             * 可能丢掉刚到的一帧 —— 退页面时丢一帧无所谓,而漏还帧会卡死整条链路,
             * 所以宁可无条件还一次。 */
            kart_camera_frame_release();
            current_level = MENU_LEVEL_MAIN;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_SAVE:
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_S2_GATE;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_LOAD:
            current_level = MENU_LEVEL_S2_GATE;
            break;

        case MENU_LEVEL_S2_RET:
            current_level = MENU_LEVEL_SUBJECT2;
            break;

        case MENU_LEVEL_S2_RET_SLOT_SAVE:
            rec_state = REC_STATE_IDLE;
            current_level = MENU_LEVEL_S2_RET;
            break;

        case MENU_LEVEL_S2_RET_SLOT_LOAD:
            current_level = MENU_LEVEL_S2_RET;
            break;

        default:
            break;
    }
}

/* mul = 改值时一次走几个步长(1=细调,MENU_KEY_STEP_FAST=粗调)。
 * 光标移动一律只走一行,不受 mul 影响 —— 长按/快旋时"重复得更快"就够了,
 * 一次跳 10 行会直接窜到底,反而更难停在想要的项上。 */
static void menu_handle_key_up_press(uint8 mul)
{
    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
        return;

    need_repaint = 1;
    /* 默认整页重画,只有下面"编辑态改值"那一支才降级成单行 —— 移光标要同时擦掉
     * 旧行的高亮底、画上新行的,单行盖不住。 */
    repaint_row_only = 0;
    switch(current_level)
    {
        case MENU_LEVEL_MAIN:
            if(cursor_main > 0) cursor_main--;
            break;

        case MENU_LEVEL_CAMERA:
            /* 准星上移。mul 是长按加速倍率,直接用上:120 行靠单步挪太慢。 */
            camdbg_probe_y = (int16)(camdbg_probe_y - (int16)mul);
            if(camdbg_probe_y < 0) { camdbg_probe_y = 0; }
            break;

        case MENU_LEVEL_S1_TRAJ:
            if(traj_edit)
            {
                if(kart_record_adjust_segment(traj_cursor, TRAJ_EDIT_RADIUS,
                                              0.0f, TRAJ_EDIT_STEP_M * (float)mul))
                    traj_dirty = 1;
            }
            else
            {
                uint16 step = (uint16)(TRAJ_SELECT_STEP * mul);
                traj_cursor = (traj_cursor > step) ? (uint16)(traj_cursor - step) : 0U;
            }
            kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
            need_clear = 1;
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

        case MENU_LEVEL_S2_RET:
            if(cursor_s2_ret > 0) cursor_s2_ret--;
            break;

        case MENU_LEVEL_S2_RET_SLOT_SAVE:
        case MENU_LEVEL_S2_RET_SLOT_LOAD:
            if(cursor_slot > 0) cursor_slot--;
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
            if(set_edit)
            {
                /* 编辑态:只有当前那一行的数字变了,整屏没必要重画。
                 * cursor_set 现在是显示行号,要先翻成参数 id 再调值。 */
                uint8 kart_pid = menu_settings_param_id();
                if(kart_pid < PARAM_MAX)
                {
                    kart_params_step_mul(kart_pid, +1, mul);
                    /* 改到"管别人灰不灰"的项就得整页重画,否则别的行灰着不变。 */
                    repaint_row_only = (uint8)(!menu_param_gates_others(kart_pid));
                }
            }
            else menu_settings_seek(-1);        /* 上移一格,跳过分组标题 */
            break;

        /* 科目一就绪界面:UP/DOWN 热调复现速度倍率(PB Scale)。
         * 场地上试速度不用退菜单:UP 加胆量、DOWN 减胆量,再按 START 跑一趟。
         * 只改 RAM 值,满意了再进 Settings 存 Flash。剖面在 kart_playback_start 时重算,
         * 故改完必须重跑才生效(跑动中不会中途变速)。
         * 科目三界面不挂这个:它走开环倒车固定速,不用剖面,且流程已自动推进,
         * 不在这里插任何按键行为。 */
        case MENU_LEVEL_S1_READY:
            kart_params_step_mul(PARAM_PB_SCALE, +1, mul);
            break;

        default:
            break;
    }
}

static void menu_handle_key_down_press(uint8 mul)
{
    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
        return;

    need_repaint = 1;
    repaint_row_only = 0;       /* 见 UP 处注释 */
    switch(current_level)
    {
        case MENU_LEVEL_MAIN:
            if(cursor_main < MENU_MAIN_MAX - 1) cursor_main++;
            break;

        case MENU_LEVEL_CAMERA:
            camdbg_probe_y = (int16)(camdbg_probe_y + (int16)mul);
            if(camdbg_probe_y > (int16)(SCC8660_H - 1))
            {
                camdbg_probe_y = (int16)(SCC8660_H - 1);
            }
            break;

        case MENU_LEVEL_S1_TRAJ:
        {
            uint16 n = kart_record_get_count();
            if(traj_edit)
            {
                if(kart_record_adjust_segment(traj_cursor, TRAJ_EDIT_RADIUS,
                                              0.0f, -TRAJ_EDIT_STEP_M * (float)mul))
                    traj_dirty = 1;
            }
            else if(n > 0U)
            {
                uint32 next = (uint32)traj_cursor + (uint32)TRAJ_SELECT_STEP * mul;
                traj_cursor = (next < n) ? (uint16)next : (uint16)(n - 1U);
            }
            kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
            need_clear = 1;
            break;
        }

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
            /* 载入页没有"Don't Save"那一行,末项就是最后一个槽位,故 -1。
             * 写成 cursor_slot+1 而不是 NUM-1:槽位数=1 时 NUM-1 为 0,
             * uint8 与 0 比大小编译器会报"恒为假"。 */
            if((int)cursor_slot + 1 < KART_MENU_S1_SLOT_NUM) cursor_slot++;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_SAVE:
            if(cursor_slot < KART_MENU_S2_GATE_SLOT_NUM) cursor_slot++;
            break;

        case MENU_LEVEL_S2_GATE_SLOT_LOAD:
            if(cursor_slot < KART_MENU_S2_GATE_SLOT_NUM - 1) cursor_slot++;
            break;

        case MENU_LEVEL_S2_RET:
            if(cursor_s2_ret < MENU_S2_RET_MAX - 1) cursor_s2_ret++;
            break;

        case MENU_LEVEL_S2_RET_SLOT_SAVE:
            if(cursor_slot < MENU_S2_RET_SLOT_NUM) cursor_slot++;
            break;

        case MENU_LEVEL_S2_RET_SLOT_LOAD:
            if(cursor_slot < MENU_S2_RET_SLOT_NUM - 1) cursor_slot++;
            break;

        case MENU_LEVEL_SETTINGS:
            if(set_edit)
            {
                uint8 kart_pid = menu_settings_param_id();        /* 见 UP 处注释 */
                if(kart_pid < PARAM_MAX)
                {
                    kart_params_step_mul(kart_pid, -1, mul);
                    repaint_row_only = (uint8)(!menu_param_gates_others(kart_pid));
                }
            }
            else menu_settings_seek(+1);        /* 下移一格,跳过分组标题 */
            break;

        case MENU_LEVEL_S1_READY:
            kart_params_step_mul(PARAM_PB_SCALE, -1, mul);
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
            break;

        case REC_STATE_RECORDING:
            kart_record_stop();
            rec_state = REC_STATE_SAVE_PROMPT;
            current_level = (rec_target == 2) ? MENU_LEVEL_S2_RET_SLOT_SAVE :
                            (rec_target == 1) ? MENU_LEVEL_S2_GATE_SLOT_SAVE
                                              : MENU_LEVEL_S1_SLOT_SAVE;
            cursor_slot = 0;
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
 * 攒够 MENU_ENC_DIV 个边沿才产出一格,余数留在 enc_accum,慢旋不丢。 */
void kart_menu_enc_poll(void)
{
    uint8 a = gpio_get_level(MENU_ENC_A);
    uint8 b = gpio_get_level(MENU_ENC_B);

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

    while(enc_accum >= MENU_ENC_DIV)
    {
        enc_accum = (int8)(enc_accum - MENU_ENC_DIV);
        if(enc_detent < MENU_ENC_PEND_MAX) enc_detent++;
    }
    while(enc_accum <= -MENU_ENC_DIV)
    {
        enc_accum = (int8)(enc_accum + MENU_ENC_DIV);
        if(enc_detent > -MENU_ENC_PEND_MAX) enc_detent--;
    }
}

/* 长按判定:hold 是"按住了多少个 10ms 拍"。返回 1 = 这一拍要产出一次动作。
 * 时序:按下瞬间出 1 次 → 静默 300ms(防手抖变连发) → 之后每 100ms 出一次。
 * mul 回写粗调倍率:按住超过 1s 才升级,短按到手的永远是最小步长。 */
static uint8 menu_key_hold_fire(uint8 level, uint16 *hold, uint8 *mul)
{
    *mul = 1u;

    if(level != 0)              /* 松开(上拉,读 1) */
    {
        *hold = 0;
        return 0;
    }

    if(*hold < MENU_KEY_HOLD_MAX) (*hold)++;

    if(*hold == 1u) return 1;                       /* 按下第一拍:出一次 */
    if(*hold < MENU_KEY_REPEAT_DELAY) return 0;     /* 还在防抖静默期 */

    if(*hold >= MENU_KEY_REPEAT_FAST) *mul = MENU_KEY_STEP_FAST;

    /* 相对 DELAY 起点算相位,保证重复间隔严格 100ms。 */
    return (uint8)(((*hold - MENU_KEY_REPEAT_DELAY) % MENU_KEY_REPEAT_PERIOD) == 0u);
}

static void menu_scan_keys(void)
{
    uint8 key_mid = gpio_get_level(KART_MENU_KEY_MID);
    uint8 key_up = gpio_get_level(KART_MENU_KEY_UP);
    uint8 key_down = gpio_get_level(KART_MENU_KEY_DOWN);
    uint8 key_left = gpio_get_level(KART_MENU_KEY_LEFT);
    uint8 key_right = gpio_get_level(KART_MENU_KEY_RIGHT);
    uint8 key_start = gpio_get_level(BOARD_START_KEY_PIN);
    uint8 key_esw = gpio_get_level(MENU_ENC_SW);
    uint8 mid_edge;
    uint8 up_mul, down_mul, right_mul;
    /* 取走本拍解码出的格数。解码和这里都在主循环上下文(同一个 10ms 拍里顺序调),
     * 不是中断,不存在读改写竞争,不需要临界区。 */
    int8  enc = enc_detent;
    enc_detent = 0;

    /* 快旋判定:上一格离现在多久。≤60ms 认为在连续拧,改值直接上粗调倍率 ——
     * "从 0.50 拧到 2.00" 由 30 格变成 3 格,一把就到。慢慢拧仍是细调。
     * enc_gap 只在有格产出时归零,所以停手后下一格必定按细调起步。 */
    if(enc_gap < 0xFFFFu) enc_gap++;
    if(enc != 0)
    {
        enc_fast = (uint8)(enc_gap <= MENU_ENC_FAST_GAP);
        enc_gap = 0;
    }

    /* 旋钮按下(P20.6)与五向 MID(P33.4)等价,取"任一下降沿"。
     * 两个脚独立,不会互相影响;哪个手顺用哪个。 */
    mid_edge = ((key_mid == 0 && key_mid_last == 1) ||
                (key_esw == 0 && enc_sw_last  == 1)) ? 1u : 0u;

    if(rec_state == REC_STATE_WAIT_START || rec_state == REC_STATE_RECORDING)
    {
        if(mid_edge)
            menu_handle_recording_mid_press();
    }
    else if(current_level == MENU_LEVEL_S1_READY || current_level == MENU_LEVEL_S3_RUN)
    {
        /* 这两个界面 MID 无对应菜单项,但 menu_handle_key_mid_press 一进去就
         * 置 need_repaint → 会在等发车/倒车推进期间插一次 IPS200 刷新
         * (2026-07-27 已定过:运行中一律不刷屏)。所以直接吞掉。
         * 注意与旧板不同,现在吞的原因只是"防无谓刷屏",不再是引脚共用 ——
         * 新板 START 是独立的 P20.7,菜单 MID 是 P33.4。 */
    }
    else
    {
        if(mid_edge)
            menu_handle_key_mid_press();
    }

    /* UP/DOWN 走长按重复:按住不放会自己连发,不必狂点。
     * 光标移动 100ms 一行(一秒 10 行),改值超过 1s 后自动升粗调。 */
    if(menu_key_hold_fire(key_up, &key_up_hold, &up_mul))
        menu_handle_key_up_press(up_mul);

    if(menu_key_hold_fire(key_down, &key_down_hold, &down_mul))
        menu_handle_key_down_press(down_mul);

    /* KART_LEFT 只认下降沿:它是"返回/退出",连发会一路退到主菜单。 */
    if(key_left == 0 && key_left_last == 1)
        menu_handle_key_left_press();

    /* RIGHT 有两处用途:摄像头调试页准星【列】+1(撞到右边界回卷到 0),
     * 以及轨迹页编辑态沿 x 平移一段(见下面那条 else if)。
     * 【为什么不用编码器】原设计把列交给 EC11,但那个旋钮实测一直不好用
     * (2026-08-13 确认),等于取样点根本挪不动,取色标定做不下去。
     * 二维准星必须有两个输入件,而五向只剩 RIGHT 空着 —— KART_LEFT 是返回、
     * MID 是切叠加层、UP/DOWN 已经是行。
     * 【为什么是单向回卷】双向要占两个键,没有了;单向长按 1s 出粗调 x10,
     * 走完 160 列约 1.6s,现场举着板子能接受。
     * 编码器那条路径保留不动,哪天旋钮修好了两个都能用。 */
    if(menu_key_hold_fire(key_right, &key_right_hold, &right_mul))
    {
        if(current_level == MENU_LEVEL_CAMERA)
        {
            camdbg_probe_x = (int16)(camdbg_probe_x + (int16)right_mul);
            if(camdbg_probe_x > (int16)(SCC8660_W - 1))
            {
                camdbg_probe_x = 0;
            }
            need_repaint = 1;
            repaint_row_only = 0;
        }
        else if(current_level == MENU_LEVEL_S1_TRAJ && traj_edit)
        {
            if(kart_record_adjust_segment(traj_cursor, TRAJ_EDIT_RADIUS,
                                          TRAJ_EDIT_STEP_M * (float)right_mul, 0.0f))
                traj_dirty = 1;
            kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
            need_clear = 1;
        }
    }

    /* START 只在轨迹页承担保存，覆盖科目一槽 0。未修改时不擦 Flash。 */
    if(current_level == MENU_LEVEL_S1_TRAJ
       && key_start == 0 && key_start_last == 1 && traj_dirty)
    {
        traj_edit = 0;
        if(kart_record_save_to_flash(0) == 0)
        {
            traj_dirty = 0;
            menu_flash_notice("Path Saved!", 800);
        }
        kart_traj_view_set_cursor(traj_cursor, traj_edit, traj_dirty);
        need_clear = 1;
    }

    /* 旋钮转动 = 连按 UP/DOWN。走同一套处理函数,行为与按键完全一致
     * (含编辑态改值、S1_READY 热调速度倍率)。快旋时带粗调倍率。
     *
     * 唯一例外是摄像头调试页:那里 UP/DOWN 已经占了准星的【行】,旋钮改成调【列】。
     * 不复用 up/down 处理函数是因为它们只认一个轴 —— 二维准星要靠两个输入件
     * 分工,否则调个取样点得先切轴再调,现场举着板子腾不出第三只手。 */
    if(current_level == MENU_LEVEL_CAMERA)
    {
        if(enc != 0)
        {
            int16 step = (int16)(enc_fast ? MENU_KEY_STEP_FAST : 1);

            camdbg_probe_x = (int16)(camdbg_probe_x + (int16)enc * step);
            if(camdbg_probe_x < 0) { camdbg_probe_x = 0; }
            if(camdbg_probe_x > (int16)(SCC8660_W - 1))
            {
                camdbg_probe_x = (int16)(SCC8660_W - 1);
            }
            need_repaint = 1;
            repaint_row_only = 0;
        }
    }
    else if(current_level != MENU_LEVEL_S1_TRAJ) /* 轨迹页明确不用旋钮 */
    {
        uint8 emul = enc_fast ? MENU_KEY_STEP_FAST : 1u;
        while(enc > 0) { menu_handle_key_up_press(emul);   enc--; }
        while(enc < 0) { menu_handle_key_down_press(emul); enc++; }
    }

    /* 任一输入活动都重新计时:连续调值期间不落盘,停手 2s 后才写一次 Flash。
     * enc_gap==0 表示这一拍刚转出一格,旋钮也算活动(否则拧的时候会被中途打断去擦 Flash)。 */
    if(key_mid == 0 || key_up == 0 || key_down == 0 || key_left == 0 || key_esw == 0
       || enc_gap == 0)
        params_idle_cnt = 0;

    /* UP/DOWN 不再记上一拍电平:边沿判定已交给 key_*_hold(0→1 就是按下第一拍)。 */
    key_mid_last = key_mid;
    key_left_last = key_left;
    key_start_last = key_start;
    enc_sw_last = key_esw;

    /* RIGHT(P21.7)没绑"进入":与 MID 重复,而误触"进入"会直接执行菜单项
     * (发车/擦写 Flash)。要接就接到 menu_handle_key_mid_press(记得加
     * key_right_last)。
     * 但它并非全无动作 —— 上面 menu_key_hold_fire(key_right) 已经把它接到摄像头
     * 页的准星列和轨迹页的平移,只是不做菜单导航。原注释写的"刻意不绑动作"是
     * 加那两处之前的说法。 */
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

    gpio_init(MENU_ENC_A,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(MENU_ENC_B,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(MENU_ENC_SW, GPI, 0, GPI_FLOATING_IN);

    /* 按当前真实电平初始化"上一拍"记录,防上电瞬间被当成一次按下/转动:
     * 若某键上电时正被按住,记 0 就不会产生下降沿。 */
    key_mid_last  = gpio_get_level(KART_MENU_KEY_MID);
    key_left_last = gpio_get_level(KART_MENU_KEY_LEFT);
    key_start_last = gpio_get_level(BOARD_START_KEY_PIN);
    enc_sw_last   = gpio_get_level(MENU_ENC_SW);
    enc_a_last    = gpio_get_level(MENU_ENC_A);
    enc_b_last    = gpio_get_level(MENU_ENC_B);
    enc_accum     = 0;
    enc_detent    = 0;
    /* UP/DOWN 用 hold 计数代替电平记录。上电时若某键正被按住,第一拍 hold 会从 0
     * 走到 1 而产出一次动作 —— 这里预置到 REPEAT_DELAY 之上就把它吞掉:
     * 只有真正松开再按才会有新动作。 */
    key_up_hold   = (gpio_get_level(KART_MENU_KEY_UP)   == 0) ? MENU_KEY_REPEAT_DELAY : 0;
    key_down_hold = (gpio_get_level(KART_MENU_KEY_DOWN) == 0) ? MENU_KEY_REPEAT_DELAY : 0;
    key_right_hold = (gpio_get_level(KART_MENU_KEY_RIGHT) == 0) ? MENU_KEY_REPEAT_DELAY : 0;
    enc_gap       = 0xFFFFu;
    enc_fast      = 0;

    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    kart_boot_anim_play();
    /* 【这里必须本地直画,不能走 draw_*】本函数在 cpu0_main.c 的
     * cpu_wait_event_ready() 之前跑,core2 还没进 service 循环,入队没人取,
     * 屏幕会停在开机动画最后一帧。kart_boot_anim_play() 同理,一直是本地画。 */
    ips200_set_color(UI_FG, UI_BG);
    ips200_clear();

    current_level = MENU_LEVEL_MAIN;
    cursor_main = 0;
    rec_state = REC_STATE_IDLE;

    last_drawn_level = MENU_LEVEL_MAIN;
    last_drawn_rec   = REC_STATE_IDLE;
    need_clear   = 1;
    need_repaint = 1;
    repaint_row_only = 0;
}

/* 按当前状态画整页。抽出来供清屏后重绘与换页重绘共用。 */
static void menu_draw_page(void)
{
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
            case MENU_LEVEL_MAIN:                menu_draw_main();               break;
            case MENU_LEVEL_SUBJECT1:            menu_draw_subject1();           break;
            case MENU_LEVEL_SUBJECT2:            menu_draw_subject2();           break;
            case MENU_LEVEL_S2_VOICE:            menu_draw_s2_voice();           break;
            case MENU_LEVEL_S2_GATE:             menu_draw_s2_gate();            break;
            case MENU_LEVEL_S2_GATE_SLOT_SAVE:   menu_draw_s2_gate_slot_save();  break;
            case MENU_LEVEL_S2_GATE_SLOT_LOAD:   menu_draw_s2_gate_slot_load();  break;
            case MENU_LEVEL_S2_RET:              menu_draw_s2_ret();             break;
            case MENU_LEVEL_S2_RET_SLOT_SAVE:    menu_draw_s2_ret_slot_save();   break;
            case MENU_LEVEL_S2_RET_SLOT_LOAD:    menu_draw_s2_ret_slot_load();   break;
            case MENU_LEVEL_S1_SLOT_SAVE:        menu_draw_slot_save();          break;
            case MENU_LEVEL_S1_SLOT_LOAD:        menu_draw_slot_load();          break;
            case MENU_LEVEL_S1_READY:            menu_draw_s1_ready();           break;
            case MENU_LEVEL_S1_TRAJ:             kart_traj_view_draw();          break;
            case MENU_LEVEL_S3_RUN:              menu_draw_s3_run();             break;
            case MENU_LEVEL_CAMERA:              menu_draw_camera();             break;
            case MENU_LEVEL_SETTINGS:            menu_draw_settings();           break;
            default:                                                             break;
        }
    }
}

/* 输入采样。放 10ms 拍(见 kart_menu.h):旋钮解码本来就必须 10ms,按键判定跟着搬过来,
 * 于是"按下→动作"的延迟从最坏 50ms 降到 10ms,长按重复也才有 100ms 的分辨率。
 * 这里只读 GPIO + 改菜单状态变量,不碰屏 —— 屏幕留给 kart_menu_poll 在 50ms 拍画。
 * 例外:菜单项里点 Save/Load 会阻塞擦写 DFlash 并闪提示,但那些只在静止页面点得到。 */
void kart_menu_input_poll(void)
{
    /* 先解码再判键:本拍转出的格子本拍就消费掉,不多等一拍。 */
    kart_menu_enc_poll();
    menu_scan_keys();
}

static void menu_poll_body(void)
{
    /* 语音收帧/分发已交给 subject2_loop 独占(进 Voice A/B 会切 MISSION_SUBJECT_2)。
     * 此处不再调 kart_voice_poll/dispatch,避免与 subject2_loop 双份分发抢同一队列。
     * 按键/旋钮采样已搬到 10ms 拍的 kart_menu_input_poll,这里只负责画。 */

    /* 换页自动判定:哪个处理函数改了 current_level / rec_state 都不用自己记得置清屏标志,
     * 这里比对上一次画的是哪页即可。页内改值/移光标两者都不变 → 不清屏。
     * 科目三阶段跳变刻意不算换页(2026-07-27 定):倒车过程自动推进,跟着刷屏会在
     * 控制窗口里插整屏 SPI 写。进 S3 界面的首屏由换页那次画出,之后保持静止。 */
    if(current_level != last_drawn_level || rec_state != last_drawn_rec)
    {
        need_clear = 1;
        last_drawn_level = current_level;
        last_drawn_rec   = rec_state;
    }


    /* 科目三阶段跳变的唯一重绘例外:进 S3_SIGNAL / S3_FINISHED。
     * 这两个阶段车已完整停机(倒车 kart_playback 跑完 + mission_stop_all),
     * 不存在"整屏 SPI 写挤掉控制拍"的风险;而 S3_SIGNAL 必须让现场看到
     * 可以喊口令了,否则屏幕会一直停在倒车页。其余阶段跳变仍然不刷。 */
    if(current_level == MENU_LEVEL_S3_RUN)
    {
        kart_subject3_stage_t s3 = kart_mission_get_subject3_stage();

        if(s3 != last_drawn_s3)
        {
            if(s3 == S3_SIGNAL || s3 == S3_FINISHED)
            {
                need_clear = 1;
            }
            last_drawn_s3 = s3;
        }
    }

    if(rec_state == REC_STATE_RECORDING)
    {
        if(need_clear)
        {
            ui_clear();
            menu_draw_recording_active();
            need_clear = 0;
            need_repaint = 0;
        }
        else
        {
            menu_draw_rec_count();  /* 录制中只刷点数那一行,不动整页 */
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
     * KART_LEFT 退出"这条路径 —— 否则那次改动在 reset(重置 IMU)后就没了。
     * 这里已经在 RECORDING / kart_playback_is_running 两个 return 之后,车必定静止;
     * 且只有 dirty=1 才真写,连续微调也只在停手后写一次。 */
    if(current_level == MENU_LEVEL_SETTINGS || current_level == MENU_LEVEL_S1_READY)
    {
        if(kart_params_is_dirty())
        {
            if(params_idle_cnt < MENU_PARAM_AUTOSAVE_TICKS) params_idle_cnt++;

            if(params_idle_cnt >= MENU_PARAM_AUTOSAVE_TICKS)
            {
                kart_params_save_if_dirty();
                need_repaint = 1;       /* 重画,把标脏的 '*' 抹掉 */
            }
        }
    }

    /* 摄像头调试页每拍都要重画:它是唯一"内容自己会变"的页面(要出实时视频),
     * 其他页都是按键驱动 —— 不在这里主动置标志,画面就只在按键时更新一格,
     * 那就不是预览而是单帧抓拍了。
     * 【为什么敢每拍画】这一拍(50ms)本来就为换页留了整屏 ui_clear 的余量,
     * 出图 4ms + 视觉 3ms 在同一量级;且本页强制 MISSION_IDLE,车不在动。
     * 只置 need_repaint 不置 need_clear:清屏会让视频闪,而定宽文本 + 整幅
     * 覆盖出图本来就能把上一帧盖干净。 */
    if(current_level == MENU_LEVEL_CAMERA)
    {
        need_repaint = 1;
        repaint_row_only = 0;
    }

    /* 科目三跟随实时画面(出厂用的就是它,见 menu_draw_s3_live)。理由同上:内容自己会变,不主动置标志
     * 就只在按键时更新一格,那不是预览而是单帧抓拍 —— 而这一页存在的唯一目的
     * 就是看识别对不对。
     * 【与上面那页不同,这里车是自己在动的】所以只放开跟随阶段这一个阶段:
     *   倒车阶段 kart_playback_is_running() 在上面已经 return 了,本来就轮不到;
     *   S3_SIGNAL/FINISHED 车已停机,由换页那次画出即可。
     * 只置 need_repaint 不置 need_clear:ips200_clear 整屏十几 ms,既闪又会
     * 落在车正跑的时候。代价仍在:出图约 4ms 插进 50ms 拍 —— 原打算"测完删掉
     * 本块",赛后没删,整场就靠这一页看识别对不对。 */
#if (S3_FOLLOW_SRC == S3_FOLLOW_SRC_VISION) && (CAMERA_ENABLE)
    if(current_level == MENU_LEVEL_S3_RUN
       && kart_mission_get_subject3_stage() == S3_PHASE1_FOLLOW)
    {
        need_repaint = 1;
        repaint_row_only = 0;
    }
#elif (S3_FOLLOW_SRC == S3_FOLLOW_SRC_PLINK)
    /* PLINK 页同样要每拍刷（link/bear/h 都是自己在变的量），但它只有四行
     * 定宽文本、不出图，1~2ms/拍，比视觉那页的 4ms 出图轻，不是“测完删”的临时块。
     * 不过出厂 PERSON_LINK_ENABLE=0,整块没编译 —— 跑的是 VISION 那一支。
     * 同样只放开跟随阶段：倒车阶段 kart_playback_is_running() 已在上面 return。 */
    if(current_level == MENU_LEVEL_S3_RUN
       && kart_mission_get_subject3_stage() == S3_PHASE1_FOLLOW)
    {
        need_repaint = 1;
        repaint_row_only = 0;
    }
#endif

    /* 三档开销,从大到小:
     *   换页   → ui_clear() 整屏 76800 像素(软件 SPI,十几 ms)+ 整页重画;
     *   页内   → 只重画本页各行(定宽文本盖旧字,不清屏),约 1~2ms;
     *   改值   → 只重画光标那一行,<1ms —— 长按/快旋连续调值走的就是这条。 */
    if(need_clear)
    {
        ui_clear();
        menu_draw_page();
        need_clear = 0;
        need_repaint = 0;
        repaint_row_only = 0;
    }
    else if(need_repaint)
    {
        /* row_only 只对 Settings 有意义(唯一有滚动列表 + 就地改值的页)。
         * 其他页改值时(S1_READY 的速度倍率)本来就只有两三行,整页重画也不贵。 */
        if(repaint_row_only && current_level == MENU_LEVEL_SETTINGS)
        {
            menu_draw_settings_cursor_row();
        }
        else
        {
            menu_draw_page();
        }
        need_repaint = 0;
        repaint_row_only = 0;
    }

    /* 状态栏每拍刷新(放主体之后,不会被清屏擦掉),IMU yaw 实时更新不靠按键。
     * 轨迹页的 y=18/36 两行用于点数、里程和包围盒，状态栏会与它们重叠；
     * 该页又是停车后的静态诊断页，不需要实时 Yaw/RC/SW，故单独让出这两行。 */
    if(current_level != MENU_LEVEL_S1_TRAJ)
    {
        menu_draw_status_bar();
    }
}

/* 屏幕绘制的帧闸。【为什么要包一层而不是在 body 里到处写 begin/commit】
 * body 有三条提前 return(录制中、kart_playback running、轨迹页让出状态栏),
 * 每条都得配一次 commit,漏一条 busy 就永远不释放、屏幕彻底不动。
 * 包一层让 begin/commit 在语法上必然配对。
 *
 * 【节流】core0 每 50ms 产生一帧,core2 排完一帧要 355ms(软件 SPI 61.8 万 bit),
 * 生产比消费快 7 倍。所以 core2 还在画就整帧跳过,不排队 —— 排队只会让
 * 屏上画面越来越滞后。实际预览帧率约 2.8fps,与搬核前(405ms/帧)基本相同,
 * 差别是这 355ms 不再占着 core0 的控制拍。 */
void kart_menu_poll(void)
{
    if(!draw_begin())
    {
        return;     /* core2 还在画上一帧,本拍不刷屏。控制环不受影响。 */
    }

    menu_poll_body();
    draw_commit();
}
