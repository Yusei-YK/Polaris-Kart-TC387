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
    MENU_LEVEL_S2_RET,
    MENU_LEVEL_S2_RET_SLOT_SAVE,
    MENU_LEVEL_S2_RET_SLOT_LOAD,
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

/* -------------------- Settings 页行表 --------------------
 * 改之前 Settings 是"按 kart_param_id_t 顺序铺 24 行 + 两个动作项",
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
    { KART_PARAM_PB_PROF,       NULL              },
    { KART_PARAM_PB_SCALE,      NULL              },
    { KART_PARAM_PB_VMAX,       NULL              },
    { KART_PARAM_PB_VMIN,       NULL              },
    { KART_PARAM_PB_ALAT,       NULL              },
    { KART_PARAM_PB_CLAMP,      NULL              },
    { KART_PARAM_PB_ABRAKE,     NULL              },
    { KART_PARAM_PB_LDGAIN,     NULL              },
    { KART_PARAM_PB_LDMAX,      NULL              },

    { SET_ROW_HEAD,             "-- LIMITS --"    },
    { KART_PARAM_SPD_KP,        NULL              },
    { KART_PARAM_SPD_IMAX,      NULL              },
    { KART_PARAM_STR_OUTMAX,    NULL              },
    { KART_PARAM_SLEW_REAR,     NULL              },
    { KART_PARAM_RAMP,          NULL              },

    { SET_ROW_HEAD,             "-- S4 REVERSE --"},
    { KART_PARAM_S4_OL_SPD,     NULL              },
    { KART_PARAM_S4_OL_MODE,    NULL              },
    { KART_PARAM_S4_OL_KH,      NULL              },
    { KART_PARAM_S4_OL_KE,      NULL              },
    { KART_PARAM_PB_REVSCL,     NULL              },
    { KART_PARAM_PB_REVCORR,    NULL              },

    { SET_ROW_HEAD,             "-- MISC --"      },
    { KART_PARAM_RC_VMAX,       NULL              },
    { KART_PARAM_HEAD_KP,       NULL              },
    { KART_PARAM_S1_REV_SPD,    NULL              },
    { KART_PARAM_S1_REV_STOP,   NULL              },

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

static uint8 key_mid_last = 1;
static uint8 enc_sw_last = 1;       /* 旋钮按下键上一拍电平(与 MID 等价) */
static uint8 enc_a_last = 1;        /* 旋钮 A/B 相上一拍电平 */
static uint8 enc_b_last = 1;
static int8  enc_accum = 0;         /* 未攒满一格的边沿余数 */
static int8  enc_detent = 0;        /* 解码产出、按键扫描取走的待处理格数 */
static uint8 key_left_last = 1;

/* UP/DOWN 按住时长(10ms 拍)。0=没按住。长按自动重复+粗调靠它。 */
static uint16 key_up_hold = 0;
static uint16 key_down_hold = 0;
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

static void menu_draw_settings(void);
static void menu_draw_settings_row(uint8 row, uint16 y);

/* ==================== VSCode Dark+ 配色 ====================
 * 【为什么换配色不花时间】ips200_show_char 画每个字时,笔画像素写 pencolor、
 * 其余像素写 bgcolor —— 一个字格 8x16=128 个像素本来就都要写一遍。所以
 * "换颜色"和"给整行铺高亮底"都是改写入的数值,SPI 字节数一个不多。
 * 这块屏是软件 SPI(IPS200_USE_SOFT_SPI=1,P02.8/P20.3 翻脚),真正贵的是
 * ips200_clear() 整屏 76800 像素 —— 那个已经改成只在换页时做了。
 *
 * 【硬约束:定宽】不清屏重绘,靠的是新文本把旧文本每个字格盖掉。所以所有
 * 行输出都走 ui_bar/ui_item 补空格到 UI_COLS,不要直接 ips200_show_string
 * 写不定长串,否则会看到上一帧的尾巴。 */
#define UI_BG        (0x18E3)   /* #1E1E1E 编辑器背景 */
#define UI_FG        (0xD6BA)   /* #D4D4D4 正文 */
#define UI_SEL_BG    (0x226F)   /* #264F78 选中行底 */
#define UI_SEL_FG    (0xFFFF)   /* #FFFFFF 选中行字 */
#define UI_BAR_BG    (0x03D9)   /* #007ACC 标题栏蓝 */
#define UI_BAR_FG    (0xFFFF)
#define UI_KEY       (0x54FA)   /* #569CD6 关键字蓝 */
#define UI_NUM       (0xCC8F)   /* #CE9178 数值橙 */
#define UI_OK        (0x6CCA)   /* #6A9955 正常绿 */
#define UI_WARN      (0xDEF5)   /* #DCDCAA 提示黄 */
#define UI_ERR       (0xF228)   /* #F44747 故障红 */
#define UI_DIM       (0x8430)   /* #858585 次要灰 */

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
    ips200_set_color(UI_FG, UI_BG);
    ips200_clear();
}

/* 整行定宽输出:补空格到 30 字铺满一行,右侧残留一并盖掉。 */
static void ui_bar(uint16 y, const char *s, uint16 fg, uint16 bg)
{
    char line[UI_COLS + 1];
    uint8 i = 0;

    while(i < UI_COLS && s[i] != '\0') { line[i] = s[i]; i++; }
    while(i < UI_COLS)                 { line[i] = ' ';  i++; }
    line[UI_COLS] = '\0';

    ips200_set_color(fg, bg);
    ips200_show_string(0, y, line);
}

/* 局部着色文本(状态行那种字段)。调用方自己保证定宽。 */
static void ui_text(uint16 x, uint16 y, const char *s, uint16 fg, uint16 bg)
{
    ips200_set_color(fg, bg);
    ips200_show_string(x, y, s);
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

    ips200_set_color(UI_BAR_FG, UI_BAR_BG);
    ips200_show_string(0, UI_Y_TITLE, line);
}

/* 底部按键提示。这里写的键名必须跟真实按键一致:
 * knob=转旋钮, MID=五向中键或按下旋钮(两者等价), LEFT=五向左,
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
    ui_title("KART TC387", NULL);
    ui_bar(UI_Y_HEAD, " Select subject", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Subject 1   Slalom",
            (uint8)(cursor_main == MENU_MAIN_SUBJECT1));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Subject 2   Voice",
            (uint8)(cursor_main == MENU_MAIN_SUBJECT2));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "Subject 4   Maze",
            (uint8)(cursor_main == MENU_MAIN_SUBJECT4));
    ui_item(UI_Y_ROW0 + 3 * UI_ROW_H, "Settings",
            (uint8)(cursor_main == MENU_MAIN_SETTINGS));

    ui_hint(" knob/UP/DN move  MID enter");
}

/* 参数一行:"光标 名字 值"。值按 meta.decimals 决定小数位,整数量(Vmax/Kp)不显示 .0。
 * 选中且在编辑态时光标变 '*',提示此刻 knob/UP/DN 改的是值不是光标。
 *
 * 名字用 %-11s 补到定宽:滚屏会让同一行换成另一个参数,名字短一截就会留下
 * 上一个名字的尾巴("Ramp Step"→"PB Prof" 剩个 'e')。 */
#define MENU_SET_ROW_Y(row)   ((uint16)(UI_Y_ROW0 + (row) * UI_ROW_H))

/* -------------------- 参数此刻是否真的起作用 --------------------
 * 返回 0 = 这一项现在调了也没用,菜单画成灰字。
 * 起因:菜单里有一批"看着能调、实际不进任何计算"的项,现场照着调会白费一趟。
 * 逐条都是查过代码的,改代码时记得同步这里:
 *
 *  S1 RevSpd / S1 RevStop —— kart_mission.h 的 KART_S1_AUTO_REVERSE=0。
 *      读它们的 S1_GARAGE_APPROACH/S1_REVERSE_IN 分支还在、也照样编译,但唯一
 *      切进去的那行在 #if 里(kart_mission.c:200),没路径走得到 → 恒灰。
 *      倒库现在是录在轨迹里跟着 playback 一起复现的。
 *  PB Scale/Vmax/Vmin/Alat/ABrake/RevScl —— 只在 kart_playback_build_profile() 里读,
 *      而该函数在 PB Prof<0.5 时开头就 return(kart_playback.c:97)→ 剖面关着时全灰。
 *      注意 Vmax 也在里面(kart_playback.c:89):剖面关着时限速只剩 PB Clamp 一道。
 *  S4 OL Ke —— 只在 poll_closedloop() 里读,出厂 S4 OLMode=0 走 openloop → 灰。
 *  PB Clamp —— 它是下发前最后一道闸,PB Vmax 是剖面内的天花板。
 *      Clamp ≥ Vmax 时永远轮不到它裁到东西(录制速度回放仍走它,故只在
 *      剖面开着时才判灰)。
 *  PB LdMax —— 前视距离上限。它比"基础前视+增益×速度"能达到的最大值还大时,
 *      同样永远裁不到东西。 */
static uint8 menu_param_is_active(uint8 id)
{
    switch(id)
    {
        case KART_PARAM_S1_REV_SPD:
        case KART_PARAM_S1_REV_STOP:
            return 0;                       /* KART_S1_AUTO_REVERSE=0,那两个阶段进不去 */

        case KART_PARAM_PB_SCALE:
        case KART_PARAM_PB_VMAX:
        case KART_PARAM_PB_VMIN:
        case KART_PARAM_PB_ALAT:
        case KART_PARAM_PB_ABRAKE:
        case KART_PARAM_PB_REVSCL:
            return (uint8)(kart_params_get(KART_PARAM_PB_PROF) > 0.5f);

        case KART_PARAM_S4_OL_KE:
            return (uint8)(kart_params_get(KART_PARAM_S4_OL_MODE) > 0.5f);

        case KART_PARAM_PB_CLAMP:
            /* 剖面关着时录制速度回放照样过这道闸 → 有效。开着时才比大小。 */
            if(kart_params_get(KART_PARAM_PB_PROF) <= 0.5f) return 1;
            return (uint8)(kart_params_get(KART_PARAM_PB_CLAMP)
                           < kart_params_get(KART_PARAM_PB_VMAX));

        case KART_PARAM_PB_LDMAX:
            /* 基础前视 + 增益×钳位速度 = 实际能达到的最大前视。够不到 LdMax 就裁不到。 */
            return (uint8)(kart_params_get(KART_PARAM_PB_LDMAX)
                           < KART_PLAYBACK_LD_BASE
                             + kart_params_get(KART_PARAM_PB_LDGAIN)
                               * kart_params_get(KART_PARAM_PB_CLAMP));

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
        case KART_PARAM_PB_PROF:        /* 管 Scale/Vmax/Vmin/Alat/ABrake/RevScl/Clamp */
        case KART_PARAM_S4_OL_MODE:     /* 管 S4 OL Ke */
        case KART_PARAM_PB_VMAX:        /* 参与 PB Clamp 的判据 */
        case KART_PARAM_PB_CLAMP:       /* 参与 PB LdMax 的判据 */
        case KART_PARAM_PB_LDGAIN:      /* 参与 PB LdMax 的判据 */
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
    float v = kart_params_get(id);
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

/* Settings 页:滚屏列表。MID 切"移光标/改值",LEFT 退出(编辑态先退编辑)。
 * 改完立即生效(存 RAM),停手 2s 或按 LEFT 自动落盘,Save to Flash 是手动兜底。 */
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
                     : " MID edit  LEFT back");
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

/* 光标那一行对应的参数 id;标题/动作行返回 KART_PARAM_MAX(表示"不是参数")。
 * 编辑态改值前用它翻译:分组之后 cursor_set 是行号,不再等于参数 id。 */
static uint8 menu_settings_param_id(void)
{
    uint8 id;

    if(cursor_set >= MENU_SET_TOTAL) return KART_PARAM_MAX;

    id = set_rows[cursor_set].id;
    return (id < KART_PARAM_MAX) ? id : KART_PARAM_MAX;
}

/* 科目四运行界面:菜单不吃 MID(防运行中刷屏),只留 LEFT 退出。
 * 到停车区【把车停住即自动】停录并开环倒车原路返回,车头不掉转、不搬车。
 * 按 mission 当前阶段显示,阶段跳变时由 kart_menu_poll 触发重绘。 */
static void menu_draw_s4_run(void)
{
    /* 这一页的 START 是独立的物理发车键(BOARD_START_KEY_PIN / P20.7),
     * 由 kart_mission.c 检下降沿 —— 不是菜单 MID,文字别改。 */
    switch(kart_mission_get_subject4_stage())
    {
        case S4_PHASE1_RECORD:
            /* 文案以自动判停为主:kart_mission.c subject4_loop 里,走够 1m 之后
             * 车速连续 150ms ≈0 就自己停录+开倒车,START 只是"不想等那 150ms"的
             * 手动提前触发。原来写成 "START to go back",现场会以为必须按键。 */
            ui_title("Subject 4", "REC");
            ui_bar(UI_Y_HEAD,        " Recording",         UI_ERR,  UI_BG);
            ui_bar(UI_Y_ROW0,        " RC drive the maze", UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, " Stop car: auto reverse",  UI_WARN, UI_BG);
            ui_hint(" auto  START now  LEFT exit");
            break;
        case S4_PHASE2_REVERSE:
            /* 原文 "Openloop, no turn" 已过期:倒车段会跟着录制打角走,
             * 且带航向 P 纠偏(KART_PLAYBACK_OL_HEAD_EN=1),S4 OLMode=1 时
             * 再加横向位置 P。"不转向"是最早那版纯开环留下的说法。 */
            ui_title("Subject 4", "BACK");
            ui_bar(UI_Y_HEAD,        " Reversing...",      UI_WARN, UI_BG);
            ui_bar(UI_Y_ROW0,        " Retrace + heading corr", UI_FG, UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, "",                   UI_FG,   UI_BG);
            ui_hint("");
            break;
        case S4_FINISHED:
            ui_title("Subject 4", "DONE");
            ui_bar(UI_Y_HEAD,        " Finished",          UI_OK,   UI_BG);
            ui_bar(UI_Y_ROW0,        " Back at start",     UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, "",                   UI_FG,   UI_BG);
            ui_hint(" LEFT exit");
            break;
        case S4_FAULT:
        default:
            ui_title("Subject 4", "FAULT");
            ui_bar(UI_Y_HEAD,        " FAULT",             UI_ERR,  UI_BG);
            ui_bar(UI_Y_ROW0,        " Path invalid",      UI_FG,   UI_BG);
            ui_bar(UI_Y_ROW0 + UI_ROW_H, "",                   UI_FG,   UI_BG);
            ui_hint(" LEFT exit");
            break;
    }
}

static void menu_draw_subject1(void)
{
    ui_title("Subject 1", "Slalom");
    ui_bar(UI_Y_HEAD, " Record / playback path", UI_KEY, UI_BG);

    ui_item(UI_Y_ROW0 + 0 * UI_ROW_H, "Record Path",
            (uint8)(cursor_s1 == MENU_S1_RECORD));
    ui_item(UI_Y_ROW0 + 1 * UI_ROW_H, "Playback Path",
            (uint8)(cursor_s1 == MENU_S1_PLAYBACK));
    ui_item(UI_Y_ROW0 + 2 * UI_ROW_H, "Enter Remote",
            (uint8)(cursor_s1 == MENU_S1_ENTER_REMOTE));
    ui_item(UI_Y_ROW0 + 3 * UI_ROW_H, "Exit Remote",
            (uint8)(cursor_s1 == MENU_S1_EXIT_REMOTE));

    ui_hint(" MID enter  LEFT back");
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

    ui_hint(" MID enter  LEFT back");
}

static void menu_draw_s2_voice(void)
{
    uint8 man = kart_mission_subject2_get_manual_return();

    ui_title(man ? "Voice B" : "Voice A", "LIVE");
    ui_bar(UI_Y_HEAD, " Speak command", UI_OK, UI_BG);
    ui_bar(UI_Y_ROW0, " Listening...",  UI_FG, UI_BG);
    ui_bar(UI_Y_ROW0 + UI_ROW_H,
           man ? " Return: RC + voice" : " Return: auto GOTO",
           man ? UI_WARN : UI_KEY, UI_BG);
    ui_hint(" MID exit  LEFT exit");
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

    ui_hint(" MID enter  LEFT back");
}

/* 录制等待/录制中:开录和停录走的是 menu_scan_keys 里的 mid_edge —— 五向 MID
 * 或按下旋钮,跟独立 START 键无关。所以提示必须写 MID,不能写 START。 */
static void menu_draw_recording_wait(void)
{
    ui_title("Record", "READY");
    ui_bar(UI_Y_HEAD,      " Ready to record",     UI_OK,   UI_BG);
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

    ui_hint(" MID confirm  LEFT discard");
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

    ui_hint(" MID load  LEFT back");
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

    ui_hint(" MID confirm  LEFT discard");
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

    ui_hint(" MID load  LEFT back");
}

static const char* menu_ret_slot_name(uint8 i)
{
    static const char* names[KART_MENU_S2_RET_SLOT_NUM] =
        { "Ret1 Right", "Ret1", "Ret2", "Ret3", "Ret3 Left" };
    return (i < KART_MENU_S2_RET_SLOT_NUM) ? names[i] : "?";
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

    ui_hint(" MID enter  LEFT back");
}

static void menu_draw_s2_ret_slot_save(void)
{
    uint8 i;

    ui_title("Save Return Path", "S2");
    ui_bar(UI_Y_HEAD, " Pick a return slot", UI_KEY, UI_BG);

    for(i = 0; i < KART_MENU_S2_RET_SLOT_NUM; i++)
    {
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), menu_ret_slot_name(i),
                           kart_flash_slot_count((uint8)(i + KART_FLASH_S2R_FIRST_SLOT)),
                           (uint8)(cursor_slot == i));
    }

    ui_item((uint16)(UI_Y_ROW0 + KART_MENU_S2_RET_SLOT_NUM * UI_ROW_H), "Don't Save",
            (uint8)(cursor_slot == KART_MENU_S2_RET_SLOT_NUM));

    ui_hint(" MID confirm  LEFT discard");
}

static void menu_draw_s2_ret_slot_load(void)
{
    uint8 i;

    ui_title("Load Return Path", "S2");
    ui_bar(UI_Y_HEAD, " Pick a return slot", UI_KEY, UI_BG);

    for(i = 0; i < KART_MENU_S2_RET_SLOT_NUM; i++)
    {
        menu_draw_slot_row((uint16)(UI_Y_ROW0 + i * UI_ROW_H), menu_ret_slot_name(i),
                           kart_flash_slot_count((uint8)(i + KART_FLASH_S2R_FIRST_SLOT)),
                           (uint8)(cursor_slot == i));
    }

    ui_hint(" MID run  LEFT back");
}

static void menu_draw_s1_ready(void)
{
    char buf[40];
    uint8 prof_on = (uint8)(kart_params_get(KART_PARAM_PB_PROF) > 0.5f);

    /* 标题右角标 * = 改了还没进 Flash;按 LEFT 退出这一页会自动存(星号随之消失)。 */
    ui_title("Ready", kart_params_is_dirty() ? "*" : " ");
    ui_bar(UI_Y_HEAD, " Path loaded, at start pt", UI_OK, UI_BG);

    /* 就地显示复现胆量,knob/UP/DN 直接改。改完按 START 才生效(剖面在 start 时重算)。
     * Prof=OFF 时倍率不起作用(速度仍取录制速度),所以把开关状态一并显示出来,
     * 免得在场上狂拧旋钮却毫无变化。 */
    sprintf(buf, " Speed  x%-5.2f", kart_params_get(KART_PARAM_PB_SCALE));
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
    ui_hint(" knob speed  START go  LEFT back");
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
    /* 清 row_only:MID/LEFT 会改标题栏、副标题、底部提示(进出编辑态等),
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
                rec_target = 0;
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
            if(cursor_s2 == MENU_S2_VOICE || cursor_s2 == MENU_S2_VOICE_B)
            {
                kart_mission_subject2_set_manual_return(
                    (uint8)(cursor_s2 == MENU_S2_VOICE_B));
                /* 真正进科目二状态机:subject2_loop 才会每拍跑 voice_dispatch+motion_update,
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
                 * 车在跑(playback_is_running)时 kart_menu_poll 早就 return 了,
                 * 按键根本进不到这里,不会在运行中插入 Flash 擦写。 */
                kart_params_save();
                menu_flash_notice("Params Saved!", 800);
            }
            else if(row_id == SET_ROW_DEF)
            {
                kart_params_load_default();
                menu_flash_notice("Default Loaded", 800);
            }
            else if(row_id < KART_PARAM_MAX)
            {
                set_edit = set_edit ? 0 : 1;     /* MID 切换 移光标/改值 */
            }
            break;
        }

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
            if(cursor_slot < KART_MENU_S2_RET_SLOT_NUM)
            {
                uint8 ret = kart_record_save_to_flash(
                                (uint8)(cursor_slot + KART_FLASH_S2R_FIRST_SLOT));
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
            if(cursor_slot < KART_MENU_S2_RET_SLOT_NUM)
            {
                if(kart_flash_slot_count((uint8)(cursor_slot + KART_FLASH_S2R_FIRST_SLOT)) > 0)
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
                uint8 pid = menu_settings_param_id();
                if(pid < KART_PARAM_MAX)
                {
                    kart_params_step_mul(pid, +1, mul);
                    /* 改到"管别人灰不灰"的项就得整页重画,否则别的行灰着不变。 */
                    repaint_row_only = (uint8)(!menu_param_gates_others(pid));
                }
            }
            else menu_settings_seek(-1);        /* 上移一格,跳过分组标题 */
            break;

        /* 科目一就绪界面:UP/DOWN 热调复现速度倍率(PB Scale)。
         * 场地上试速度不用退菜单:UP 加胆量、DOWN 减胆量,再按 START 跑一趟。
         * 只改 RAM 值,满意了再进 Settings 存 Flash。剖面在 playback_start 时重算,
         * 故改完必须重跑才生效(跑动中不会中途变速)。
         * 科目四界面不挂这个:它走开环倒车固定速,不用剖面,且流程已自动推进,
         * 不在这里插任何按键行为。 */
        case MENU_LEVEL_S1_READY:
            kart_params_step_mul(KART_PARAM_PB_SCALE, +1, mul);
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
            if(cursor_slot < KART_MENU_S2_RET_SLOT_NUM) cursor_slot++;
            break;

        case MENU_LEVEL_S2_RET_SLOT_LOAD:
            if(cursor_slot < KART_MENU_S2_RET_SLOT_NUM - 1) cursor_slot++;
            break;

        case MENU_LEVEL_SETTINGS:
            if(set_edit)
            {
                uint8 pid = menu_settings_param_id();        /* 见 UP 处注释 */
                if(pid < KART_PARAM_MAX)
                {
                    kart_params_step_mul(pid, -1, mul);
                    repaint_row_only = (uint8)(!menu_param_gates_others(pid));
                }
            }
            else menu_settings_seek(+1);        /* 下移一格,跳过分组标题 */
            break;

        case MENU_LEVEL_S1_READY:
            kart_params_step_mul(KART_PARAM_PB_SCALE, -1, mul);
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
    uint8 key_esw = gpio_get_level(KART_MENU_ENC_SW);
    uint8 mid_edge;
    uint8 up_mul, down_mul;
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
    else if(current_level == MENU_LEVEL_S1_READY || current_level == MENU_LEVEL_S4_RUN)
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

    /* LEFT 只认下降沿:它是"返回/退出",连发会一路退到主菜单。 */
    if(key_left == 0 && key_left_last == 1)
        menu_handle_key_left_press();

    /* 旋钮转动 = 连按 UP/DOWN。走同一套处理函数,行为与按键完全一致
     * (含编辑态改值、S1_READY 热调速度倍率)。快旋时带粗调倍率。 */
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
    key_mid_last  = gpio_get_level(KART_MENU_KEY_MID);
    key_left_last = gpio_get_level(KART_MENU_KEY_LEFT);
    enc_sw_last   = gpio_get_level(KART_MENU_ENC_SW);
    enc_a_last    = gpio_get_level(KART_MENU_ENC_A);
    enc_b_last    = gpio_get_level(KART_MENU_ENC_B);
    enc_accum     = 0;
    enc_detent    = 0;
    /* UP/DOWN 用 hold 计数代替电平记录。上电时若某键正被按住,第一拍 hold 会从 0
     * 走到 1 而产出一次动作 —— 这里预置到 REPEAT_DELAY 之上就把它吞掉:
     * 只有真正松开再按才会有新动作。 */
    key_up_hold   = (gpio_get_level(KART_MENU_KEY_UP)   == 0) ? MENU_KEY_REPEAT_DELAY : 0;
    key_down_hold = (gpio_get_level(KART_MENU_KEY_DOWN) == 0) ? MENU_KEY_REPEAT_DELAY : 0;
    enc_gap       = 0xFFFFu;
    enc_fast      = 0;

    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ui_clear();

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
            case MENU_LEVEL_S4_RUN:              menu_draw_s4_run();             break;
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

void kart_menu_poll(void)
{
    /* 语音收帧/分发已交给 subject2_loop 独占(进 Voice Control 会切 MISSION_SUBJECT_2)。
     * 此处不再调 voice_poll/dispatch,避免与 subject2_loop 双份分发抢同一队列。
     * 按键/旋钮采样已搬到 10ms 拍的 kart_menu_input_poll,这里只负责画。 */

    /* 换页自动判定:哪个处理函数改了 current_level / rec_state 都不用自己记得置清屏标志,
     * 这里比对上一次画的是哪页即可。页内改值/移光标两者都不变 → 不清屏。
     * 科目四阶段跳变刻意不算换页(2026-07-27 定):倒车过程自动推进,跟着刷屏会在
     * 控制窗口里插整屏 SPI 写。进 S4 界面的首屏由换页那次画出,之后保持静止。 */
    if(current_level != last_drawn_level || rec_state != last_drawn_rec)
    {
        need_clear = 1;
        last_drawn_level = current_level;
        last_drawn_rec   = rec_state;
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
                need_repaint = 1;       /* 重画,把标脏的 '*' 抹掉 */
            }
        }
    }

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

    /* 状态栏每拍刷新(放主体之后,不会被清屏擦掉),IMU yaw 实时更新不靠按键。 */
    menu_draw_status_bar();
}
