#include "kart_light.h"

#include <string.h>

typedef struct
{
    uint16 row[KART_LIGHT_ROW_NUM];    /* 7 行点阵数据，每行只使用低 15 位。 */
} kart_light_frame_t;

typedef struct
{
    kart_light_command_t command;      /* 当前正在显示的灯光命令。 */
    volatile uint16 brightness;        /* 主循环写、扫描中断读，必须保持可见性。 */
    uint16 phase_elapsed_ms;           /* 当前动画帧已经持续的时间。 */
    uint8 phase;                       /* 当前动画相位序号。 */
} kart_light_state_t;

typedef struct
{
    char ch;                           /* 字模对应的 ASCII 字符。 */
    uint8 row[KART_LIGHT_ROW_NUM];     /* 字符的 5x7 点阵数据。 */
} kart_light_glyph_t;

/*
 * 每个字符宽 5 像素，三个字符正好铺满 15 列。
 * 精简字库只保留当前比赛灯板会使用的字符，后续图案可以继续扩展。
 */
static const kart_light_glyph_t kart_light_font[] =
{
    { ' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
    { '!', {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04} },
    /* 2026-07-26 修正 '<' '>' '/' '\\' 四个符号的左右方向(实车观察:左转显右箭头)。
     * 本字库的列位约定是 bit4=最左列、bit0=最右列 —— 由字母坐实:
     *   'L'={0x10..,0x1F}:0x10=bit4 竖笔在最左,底横在下,正确;
     *   'F'={0x1F,0x10,0x10,0x1E,..}:同理竖笔在左;
     *   'C'/'B' 的开口与竖笔方向也只有在 bit4=左 时才对。
     * 但原来这四个符号是按 bit0=最左 画的,于是只有它们是镜像的:
     *   旧 '<'={0x08,0x04,0x02,0x01,..} 尖端落在 bit0=最右列 → 实际显示成右箭头。
     * 修法:把 '<'/'>' 两组数据互换,'/'/'\\' 两组数据互换。
     * 注意:不要改 cpu0_main.c 的 kart_light_bits_to_dot() 去"抵消"这个错 ——
     * 那会把本来正确的字母一起弄反(实车已确认 HIG/LOW/FOG/CAB 显示正常)。 */
    { '/', {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10} },
    {'\\', {0x10, 0x08, 0x08, 0x04, 0x02, 0x02, 0x01} },
    { '|', {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04} },
    { '<', {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02} },
    { '>', {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08} },
    { 'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11} },
    { 'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E} },
    { 'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E} },
    { 'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10} },
    { 'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E} },
    { 'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11} },
    { 'I', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F} },
    { 'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F} },
    { 'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E} },
    { 'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10} },
    { 'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A} }
};

static kart_light_frame_t kart_light_frames[2];
static volatile uint8 kart_light_active_frame = 0;  /* 当前可供扫描驱动读取的帧。 */
static kart_light_state_t kart_light_state;          /* 灯板逻辑的运行状态。 */

/*
 * 在精简字库中查找指定 ASCII 字符并返回它的 7 行字模数据。
 * 找不到字符时使用空格图案。
 */
static const uint8 *kart_light_find_glyph(char ch)
{
    uint8 i;

    for(i = 0; i < (uint8)(sizeof(kart_light_font) / sizeof(kart_light_font[0])); i++)
    {
        if(kart_light_font[i].ch == ch)
        {
            return kart_light_font[i].row;
        }
    }

    return kart_light_font[0].row;
}

/*
 * 把三个 5x7 字符拼成一帧 7x15 点阵，并通过双缓冲的备用帧发布。
 * text[0] 放在最左侧，text[2] 放在最右侧。
 */
static void kart_light_publish_text(const char text[3])
{
    uint8 row;
    uint8 next = (uint8)(kart_light_active_frame ^ 1U);
    const uint8 *left = kart_light_find_glyph(text[0]);
    const uint8 *middle = kart_light_find_glyph(text[1]);
    const uint8 *right = kart_light_find_glyph(text[2]);

    for(row = 0; row < KART_LIGHT_ROW_NUM; row++)
    {
        kart_light_frames[next].row[row] = (uint16)(((uint16)left[row] << 10) |
                                                    ((uint16)middle[row] << 5) |
                                                    (uint16)right[row]);
    }

    /* 备用帧写完整后再切换，避免扫描驱动读到只更新了一半的画面。 */
    kart_light_active_frame = next;
}

/*
 * 返回指定命令每个动画相位的持续时间。
 * 静态画面不需要周期更新，因此返回 0。
 */
static uint16 kart_light_phase_period(kart_light_command_t command)
{
    switch(command)
    {
        case KART_LIGHT_CMD_LEFT_TURN:
        case KART_LIGHT_CMD_RIGHT_TURN:
            return KART_LIGHT_TURN_STEP_MS;

        case KART_LIGHT_CMD_HAZARD:
            return KART_LIGHT_HAZARD_STEP_MS;

        case KART_LIGHT_CMD_WIPER:
            return KART_LIGHT_WIPER_STEP_MS;

        default:
            return 0;
    }
}

/*
 * 返回指定命令包含的动画相位数量。
 */
static uint8 kart_light_phase_count(kart_light_command_t command)
{
    switch(command)
    {
        case KART_LIGHT_CMD_LEFT_TURN:
        case KART_LIGHT_CMD_RIGHT_TURN:
        case KART_LIGHT_CMD_WIPER:
            return 4;

        case KART_LIGHT_CMD_HAZARD:
            return 2;

        default:
            return 1;
    }
}

/*
 * 根据当前命令和动画相位生成一帧新的 7x15 点阵。
 * 本函数只生成逻辑画面，不会直接向 TLD7002 发送数据。
 */
static void kart_light_render(void)
{
    static const char blank[3] = {' ', ' ', ' '};
    static const char left_animation[4][3] =
    {
        {' ', ' ', '<'},
        {' ', '<', '<'},
        {'<', '<', '<'},
        {' ', ' ', ' '}
    };
    static const char right_animation[4][3] =
    {
        {'>', ' ', ' '},
        {'>', '>', ' '},
        {'>', '>', '>'},
        {' ', ' ', ' '}
    };
    static const char wiper_animation[4][3] =
    {
        {'/', '/', '/'},
        {'|', '|', '|'},
        {'\\', '\\', '\\'},
        {'|', '|', '|'}
    };

    switch(kart_light_state.command)
    {
        case KART_LIGHT_CMD_LEFT_TURN:
            kart_light_publish_text(left_animation[kart_light_state.phase]);
            break;

        case KART_LIGHT_CMD_RIGHT_TURN:
            kart_light_publish_text(right_animation[kart_light_state.phase]);
            break;

        case KART_LIGHT_CMD_HIGH_BEAM:
            kart_light_publish_text("HIG");
            break;

        case KART_LIGHT_CMD_LOW_BEAM:
            kart_light_publish_text("LOW");
            break;

        case KART_LIGHT_CMD_FOG:
            kart_light_publish_text("FOG");
            break;

        case KART_LIGHT_CMD_HAZARD:
            kart_light_publish_text((kart_light_state.phase == 0U) ? "!!!" : blank);
            break;

        case KART_LIGHT_CMD_CABIN:
            kart_light_publish_text("CAB");
            break;

        case KART_LIGHT_CMD_WIPER:
            kart_light_publish_text(wiper_animation[kart_light_state.phase]);
            break;

        case KART_LIGHT_CMD_OFF:
        default:
            kart_light_publish_text(blank);
            break;
    }
}

/* 初始化灯板逻辑，默认关闭显示并使用默认亮度。 */
void kart_light_init(void)
{
    memset(kart_light_frames, 0, sizeof(kart_light_frames));
    kart_light_active_frame = 0;
    kart_light_state.command = KART_LIGHT_CMD_OFF;
    kart_light_state.brightness = KART_LIGHT_DEFAULT_BRIGHTNESS;
    kart_light_state.phase_elapsed_ms = 0;
    kart_light_state.phase = 0;
    kart_light_render();
}

/* 切换灯光命令；命令发生变化时立即从动画第 0 帧重新开始。 */
void kart_light_set_command(kart_light_command_t command)
{
    /* 转成无符号数后，负数枚举和值过大的枚举都会落在合法范围之外。 */
    if((uint32)command >= (uint32)KART_LIGHT_CMD_COUNT)
    {
        command = KART_LIGHT_CMD_OFF;
    }

    if(kart_light_state.command != command)
    {
        kart_light_state.command = command;
        kart_light_state.phase_elapsed_ms = 0;
        kart_light_state.phase = 0;
        kart_light_render();
    }
}

/* 获取当前正在显示的灯光命令。 */
kart_light_command_t kart_light_get_command(void)
{
    return kart_light_state.command;
}

/* 设置灯板亮度，并把输入限制在 TLD7002 使用的 0~10000 范围。 */
void kart_light_set_brightness(uint16 brightness)
{
    if(brightness > KART_LIGHT_MAX_BRIGHTNESS)
    {
        brightness = KART_LIGHT_MAX_BRIGHTNESS;
    }

    kart_light_state.brightness = brightness;
}

/* 获取当前灯板逻辑亮度。 */
uint16 kart_light_get_brightness(void)
{
    return kart_light_state.brightness;
}

/*
 * 使用调用方提供的实际时间推进动画。
 * 静态图案的周期为 0，因此不会反复生成完全相同的画面。
 */
void kart_light_update(uint16 elapsed_ms)
{
    uint16 period = kart_light_phase_period(kart_light_state.command);
    uint8 phase_count;
    uint32 total_elapsed_ms;
    uint32 elapsed_phase_count;

    if(period == 0U)
    {
        return;
    }

    /* 使用 32 位临时值，避免较大的 elapsed_ms 与已有余量相加时发生溢出。 */
    total_elapsed_ms = (uint32)kart_light_state.phase_elapsed_ms + (uint32)elapsed_ms;
    if(total_elapsed_ms < (uint32)period)
    {
        kart_light_state.phase_elapsed_ms = (uint16)total_elapsed_ms;
        return;
    }

    /*
     * 直接计算跨过的相位数，只渲染最终画面。
     * 即使主循环偶发卡顿，也不会通过循环逐帧补算而长时间阻塞。
     */
    elapsed_phase_count = total_elapsed_ms / (uint32)period;
    kart_light_state.phase_elapsed_ms = (uint16)(total_elapsed_ms % (uint32)period);
    phase_count = kart_light_phase_count(kart_light_state.command);
    kart_light_state.phase = (uint8)(((uint32)kart_light_state.phase +
                                      (elapsed_phase_count % (uint32)phase_count)) %
                                     (uint32)phase_count);
    kart_light_render();
}

/* 读取当前画面的指定一行，供底层逐行扫描使用。 */
uint16 kart_light_get_row(uint8 row)
{
    uint8 active;

    if(row >= KART_LIGHT_ROW_NUM)
    {
        return 0;
    }

    active = kart_light_active_frame;
    return kart_light_frames[active].row[row];
}

/* 复制当前完整画面，避免一轮底层扫描途中切换动画帧。 */
void kart_light_copy_frame(uint16 out_rows[KART_LIGHT_ROW_NUM])
{
    uint8 row;
    uint8 active;

    if(out_rows == NULL)
    {
        return;
    }

    active = kart_light_active_frame;
    for(row = 0; row < KART_LIGHT_ROW_NUM; row++)
    {
        out_rows[row] = kart_light_frames[active].row[row];
    }
}
