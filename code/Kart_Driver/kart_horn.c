#include "kart_horn.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"

/*
 * 鸣笛执行层实现 —— 独立中断驱动节拍机+GPIO翻转
 * ------------------------------------------------------------------
 * 使用 CCU60_CH1 独立1ms中断运行节拍机和GPIO翻转。
 * 节拍表:每条命令一串拍数,偶数索引=响、奇数索引=停,以 0 结尾。
 * 响步:根据目标频率按周期翻转GPIO。停步:GPIO保持低电平。
 * 警报模式:每步切换翻转频率产生双频效果。写进去的是 300 与 900
 * (kart_horn_start 与本文件 ISR 里那两处 900 : 300),实际方波约 167Hz 与
 * 500Hz —— 为什么对不上,头文件顶部算过。
 * ------------------------------------------------------------------
 */

static const uint16 *horn_pattern = NULL;
static uint8         horn_step     = 0;
static uint16        horn_step_ticks = 0;
static uint8         horn_is_alarm = 0;

static uint16 horn_freq_target = 0;
static uint16 horn_freq_counter = 0;
static uint16 horn_freq_period = 0;

/* 设翻转周期。period = 1000/freq 拍,horn_gpio_tick() 每 period 拍翻转一次;
 * 翻转两次才是一个方波周期,所以实际频率是 freq/2,还要被整数除截断
 * (600→1 拍→500Hz,300→3 拍→167Hz,900→1 拍→500Hz)。
 * freq=0 是静音:period 清 0 让 tick 直接返回,并把 GPIO 拉低,不留半个电平。 */
static void horn_set_freq(uint16 freq)
{
    if(freq == 0)
    {
        horn_freq_target = 0;
        horn_freq_period = 0;
        gpio_set_level(KART_HORN_GPIO_PIN, GPIO_LOW);
    }
    else
    {
        horn_freq_target = freq;
        horn_freq_period = 1000 / freq;
        horn_freq_counter = 0;
    }
}

static void horn_gpio_tick(void)
{
    if(horn_freq_period == 0)
        return;

    horn_freq_counter++;
    if(horn_freq_counter >= horn_freq_period)
    {
        horn_freq_counter = 0;
        gpio_toggle_level(KART_HORN_GPIO_PIN);
    }
}

/* -------------------- 节拍表(单位:拍数,0 结尾) -------------------- */
/* 偶数索引=响拍数,奇数索引=停拍数。1拍=1ms,1000拍=1秒。 */
static const uint16 horn_1s[]        = {1000, 0};
static const uint16 horn_2s[]        = {2000, 0};
static const uint16 horn_3s[]        = {3000, 0};
static const uint16 horn_2times[]    = {1000, 1000, 1000, 0};
static const uint16 horn_3times[]    = {1000, 1000, 1000, 1000, 1000, 0};
static const uint16 horn_4times[]    = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 0};
static const uint16 horn_long_short[]= {1000, 1000, 3000, 0};
static const uint16 horn_urgent[]    = {500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 0};
/* 警报表与其它八条不同:响步全在偶数索引(1000 拍),停步写的是 0 拍,靠 ISR 里
 * "警报 + 当前步为 0 + 奇数索引就跳过"那段跨过去,所以警报是连续变频、中间不停。
 * 末尾那个 1 是占位,防最后一步被当成 0 终止符 —— 但 KART_HORN_MAX_STEPS=16
 * 先把表截断了,它到不了(头文件那条写了后果)。 */
static const uint16 horn_alarm[]     = {1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 1};

/* -------------------- 节拍机状态 -------------------- */

/* 按 cmd 选节拍表。未知命令返回 NULL(不鸣)。
 * cmd 1~9: 对应9种鸣笛(kart_voice_dispatch已将0x0C~0x14映射为1~9) */
static const uint16 *horn_select_pattern(uint8 cmd)
{
    switch(cmd)
    {
        case 1:  return horn_1s;
        case 2:  return horn_2s;
        case 3:  return horn_3s;
        case 4:  return horn_2times;
        case 5:  return horn_3times;
        case 6:  return horn_4times;
        case 7:  return horn_long_short;
        case 8:  return horn_urgent;
        case 9:  return horn_alarm;
        default: return NULL;
    }
}

/* 停止并静音。 */
static void horn_stop_internal(void)
{
    horn_pattern  = NULL;
    horn_step     = 0;
    horn_step_ticks = 0;
    horn_is_alarm = 0;
    horn_set_freq(0);
}

/* =========================== 对外接口 =========================== */
void kart_horn_init(void)
{
    gpio_init(KART_HORN_GPIO_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pit_ms_init(CCU60_CH1, 1);
    horn_stop_internal();
}

void kart_horn_start(uint8 cmd)
{
    const uint16 *pat = horn_select_pattern(cmd);

    if(pat == NULL)
    {
        return;
    }

    horn_pattern  = pat;
    horn_step     = 0;
    horn_step_ticks = 0;
    horn_is_alarm = (cmd == 9) ? 1 : 0;

    if(horn_is_alarm)
    {
        horn_set_freq(300);
    }
    else
    {
        horn_set_freq(600);
    }
}

void kart_horn_isr(void)
{
    if(horn_pattern == NULL)
    {
        return;
    }

    horn_step_ticks++;

    if(horn_step_ticks >= horn_pattern[horn_step])
    {
        horn_step_ticks = 0;
        horn_step++;

        if(horn_step >= KART_HORN_MAX_STEPS || horn_pattern[horn_step] == 0)
        {
            /* 警报的停步写的是 0 拍,不是终止符:跳过它直接进下一个响步,
             * 效果就是连续变频不停顿。别的表里偶数索引上的 0 才是真终止符。 */
            if(horn_is_alarm && horn_pattern[horn_step] == 0 && (horn_step & 0x01) != 0)
            {
                horn_step++;
                if(horn_step >= KART_HORN_MAX_STEPS || horn_pattern[horn_step] == 0)
                {
                    horn_stop_internal();
                    return;
                }
            }
            else
            {
                horn_stop_internal();
                return;
            }
        }

        if((horn_step & 0x01) == 0)
        {
            if(horn_is_alarm)
            {
                uint16 freq = (horn_step / 2) & 0x01 ? 900 : 300;
                horn_set_freq(freq);
            }
            else
            {
                horn_set_freq(600);
            }
        }
        else
        {
            if(horn_is_alarm)
            {
                uint16 freq = (horn_step / 2) & 0x01 ? 900 : 300;
                horn_set_freq(freq);
            }
            else
            {
                horn_set_freq(0);
            }
        }
    }
    horn_gpio_tick();
}

void kart_horn_stop(void)
{
    horn_stop_internal();
}

uint8 kart_horn_is_busy(void)
{
    return (horn_pattern != NULL) ? 1 : 0;
}
