#ifndef KART_HORN_H_
#define KART_HORN_H_
#include "zf_common_headfile.h"
#include "board_pins.h"
#include "kart_voice.h"

/*
 * 科目二鸣笛执行层 —— 独立中断驱动节拍机
 * ------------------------------------------------------------------
 * 蜂鸣器接 BOARD_BEEP_PIN(P33_10),GPIO 软件翻转产生方波。
 * 使用 CCU60_CH1 独立1ms中断运行节拍机和GPIO翻转。
 * 节拍表控制"响-停"时序,中断内按目标频率翻转GPIO产生音调。
 *
 * 用法:
 *   kart_horn_init();                    // 科目二进入时(会启动CCU60_CH1中断)
 *   kart_horn_start(cmd);                // 派发命令时(cmd=1~9)
 *   if(kart_horn_is_busy()) ...          // 判断是否还在鸣
 *
 * 蜂鸣器为无源蜂鸣器,通过GPIO翻转产生方波驱动。
 * 【600/300/900 是写进 horn_set_freq() 的参数,不是实际发出的频率】
 * horn_set_freq(f) 里 period = 1000/f 拍(1 拍 1ms),每 period 拍翻转一次 GPIO;
 * 翻转两次才凑一个方波周期,所以实际频率是 f/2,再被整数除截断:
 *   普通鸣笛 600 → period 1 拍 → 实际 500Hz
 *   警报 300      → period 3 拍 → 实际约 167Hz
 *   警报 900      → period 1 拍 → 实际 500Hz,和普通鸣笛同频
 * 所以警报听到的是 167Hz 与 500Hz 交替,每 1 秒切换。拿示波器核对时别按
 * 600/300/900 去比。当前就这样,没动过 —— 能响、两个音能分辨,够用。
 *
 * 为什么不在主循环里数拍:主循环实测约 41ms(IPS200 阻塞式 SPI 刷屏占约 36ms),
 * 按循环次数计拍会把 1 秒跑成 8.2 秒,所以节拍机放进 1ms 中断。
 * (8.2s = 200 拍 × 41ms,算法对得上。但那个 41ms 没记日期,是当时的主循环时长,
 *  现在不是这个数了 —— 结论仍然成立,数字别拿去当依据。要量就看 VOFA 里每拍
 *  派发耗时那一路,不要再数循环次数。)
 * 为什么不用硬件 PWM:pwm_init() 内部每次都 IfxGtm_Atom_Pwm_start() 重启
 * GTM ATOM,重配期间输出不稳,频繁切频会变成断续脉冲而不是连续方波。
 * ------------------------------------------------------------------
 */

#define KART_HORN_GPIO_PIN          (P33_10)

/* 节拍表单步最大数(响/停交替计,含结尾 0 终止)。
 * 【这个 16 会截断最长的表】horn_urgent 15 项、horn_alarm 20 项。
 * 警报的响步在偶数索引上,ISR 走到第 16 步就停,表里 10 个响步只走完 8 个 ——
 * 警报实际鸣 8 秒,不是表面上的 10 秒,末尾那个占位的 1(见 kart_horn.c 的
 * horn_alarm)永远到不了。
 * 这是现状,没改:要按表跑满就把这个数放到 20 以上,代价是警报长 2 秒。 */
#define KART_HORN_MAX_STEPS         (16)

/* 科目二进入时调:init GPIO(拉低静音)+ 复位节拍机。 */
void kart_horn_init(void);

/* 派发鸣笛命令:cmd 为 KART_VOICE_HORN_*。加载对应节拍表并开始。
 * 若正在鸣,新命令直接覆盖重放(按需可改排队,当前简单覆盖)。 */
void kart_horn_start(uint8 cmd);

/* CCU60_CH1中断调用(1ms):推进节拍,翻转蜂鸣器 GPIO。内部使用,外部不调用。 */
void kart_horn_isr(void);

/* 立即停止鸣笛并静音:复位节拍机 + 拉低蜂鸣器 GPIO。
 * 退出科目二时调,保证蜂鸣器不残留鸣响。 */
void kart_horn_stop(void);

/* 是否正在鸣笛(执行层判断长动作是否结束用)。 */
uint8 kart_horn_is_busy(void);

#endif
