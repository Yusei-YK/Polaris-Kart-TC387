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
 * 普通鸣笛:600Hz。
 * 警报模式:300Hz/900Hz交替,每1秒切换。
 *
 * 为什么不在主循环里数拍:主循环实测约 41ms(IPS200 阻塞式 SPI 刷屏占约 36ms),
 * 按循环次数计拍会把 1 秒跑成 8.2 秒,所以节拍机放进 1ms 中断。
 * 为什么不用硬件 PWM:pwm_init() 内部每次都 IfxGtm_Atom_Pwm_start() 重启
 * GTM ATOM,重配期间输出不稳,频繁切频会变成断续脉冲而不是连续方波。
 * ------------------------------------------------------------------
 */

#define KART_HORN_GPIO_PIN          (P33_10)

/* 节拍表单步最大数(响/停交替计,含结尾 0 终止)。9 条里最长是 4 声=8 步。 */
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
