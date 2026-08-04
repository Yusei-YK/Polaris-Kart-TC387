#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#include "zf_common_headfile.h"

#define KART_STEER_DIR_PIN              (P21_2)
#define KART_STEER_PWM_PIN              (ATOM0_CH1_P21_3)
/* 2026-07-17 单轮开环诊断(L800/R800)实测:软件左通道原接 P02_5 组却驱动物理右轮,
 * 软件右通道原接 P02_7 组却驱动物理左轮 —— PWM/DIR 引脚组左右接反,是双 PI 跑飞的根因。
 * 方向(两侧均正转)、编码器(I9=左/I10=右)都对,故只把左右后轮引脚整组对调。 */
#define KART_LEFT_REAR_DIR_PIN          (P02_6)
#define KART_LEFT_REAR_PWM_PIN          (ATOM0_CH7_P02_7)
#define KART_RIGHT_REAR_DIR_PIN         (P02_4)
#define KART_RIGHT_REAR_PWM_PIN         (ATOM0_CH5_P02_5)

#define KART_STEER_MOTOR_SIGN           (+1)
#define KART_LEFT_MOTOR_SIGN            (+1)
#define KART_RIGHT_MOTOR_SIGN           (+1)

#define KART_LEFT_ENCODER_INDEX         (TIM2_ENCODER)
#define KART_LEFT_ENCODER_CH1           (TIM2_ENCODER_CH1_P33_7)
#define KART_LEFT_ENCODER_CH2           (TIM2_ENCODER_CH2_P33_6)
#define KART_LEFT_ENCODER_A_GPIO        (P33_7)
#define KART_LEFT_ENCODER_B_GPIO        (P33_6)
#define KART_RIGHT_ENCODER_INDEX        (TIM5_ENCODER)
#define KART_RIGHT_ENCODER_CH1          (TIM5_ENCODER_CH1_P10_3)
#define KART_RIGHT_ENCODER_CH2          (TIM5_ENCODER_CH2_P10_1)
#define KART_RIGHT_ENCODER_A_GPIO       (P10_3)
#define KART_RIGHT_ENCODER_B_GPIO       (P10_1)
#define KART_LEFT_ENCODER_SIGN          (+1)
#define KART_RIGHT_ENCODER_SIGN         (-1)

/* 2026-07-24 换新左编码器后重新标定(左右轮分别手掰十圈,大缓冲抓全):
 * 左十圈累计 20481、右十圈累计 20489 → 均≈2048 脉冲/圈(512 线×4 正交),
 * 左右 PPR 一致(比 1.0004),排除编码器量纲差异是起步打滑主因。
 * 后轮实测直径 240 mm → 周长 π×0.240 = 0.753982 m,
 * PULSE_TO_M = 0.753982 / 2048 = 0.00036816 m/脉冲。
 * (旧值 0.00038777 偏大约 5.3%,里程会高估;手推5m因20s窗口截断作废。) */
#define KART_LEFT_ENC_PULSE_TO_M        (0.00036816f)
#define KART_RIGHT_ENC_PULSE_TO_M       (0.00036816f)

#define KART_STEER_ABS_SPI_INDEX        (SPI_4)
#define KART_STEER_ABS_SPI_MODE         (SPI_MODE0)
#define KART_STEER_ABS_SPI_BAUD         (1000000)
#define KART_STEER_ABS_SPI_SCK_PIN      (SPI4_SCLK_P22_3)
#define KART_STEER_ABS_SPI_MOSI_PIN     (SPI4_MOSI_P22_0)
#define KART_STEER_ABS_SPI_MISO_PIN     (SPI4_MISO_P22_1)
#define KART_STEER_ABS_SPI_HW_CS_PIN    (SPI_CS_NULL)
#define KART_STEER_ABS_CS_GPIO_PIN      (P23_1)
#define KART_STEER_ABS_RAW_SHIFT        (4)
/* 2026-07-30 换齿轮+编码器+转向电机后重标(CH14 平台值,已解 4096 环绕):
 * 左死 1248、右死 3176(相对中值 -1084)、中值取几何中点 164(手停读数 192/177 吻合)。
 * 行程 +1078/-1084 基本对称,总行程 2162(旧 2251),满舵半径回到 ~1.4m。
 * 【注意】中值贴着 raw 0,CH14 在中值附近会在 0/4095 之间跳,这是正常的;
 * delta 由 wrap_delta() 做 mod-4096 修正,除显示外无代码直接比较 raw。 */
#define KART_STEER_ABS_CENTER_RAW       (164)   /* 2026-07-30 实测(几何中点) */
#define KART_STEER_ABS_LEFT_LIMIT_RAW   (1248)  /* 最左硬限位 raw */
#define KART_STEER_ABS_RIGHT_LIMIT_RAW  (3176)  /* 最右硬限位 raw(过 0 环绕) */

/* ---------------- IMU660RA(六轴,SPI_0)----------------
 * 最新网表与 zf_device_imu660ra.h 默认引脚一致。
 * 科目一只用陀螺+加速度做 6DOF yaw，不用磁力计。 */
#define KART_IMU660RA_SPI_INDEX         (SPI_0)
#define KART_IMU660RA_SPI_SCK_PIN       (SPI0_SCLK_P20_11)
#define KART_IMU660RA_SPI_MOSI_PIN      (SPI0_MOSI_P20_14)
#define KART_IMU660RA_SPI_MISO_PIN      (SPI0_MISO_P20_12)
#define KART_IMU660RA_SPI_CS_PIN        (P20_13)

/* v2 主板:无线模块数据脚使用 UART1(P33.12/P33.13)。
 * 2026-07-19 最新网表坐实:无线串口.1→P33.12(TX)、.3→P33.13(RX)、.4→P10.2(RTS)、.8→P11.6(RST)。
 * 注意:P33.12/P33.13 与摄像头 UART1 资源冲突,已确认无线与摄像头不同时用。
 */
#define BOARD_WIRELESS_UART_INDEX       (UART_1)
#define BOARD_WIRELESS_UART_TX_PIN      (UART1_TX_P33_12)
#define BOARD_WIRELESS_UART_RX_PIN      (UART1_RX_P33_13)
#define BOARD_WIRELESS_UART_BAUD        (115200)
#define BOARD_WIRELESS_UART_BAUD_FAST   (460800)
#define BOARD_WIRELESS_RTS_PIN          (P10_2)     /* 无线模块 RTS(暂未用),v2 网表不变 */
#define BOARD_WIRELESS_RST_PIN          (P11_6)     /* 无线模块复位(暂未用),v2 网表不变 */

/* VOFA 日志串口。
 * 2026-07-24 由 UART2(P14.2/P14.3)改到 UART10(P13.0/P13.1)对接无线模块。
 *
 * 2026-07-27 增加 UART0(P14.0/P14.1)选项:那天没带无线模块,只能用 USB-TTL 直插
 * 抓 VOFA。P13.0/P13.1 是无线模块排针位,不方便接 TTL 线,故改用空闲的 ASCLIN0。
 *
 * 2026-07-28 改回 0:带了无线模块,插 P13.0/P13.1 排针无线抓 VOFA,不用拖线。
 *   KART_LOG_ON_UART0 = 1 → UART_0 / P14.0(TX) / P14.1(RX)  (USB-TTL 直插备用)
 *   KART_LOG_ON_UART0 = 0 → UART_10 / P13.0 / P13.1(无线模块)  ← 当前
 *
 * 前提与已知风险:
 *   ① P14.0/P14.1 不在《尽量不要使用的引脚.txt》禁用表内(表里是 P14.2~P14.6);
 *   ② 全工程 ASCLIN0 无其他用户,uart0_rx_isr(isr.c)只做丢弃兜底,不冲突;
 *   ③ 【历史结论】cpu0_main.c 注释记着 2026-07-26 给 TLD7002 挪线时"UART0 实测
 *      收发不通"(ERR=1/RX=0/回环 0)。但那次是灯板 2M 半双工用法,与本处 460800
 *      单向 TX 不同,未必同因。若实测仍不通,把 KART_LOG_ON_UART0 改回 0。
 *
 * 切到 UART0 后日志与语音不再共用外设 → BOARD_VOICE_SHARES_AUX_UART 自动变 0,
 * 进出科目二不再切波特率/停日志(语音仍独占 UART_10)。 */
#define KART_LOG_ON_UART0               (0)

#if KART_LOG_ON_UART0
#define BOARD_AUX_UART_INDEX            (UART_0)
#define BOARD_AUX_UART_TX_PIN          (UART0_TX_P14_0)
#define BOARD_AUX_UART_RX_PIN          (UART0_RX_P14_1)
#else
#define BOARD_AUX_UART_INDEX            (UART_10)
#define BOARD_AUX_UART_TX_PIN          (UART10_TX_P13_0)
#define BOARD_AUX_UART_RX_PIN          (UART10_RX_P13_1)
#endif
#define BOARD_AUX_UART_BAUD            (115200)
#define BOARD_AUX_UART_BAUD_FAST       (460800)

/* ---------------- 语音模块(科目二,2026-07-26 定案)----------------
 * 背景:语音原接 P33.12/13,那是 UART_1=ASCLIN1,与 TLD7002 灯板飞线
 * (P11.12/P11.10 @2M)是同一个硬件外设,谁后 init 谁改波特率 → 灯板灭。
 * 库里 UART_1 只映射到 ASCLIN1,软件无法共存,所以必须换外设。
 *
 * 为什么落在 P13.0/P13.1(UART_10=ASCLIN10):
 *   ① 全工程 ASCLIN10 只有 VOFA 日志一个用户,而比赛不接无线模块 → 该外设本就空闲;
 *   ② 语音模块直接插原无线排针即可,不用飞线;
 *   ③ 不动 ASCLIN1(灯板)、不动 ASCLIN3(SBUS 遥控)。遥控是唯一人工接管兜底,
 *      不能为让位语音而拔掉,所以放弃了 P15.6/15.7 方案。
 *
 * 使用前提(硬件):科二运行前拔掉无线模块,把语音模块插上,
 *   语音 TX → MCU P13.1(RX),语音 RX → MCU P13.0(TX)。
 * 使用前提(软件):UART_10 平时是 460800 跑日志,进科目二要重配到 115200 并停日志,
 *   见 kart_mission.c 的 mission_enter/mission_exit(MISSION_SUBJECT_2)。
 *
 * KART_VOICE_ON_AUX_UART=0 可一键退回旧接法(P33.12/13),但那样灯板与语音仍不能共存。 */
#define KART_VOICE_ON_AUX_UART          (1)

#if KART_VOICE_ON_AUX_UART
#define BOARD_VOICE_UART_INDEX          (UART_10)
#define BOARD_VOICE_UART_TX_PIN         (UART10_TX_P13_0)
#define BOARD_VOICE_UART_RX_PIN         (UART10_RX_P13_1)
#else
#define BOARD_VOICE_UART_INDEX          (BOARD_WIRELESS_UART_INDEX)
#define BOARD_VOICE_UART_TX_PIN         (BOARD_WIRELESS_UART_TX_PIN)
#define BOARD_VOICE_UART_RX_PIN         (BOARD_WIRELESS_UART_RX_PIN)
#endif
#define BOARD_VOICE_UART_BAUD           (115200)    /* 语音模块固定 115200 8N1,不可改 */

/* 语音与日志是否共用同一外设:为 1 时进科二必须切波特率 + 停日志。
 * 2026-07-27:日志可切到 UART0,故不能再写死等于 KART_VOICE_ON_AUX_UART ——
 * 两者都落在 UART_10 时才算共用。日志在 UART0 时科二无需停日志/切波特率。 */
#define BOARD_VOICE_SHARES_AUX_UART     (KART_VOICE_ON_AUX_UART && !KART_LOG_ON_UART0)

/* ---------------- GPS(UART_3,交接文档 3.5)---------------- */
/* 主板有 GPS,科目一先跑纯惯导,GPS 仅作辅助/以后融合用。
 * 注意方向:GPS_TX→MCU_RX=P15.7,MCU_TX→GPS_RX=P15.6,所以 UART3 的 TX 是 P15_6、RX 是 P15_7。*/
#define BOARD_GPS_UART_INDEX            (UART_3)
#define BOARD_GPS_UART_TX_PIN          (UART3_TX_P15_6)
#define BOARD_GPS_UART_RX_PIN          (UART3_RX_P15_7)
#define BOARD_GPS_UART_BAUD            (115200)

/* ---------------- 屏幕(SPI_2,第一版 SPI 屏,交接文档 3.6)---------------- */
/* SPI 屏是只写设备,不用 MISO;文档里 P15.4 既标 MISO 占位又标背光,实际当背光 BL 用。*/
#define BOARD_LCD_SPI_INDEX            (SPI_2)
#define BOARD_LCD_SPI_SCK_PIN          (SPI2_SCLK_P15_3)
#define BOARD_LCD_SPI_MOSI_PIN         (SPI2_MOSI_P15_5)
#define BOARD_LCD_RST_PIN              (P15_1)
#define BOARD_LCD_DC_PIN               (P15_0)
#define BOARD_LCD_CS_PIN               (P15_2)
#define BOARD_LCD_BL_PIN               (P15_4)

/* ---------------- 按键板(2026-07-28 新板,按键板.tel 网表)----------------
 * 板上三个操作件,H1 是 2.54-2×6P 排线到主板:
 *   SW1  EC11 旋转编码器  A=P11.2  B=P11.3  按下(D)=P20.6   (C/E 脚接 GND)
 *   SW2  五向开关         UP=P33.11 DOWN=P20.0 LEFT=P21.6 RIGHT=P21.7 MID=P33.4
 *   SW3  轻触开关         START=P20.7                        (3/4 脚接 GND)
 *
 * 电平:三个件的公共端全接 GND,按下/导通把信号脚拉低 → 按下读 0。
 * 上拉在主板侧(按键板上的 R1/4.7k 是 LED1 限流,不是按键上拉),故软件用
 * GPI_FLOATING_IN,不叠片内上拉。若日后换成不带上拉的主板,把 kart_menu_init /
 * kart_mission_init 里这几个脚改 GPI_PULL_UP 即可,逻辑不用动。
 *
 * 与旧板的差异(换板后同步改的地方):
 *   DOWN   P20.6 → P20.0    P20.6 让给旋钮按下
 *   MID    P20.7 → P33.4    P20.7 让给独立 START 键
 *   LEFT   P33.4 → P21.6    旧板 P21.6 与 UP 短路,故当年拿 P33.4 顶 LEFT,新板已修
 *   START  仍是 P20.7,但不再与菜单 MID 共用一个脚 ——
 *          就绪/科目四界面不必再屏蔽菜单 MID,menu 与 mission 也不会再对同一脚
 *          做两种 gpio_init(旧板 menu 配 FLOATING、mission 配 PULL_UP,谁后 init 谁生效)。
 *
 * 引脚占用已核对:P11.2/11.3 空闲(灯板占 P11.10/11.12,无线 RST 占 P11.6);
 * P20.0 空闲(isr.c 里 ERU_CH6_REQ9_P20_0 是注释掉的示例);P21.6/21.7 空闲
 * (转向占 P21.2/21.3);均不在《尽量不要使用的引脚.txt》(P14.2~14.6/P10.5/P10.6)内。 */
#define BOARD_KEY_UP_PIN               (P33_11)
#define BOARD_KEY_DOWN_PIN             (P20_0)
#define BOARD_KEY_LEFT_PIN             (P21_6)
#define BOARD_KEY_RIGHT_PIN            (P21_7)
#define BOARD_KEY_MID_PIN              (P33_4)

#define BOARD_ENC_A_PIN                (P11_2)
#define BOARD_ENC_B_PIN                (P11_3)
#define BOARD_ENC_SW_PIN               (P20_6)      /* 旋钮按下,当第二个 MID 用 */

/* 发车键(独立轻触开关)。上拉输入,按下接地读 0,科目一/科目四等此键触发。
 * 若实测按下读 1,把 kart_mission 里的边沿判据取反即可。 */
#define BOARD_START_KEY_PIN            (P20_7)

/* ---------------- 蜂鸣器 / ADC 检测(交接文档 3.6 / 5)---------------- */
#define BOARD_BEEP_PIN                 (P33_10)     /* 蜂鸣器输出 */
#define BOARD_MIC_ADC_CH               (ADC0_CH0_A0)    /* 硅麦采集(发车声控) */
#define BOARD_VBAT_ADC_CH              (ADC1_CH3_A11)   /* 电池电压检测,分压比待硬件标注 */

/* ---------------- 灯板 TLD7002 ---------------- */
/* 引脚定义不在这里 —— 见 zf_device_dot_matrix_screen.h(行译码 A0/A1/A2/EN、SYNC)
 * 与 zf_device_tld7002.h(TLD7002_UART_* = ASCLIN1 @2M,P11.12/P11.10)。
 * 注意:第一版曾把灯板挂 UART0/P14.0/P14.1,已废弃且不可复用 ——
 * P14.x 属 boot 相关引脚,占用会导致 MCU 下载不进去(见项目根 不建议使用的引脚.txt)。 */

#define KART_MAIN_LOOP_PERIOD_MS        (5)

#endif
