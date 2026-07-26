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
#define KART_STEER_ABS_CENTER_RAW       (1575)  /* 2026-07-19 齿轮重装后实测 */
#define KART_STEER_ABS_LEFT_LIMIT_RAW   (2728)  /* 最左硬限位 raw */
#define KART_STEER_ABS_RIGHT_LIMIT_RAW  (477)   /* 最右硬限位 raw */

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

/* VOFA 日志串口:接无线模块 @ UART10 P13.0(TX)/P13.1(RX)。
 * 2026-07-24 由 UART2(P14.2/P14.3)改到 UART10(P13.0/P13.1)对接无线模块。 */
#define BOARD_AUX_UART_INDEX            (UART_10)
#define BOARD_AUX_UART_TX_PIN          (UART10_TX_P13_0)
#define BOARD_AUX_UART_RX_PIN          (UART10_RX_P13_1)
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

/* 语音与日志是否共用同一外设:为 1 时进科二必须切波特率 + 停日志。 */
#define BOARD_VOICE_SHARES_AUX_UART     (KART_VOICE_ON_AUX_UART)

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

/* ---------------- 发车按键(v2 网表 H1 排针 START)---------------- */
/* START 键接 P20.7,上拉输入,按下接地读 0。科目一等此键触发发车。
 * 若实测按下读 1(高有效),把 kart_mission 里的边沿判据取反即可。 */
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
