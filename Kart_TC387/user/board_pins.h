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

/* 2026-07-17 修复 TIM2 丢余数后，地面手推 3 m 三次最终标定:
 * 左累计 7705/7730/7739，右累计 7759/7720/7766。
 * 六组车轮总路程 18 m / 总脉冲 46419 = 0.00038777 m/脉冲。 */
#define KART_LEFT_ENC_PULSE_TO_M        (0.00038777f)
#define KART_RIGHT_ENC_PULSE_TO_M       (0.00038777f)

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

/* v2 新增第二路串口(无线串口1,4P 座),UART2 @ P14.2/P14.3。
 * 当前底盘固件未使用,预留给科目二离线语音/日志。启用时按需 uart_init。 */
#define BOARD_AUX_UART_INDEX            (UART_2)
#define BOARD_AUX_UART_TX_PIN          (UART2_TX_P14_2)
#define BOARD_AUX_UART_RX_PIN          (UART2_RX_P14_3)
#define BOARD_AUX_UART_BAUD            (115200)

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

/* ---------------- 灯板 TLD7002(第一版,交接文档 3.6)---------------- */
/* 灯板 7x15 单色点阵,TLD7002驱动 + 3位行选译码。P14.0/P14.1 对应 UART0。 */
#define BOARD_LED_UART_INDEX           (UART_0)
#define BOARD_LED_UART_TX_PIN          (UART0_TX_P14_0)
#define BOARD_LED_UART_RX_PIN          (UART0_RX_P14_1)
#define BOARD_LED_UART_BAUD            (2000000)    /* TLD7002波特率,待硬件确认 */
#define BOARD_LED_ROW_A0_PIN           (P20_8)
#define BOARD_LED_ROW_A1_PIN           (P20_9)
#define BOARD_LED_ROW_A2_PIN           (P20_10)
#define BOARD_LED_ROW_EN_PIN           (P33_8)

#define KART_MAIN_LOOP_PERIOD_MS        (5)

#endif
