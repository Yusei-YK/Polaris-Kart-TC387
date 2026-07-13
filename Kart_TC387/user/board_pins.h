#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#include "zf_common_headfile.h"

#define KART_STEER_DIR_PIN              (P21_2)
#define KART_STEER_PWM_PIN              (ATOM0_CH1_P21_3)
#define KART_LEFT_REAR_DIR_PIN          (P02_4)
#define KART_LEFT_REAR_PWM_PIN          (ATOM0_CH5_P02_5)
#define KART_RIGHT_REAR_DIR_PIN         (P02_6)
#define KART_RIGHT_REAR_PWM_PIN         (ATOM0_CH7_P02_7)

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

#define KART_STEER_ABS_SPI_INDEX        (SPI_4)
#define KART_STEER_ABS_SPI_MODE         (SPI_MODE0)
#define KART_STEER_ABS_SPI_BAUD         (1000000)
#define KART_STEER_ABS_SPI_SCK_PIN      (SPI4_SCLK_P22_3)
#define KART_STEER_ABS_SPI_MOSI_PIN     (SPI4_MOSI_P22_0)
#define KART_STEER_ABS_SPI_MISO_PIN     (SPI4_MISO_P22_1)
#define KART_STEER_ABS_SPI_HW_CS_PIN    (SPI_CS_NULL)
#define KART_STEER_ABS_CS_GPIO_PIN      (P23_1)
#define KART_STEER_ABS_RAW_SHIFT        (4)
#define KART_STEER_ABS_CENTER_RAW       (2048)
#define KART_STEER_ABS_LEFT_LIMIT_RAW   (1600)
#define KART_STEER_ABS_RIGHT_LIMIT_RAW  (2496)

/* ---------------- IMU963RA(六轴,SPI_0)---------------- */
/* 引脚三方对齐:demo board_pins.h + 硬件交接文档 3.4 + imu963ra.h 库默认,全一致。
 * 卡丁车科目一只用 yaw(航向),6DOF 陀螺+加速度,不用磁力计(躲电机磁干扰)。
 * 真正生效的是逐飞 zf_device_imu963ra.h 中的配置；这里直接引用它，避免只改了
 * board_pins 的副本、实际驱动引脚却没有变化。*/
#define KART_IMU963RA_SPI_INDEX         (IMU963RA_SPI)
#define KART_IMU963RA_SPI_SCK_PIN       (IMU963RA_SPC_PIN)
#define KART_IMU963RA_SPI_MOSI_PIN      (IMU963RA_SDI_PIN)
#define KART_IMU963RA_SPI_MISO_PIN      (IMU963RA_SDO_PIN)
#define KART_IMU963RA_SPI_CS_PIN        (IMU963RA_CS_PIN)

#define BOARD_WIRELESS_UART_INDEX       (UART_2)
#define BOARD_WIRELESS_UART_TX_PIN      (UART2_TX_P10_5)
#define BOARD_WIRELESS_UART_RX_PIN      (UART2_RX_P10_6)
#define BOARD_WIRELESS_UART_BAUD        (115200)
#define BOARD_WIRELESS_UART_BAUD_FAST   (460800)
#define BOARD_WIRELESS_RTS_PIN          (P10_2)     /* 无线模块 RTS(交接文档 3.6,暂未用) */
#define BOARD_WIRELESS_RST_PIN          (P11_6)     /* 无线模块复位(暂未用) */

/* ---------------- GPS(UART_3,交接文档 3.5)---------------- */
/* 主板有 GPS,科目一先跑纯惯导,GPS 仅作辅助/以后融合用。
 * 注意方向:GPS_TX→MCU_RX=P15.7,MCU_TX→GPS_RX=P15.6,所以 UART3 的 TX 是 P15_6、RX 是 P15_7。*/
#define BOARD_GPS_UART_INDEX            (UART_3)
#define BOARD_GPS_UART_TX_PIN          (UART3_TX_P15_6)
#define BOARD_GPS_UART_RX_PIN          (UART3_RX_P15_7)
#define BOARD_GPS_UART_BAUD            (115200)

/* ---------------- 屏幕(SPI_2,第一版 SPI 屏,交接文档 3.6)---------------- */
/* SPI 屏是只写设备,不用 MISO;文档里 P15.4 既标 MISO 占位又标背光,实际当背光 BL 用。*/
#define BOARD_LCD_SPI_INDEX            (IPS200_SPI)
#define BOARD_LCD_SPI_SCK_PIN          (IPS200_SCL_PIN_SPI)
#define BOARD_LCD_SPI_MOSI_PIN         (IPS200_SDA_PIN_SPI)
#define BOARD_LCD_RST_PIN              (IPS200_RST_PIN_SPI)
#define BOARD_LCD_DC_PIN               (IPS200_DC_PIN_SPI)
#define BOARD_LCD_CS_PIN               (IPS200_CS_PIN_SPI)
#define BOARD_LCD_BL_PIN               (IPS200_BLk_PIN_SPI)

/* ---------------- 蜂鸣器 / ADC 检测(交接文档 3.6 / 5)---------------- */
#define BOARD_BEEP_PIN                 (P33_10)     /* 蜂鸣器输出 */
#define BOARD_MIC_ADC_CH               (ADC0_CH0_A0)    /* 硅麦采集(发车声控) */
#define BOARD_VBAT_ADC_CH              (ADC1_CH3_A11)   /* 电池电压检测,分压比待硬件标注 */

/* ---------------- 灯板 TLD7002(第一版,交接文档 3.6)---------------- */
/* P14.0/P14.1 在逐飞 TC387 枚举中明确属于 UART0，索引已经确认。
 * 但 UART0 当前同时被 debug_init() 占用；灯板底层接入前必须先迁移或关闭 Debug UART，
 * 绝不能让两个模块同时初始化 UART0。*/
#define BOARD_LED_UART_INDEX           (UART_0)
#define BOARD_LED_UART_TX_PIN          (UART0_TX_P14_0)
#define BOARD_LED_UART_RX_PIN          (UART0_RX_P14_1)
#define BOARD_LED_ROW_A0_PIN           (P20_8)
#define BOARD_LED_ROW_A1_PIN           (P20_9)
#define BOARD_LED_ROW_A2_PIN           (P20_10)
#define BOARD_LED_ROW_EN_PIN           (P33_8)

#define KART_MAIN_LOOP_PERIOD_MS        (5)

#endif
