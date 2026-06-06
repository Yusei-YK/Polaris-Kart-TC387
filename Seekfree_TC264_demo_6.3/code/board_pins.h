/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2026, 自制 TC264 智能车工程
 * 作者：陈俊豪 ryanchenjh@126.com
 * 文件名称          board_pins.h
 * 功能说明          自制 PCB 板级引脚统一定义
 *
 * 使用说明
 * 1. 本文件只做板级引脚映射，不修改逐飞底层库枚举。
 * 2. 原理图物理引脚定义区只描述 PCB 网络接到哪个 TC264 管脚。
 * 3. 逐飞驱动专用枚举定义区用于 uart_init/spi_init/pwm_init/exti_init/encoder_quad_init 等接口。
 * 4. UART/SPI/PWM/ERU/Encoder 不能盲目传 Pxx_x，必须使用逐飞 zf_driver_xxx.h 中存在的专用枚举。
 * 5. 找不到专用枚举的引脚使用 XXX_PIN_UNCHECKED 标记，禁止编造枚举。
 ********************************************************************************************************************/

#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#include "zf_common_headfile.h"

//==================================================== 一、原理图物理引脚定义 ========================================
// 这一部分只表示自制 PCB 原理图网络与 TC264 物理管脚的连接关系。
// 普通 GPIO、ADC、软件 IIC、蜂鸣器、按键等可以直接使用 Pxx_x 或 Axx。

//==================================================== 摄像头 MT9V03X 物理引脚 =======================================
#define BOARD_MT9V03X_UART_RX_GPIO_PIN             (P02_3)                     // 摄像头 TX -> 主控 RX
#define BOARD_MT9V03X_UART_TX_GPIO_PIN             (P02_2)                     // 摄像头 RX -> 主控 TX
#define BOARD_MT9V03X_IIC_SCL_GPIO_PIN             (P02_3)                     // 摄像头 SCCB/IIC SCL
#define BOARD_MT9V03X_IIC_SDA_GPIO_PIN             (P02_2)                     // 摄像头 SCCB/IIC SDA
#define BOARD_MT9V03X_PCLK_GPIO_PIN                (P02_1)                     // 摄像头像素时钟物理脚
#define BOARD_MT9V03X_VSYNC_GPIO_PIN               (P02_0)                     // 摄像头场同步物理脚
#define BOARD_MT9V03X_DATA_GPIO_PIN                (P00_0)                     // 摄像头 D0-D7 起始脚：P00_0 ~ P00_7

//==================================================== 屏幕接口物理引脚 ===============================================
#define BOARD_SCREEN_SCK_GPIO_PIN                  (P15_3)                     // 屏幕 SPI SCK 物理脚
#define BOARD_SCREEN_MOSI_GPIO_PIN                 (P15_5)                     // 屏幕 SPI MOSI 物理脚
#define BOARD_SCREEN_MISO_GPIO_PIN                 (P15_4)                     // 屏幕 SPI MISO 占位物理脚
#define BOARD_SCREEN_RST_GPIO_PIN                  (P15_1)                     // 屏幕复位
#define BOARD_SCREEN_DC_GPIO_PIN                   (P15_0)                     // 屏幕 DC
#define BOARD_SCREEN_CS_GPIO_PIN                   (P15_2)                     // 屏幕片选 GPIO 控制脚
#define BOARD_SCREEN_BLK_GPIO_PIN                  (P15_4)                     // 屏幕背光，注意与 MISO 占位同脚

//==================================================== 姿态传感器物理引脚 =============================================
#define BOARD_IMU_SCK_GPIO_PIN                     (P20_11)                    // IMU SCK
#define BOARD_IMU_MOSI_GPIO_PIN                    (P20_14)                    // IMU MOSI
#define BOARD_IMU_MISO_GPIO_PIN                    (P20_12)                    // IMU MISO
#define BOARD_IMU_CS_GPIO_PIN                      (P20_13)                    // IMU CS

//==================================================== 有刷电机 PWM 物理引脚 ==========================================
#define BOARD_MOTOR1_PWM1_GPIO_PIN                 (P21_2)
#define BOARD_MOTOR1_PWM2_GPIO_PIN                 (P21_3)
#define BOARD_MOTOR1_PWM3_GPIO_PIN                 (P21_4)
#define BOARD_MOTOR1_PWM4_GPIO_PIN                 (P21_5)

#define BOARD_MOTOR2_PWM1_GPIO_PIN                 (P02_4)
#define BOARD_MOTOR2_PWM2_GPIO_PIN                 (P02_5)
#define BOARD_MOTOR2_PWM3_GPIO_PIN                 (P02_6)
#define BOARD_MOTOR2_PWM4_GPIO_PIN                 (P02_7)

//==================================================== ADC 物理通道 ====================================================
#define BOARD_MIC_ADC_GPIO_PIN                     (A0)                        // 硅麦 ADC
#define BOARD_BATTERY_ADC_GPIO_PIN                 (A11)                       // 电池电压检测 ADC

//==================================================== 无线串口物理引脚 ===============================================
#define BOARD_WIRELESS_UART_TX_GPIO_PIN            (P10_5)                     // 主控 TX，接无线模块 RX
#define BOARD_WIRELESS_UART_RX_GPIO_PIN            (P10_6)                     // 主控 RX，接无线模块 TX
#define BOARD_WIRELESS_RTS_GPIO_PIN                (P10_2)
#define BOARD_WIRELESS_RST_GPIO_PIN                (P11_6)

//==================================================== 蜂鸣器物理引脚 ==================================================
#define BOARD_BEEP_GPIO_PIN                        (P33_10)                    // 自制板蜂鸣器，不是逐飞原板 P11_11

//==================================================== 编码器物理引脚 ==================================================
#define BOARD_ENCODER1_A_GPIO_PIN                  (P33_7)
#define BOARD_ENCODER1_B_GPIO_PIN                  (P33_6)

#define BOARD_ENCODER2_A_GPIO_PIN                  (P10_3)
#define BOARD_ENCODER2_B_GPIO_PIN                  (P10_1)

#define BOARD_ENCODER3_A_GPIO_PIN                  (P22_3)                     // 仅物理脚，未找到逐飞 Encoder 专用枚举
#define BOARD_ENCODER3_B_GPIO_PIN                  (P22_1)                     // 仅物理脚，未找到逐飞 Encoder 专用枚举
#define BOARD_ENCODER_EXT1_GPIO_PIN                (P22_0)
#define BOARD_ENCODER_EXT2_GPIO_PIN                (P23_1)

//==================================================== 定位模块物理引脚 ===============================================
#define BOARD_POSITION_UART_RX_GPIO_PIN            (P15_7)                     // 定位模块 TX -> 主控 RX
#define BOARD_POSITION_UART_TX_GPIO_PIN            (P15_6)                     // 定位模块 RX -> 主控 TX

//==================================================== 按键与拨码开关物理引脚 =========================================
#define BOARD_SWITCH1_GPIO_PIN                     (P33_11)
#define BOARD_SWITCH2_GPIO_PIN                     (P33_12)
#define BOARD_KEY1_GPIO_PIN                        (P20_6)
#define BOARD_KEY2_GPIO_PIN                        (P20_7)
#define BOARD_KEY3_GPIO_PIN                        (P11_2)
#define BOARD_KEY4_GPIO_PIN                        (P11_3)

//==================================================== 点阵灯光接口物理引脚 ===========================================
#define BOARD_MATRIX_UART_TX_GPIO_PIN              (P14_0)                     // 主控 UART_TX
#define BOARD_MATRIX_UART_RX_GPIO_PIN              (P14_1)                     // 主控 UART_RX
#define BOARD_MATRIX_ROW_A0_GPIO_PIN               (P20_8)
#define BOARD_MATRIX_ROW_A1_GPIO_PIN               (P20_9)
#define BOARD_MATRIX_ROW_A2_GPIO_PIN               (P20_10)
#define BOARD_MATRIX_ROW_EN_GPIO_PIN               (P33_8)

//==================================================== 二、逐飞驱动专用枚举定义 =======================================
// 这一部分用于逐飞驱动初始化接口，必须使用 zf_driver_xxx.h 中真实存在的枚举。

//==================================================== 摄像头 MT9V03X 驱动枚举 =======================================
#define BOARD_MT9V03X_UART_INDEX                   (UART_1)
#define BOARD_MT9V03X_UART_RX_PIN                  (UART1_RX_P02_3)            // 主控 RX，接摄像头 TX
#define BOARD_MT9V03X_UART_TX_PIN                  (UART1_TX_P02_2)            // 主控 TX，接摄像头 RX
#define BOARD_MT9V03X_IIC_SCL_PIN                  (P02_3)                     // 软件 IIC/SCCB 使用普通 GPIO
#define BOARD_MT9V03X_IIC_SDA_PIN                  (P02_2)                     // 软件 IIC/SCCB 使用普通 GPIO
#define BOARD_MT9V03X_PCLK_PIN                     (ERU_CH2_REQ14_P02_1)       // PCLK 必须使用 ERU/EXTI 枚举
#define BOARD_MT9V03X_VSYNC_PIN                    (ERU_CH3_REQ6_P02_0)        // VSYNC 必须使用 ERU/EXTI 枚举
#define BOARD_MT9V03X_DATA_PIN                     (P00_0)                     // 并口 D0-D7 起始 GPIO
#define BOARD_MT9V03X_DATA_ADD                     get_port_in_addr(BOARD_MT9V03X_DATA_PIN)

//==================================================== 屏幕 SPI 与 GPIO ===============================================
#define BOARD_SCREEN_SPI_INDEX                     (SPI_2)
#define BOARD_SCREEN_SPI_SCK_PIN                   (SPI2_SCLK_P15_3)
#define BOARD_SCREEN_SPI_MOSI_PIN                  (SPI2_MOSI_P15_5)
#define BOARD_SCREEN_SPI_MISO_PIN                  (SPI2_MISO_P15_4)           // 屏幕无 MISO 时仍作为 SPI 初始化占位
#define BOARD_SCREEN_SPI_CS0_PIN                   (SPI2_CS0_P15_2)            // 若使用硬件 SPI CS 时使用
#define BOARD_SCREEN_RST_PIN                       (P15_1)                     // 屏幕驱动通常按 GPIO 控制
#define BOARD_SCREEN_DC_PIN                        (P15_0)                     // 屏幕驱动通常按 GPIO 控制
#define BOARD_SCREEN_CS_PIN                        (P15_2)                     // 屏幕驱动通常按 GPIO 控制
#define BOARD_SCREEN_BLK_PIN                       (P15_4)                     // 背光 GPIO，注意与 SPI MISO 占位同脚

//我们这里用的是IPS200的显示屏

#define BOARD_IPS200_SPI_INDEX                     BOARD_SCREEN_SPI_INDEX
#define BOARD_IPS200_SCK_PIN                       BOARD_SCREEN_SPI_SCK_PIN
#define BOARD_IPS200_MOSI_PIN                      BOARD_SCREEN_SPI_MOSI_PIN
#define BOARD_IPS200_MISO_PIN                      BOARD_SCREEN_SPI_MISO_PIN
#define BOARD_IPS200_RST_PIN                       BOARD_SCREEN_RST_PIN
#define BOARD_IPS200_DC_PIN                        BOARD_SCREEN_DC_PIN
#define BOARD_IPS200_CS_PIN                        BOARD_SCREEN_CS_PIN
#define BOARD_IPS200_BLK_PIN                       BOARD_SCREEN_BLK_PIN

//==================================================== 姿态传感器驱动枚举 =============================================
// IMU963RA 默认硬件 SPI，SCK/MOSI/MISO 必须使用 SPI 专用枚举，CS 使用普通 GPIO。
#define BOARD_IMU963RA_SPI_INDEX                   (SPI_0)
#define BOARD_IMU963RA_SPI_SCK_PIN                 (SPI0_SCLK_P20_11)
#define BOARD_IMU963RA_SPI_MOSI_PIN                (SPI0_MOSI_P20_14)
#define BOARD_IMU963RA_SPI_MISO_PIN                (SPI0_MISO_P20_12)
#define BOARD_IMU963RA_CS_PIN                      (P20_13)

// MPU6050 默认软件 IIC，SCL/SDA 使用普通 GPIO。
#define BOARD_MPU6050_SCL_PIN                      (P20_11)
#define BOARD_MPU6050_SDA_PIN                      (P20_14)

//==================================================== 有刷电机 PWM 驱动枚举 ==========================================
// pwm_init() 需要 pwm_pin_enum，不能直接传 P21_2/P02_4 等普通 GPIO。
// P21_2~P21_5 与 P02_4~P02_7 同时存在 ATOM0/ATOM1 枚举，这里统一选择 ATOM0。
#define BOARD_MOTOR1_PWM1_PIN                      (ATOM0_CH0_P21_2)
#define BOARD_MOTOR1_PWM2_PIN                      (ATOM0_CH1_P21_3)
#define BOARD_MOTOR1_PWM3_PIN                      (ATOM0_CH2_P21_4)
#define BOARD_MOTOR1_PWM4_PIN                      (ATOM0_CH3_P21_5)

#define BOARD_MOTOR2_PWM1_PIN                      (ATOM0_CH4_P02_4)
#define BOARD_MOTOR2_PWM2_PIN                      (ATOM0_CH5_P02_5)
#define BOARD_MOTOR2_PWM3_PIN                      (ATOM0_CH6_P02_6)
#define BOARD_MOTOR2_PWM4_PIN                      (ATOM0_CH7_P02_7)

//==================================================== ADC 通道定义 ====================================================
#define BOARD_MIC_ADC_PIN                          (A0)
#define BOARD_BATTERY_ADC_PIN                      (A11)

//==================================================== 无线串口驱动枚举 ===============================================
#define BOARD_WIRELESS_UART_INDEX                  (UART_2)
#define BOARD_WIRELESS_UART_TX_PIN                 (UART2_TX_P10_5)            // 主控 TX，接无线模块 RX
#define BOARD_WIRELESS_UART_RX_PIN                 (UART2_RX_P10_6)            // 主控 RX，接无线模块 TX
#define BOARD_WIRELESS_RTS_PIN                     (P10_2)                     // 普通 GPIO
#define BOARD_WIRELESS_RST_PIN                     (P11_6)                     // 普通 GPIO

//==================================================== 蜂鸣器 GPIO =====================================================
#define BOARD_BEEP_PIN                             (P33_10)                    // 自制板蜂鸣器，不是逐飞原板 P11_11

//==================================================== 编码器驱动枚举 ==================================================
#define BOARD_ENCODER1_INDEX                       (TIM2_ENCODER)
#define BOARD_ENCODER1_A_PIN                       (TIM2_ENCODER_CH1_P33_7)    //左轮速度编码器
#define BOARD_ENCODER1_B_PIN                       (TIM2_ENCODER_CH2_P33_6)

#define BOARD_ENCODER2_INDEX                       (TIM5_ENCODER)
#define BOARD_ENCODER2_A_PIN                       (TIM5_ENCODER_CH1_P10_3)    //右轮角度编码器
#define BOARD_ENCODER2_B_PIN                       (TIM5_ENCODER_CH2_P10_1)
/*================ 绝对值角度传感器 / SPI 编码器接口 ================*/
/*
 * 自制 PCB P14 编码器接口
 *
 * SCK  -> P22_3
 * MISO -> P22_1
 * MOSI -> P22_0
 * CS   -> P23_1
 *
 * 注意：
 * 这是 SPI 绝对值角度传感器，不是 AB 相增量编码器。
 * 不要使用 encoder_init()，应该使用 SEEKFREE_ABSOLUTE_ENCODER 驱动。
 */

#define BOARD_ABS_ENCODER_SPI             SPI_3

#define BOARD_ABS_ENCODER_SPI_SCK_PIN     SPI3_SCK_P22_3
#define BOARD_ABS_ENCODER_SPI_MISO_PIN    SPI3_MISO_P22_1
#define BOARD_ABS_ENCODER_SPI_MOSI_PIN    SPI3_MOSI_P22_0
#define BOARD_ABS_ENCODER_SPI_CS_PIN      SPI3_CS_P23_1

#define BOARD_ABS_ENCODER_CS_GPIO         P23_1


//==================================================== 定位模块 UART 驱动枚举 =========================================
#define BOARD_POSITION_UART_INDEX                  (UART_3)
#define BOARD_POSITION_UART_RX_PIN                 (UART3_RX_P15_7)            // 主控 RX，接定位模块 TX
#define BOARD_POSITION_UART_TX_PIN                 (UART3_TX_P15_6)            // 主控 TX，接定位模块 RX

//==================================================== 按键与拨码 GPIO ================================================
#define BOARD_SWITCH1_PIN                          (P33_11)
#define BOARD_SWITCH2_PIN                          (P33_12)
#define BOARD_KEY1_PIN                             (P20_6)
#define BOARD_KEY2_PIN                             (P20_7)
#define BOARD_KEY3_PIN                             (P11_2)
#define BOARD_KEY4_PIN                             (P11_3)

//==================================================== 点阵灯光接口驱动枚举 ===========================================
#define BOARD_MATRIX_UART_INDEX                    (UART_0)
#define BOARD_MATRIX_UART_TX_PIN                   (UART0_TX_P14_0)            // 主控 UART_TX -> P14_0
#define BOARD_MATRIX_UART_RX_PIN                   (UART0_RX_P14_1)            // 主控 UART_RX -> P14_1
#define BOARD_MATRIX_ROW_A0_PIN                    (P20_8)                     // 普通 GPIO
#define BOARD_MATRIX_ROW_A1_PIN                    (P20_9)                     // 普通 GPIO
#define BOARD_MATRIX_ROW_A2_PIN                    (P20_10)                    // 普通 GPIO
#define BOARD_MATRIX_ROW_EN_PIN                    (P33_8)                     // 普通 GPIO

#endif // BOARD_PINS_H_
