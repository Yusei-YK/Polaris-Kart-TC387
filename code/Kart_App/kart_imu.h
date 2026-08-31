#ifndef KART_IMU_H_
#define KART_IMU_H_
#include "zf_common_headfile.h"
#include "kart_calc.h"

/*
 * 卡丁车航向解算库(IMU660RA + Madgwick 6DOF)
 * ------------------------------------------------------------------
 * 姿态算法参考 TopSpeed；送进算法前的二阶巴特沃斯前置滤波照搬 NEUQ 开源
 * 惯导(知乎 Morever)；传感器读数按本车 IMU660RA 重新适配。
 * 移植原则:先照搬,再按本车实测改。相对原工程做了三处"减法":
 *   1. 砍掉 Subject 发射方向偏置(科目一以后再加,现在只要纯 yaw)
 *   2. 砍掉撞击检测、自由落体检测(卡丁车用不上)
 *   3. 零偏标定不写 Flash、不刷 IPS 屏,只算到内存里(先跑起来)
 * 又做了两处"加法",都在 MadgwickAHRSupdateIMU() 里面:
 *   1. 四元数积分改用实测步长 g_imu_dt,不再用硬编码 1/sampleFreq
 *      (kart_imu.c:237),中断抖动时积分才不偏
 *   2. 加了 NaN/Inf 复位保护(kart_imu.c:252),一次异常输入
 *      不会永久污染航向
 * 另外 read_IMU() 的均值滤波换成了二阶巴特沃斯(陀螺 7.5Hz、加速度 35.0Hz)。
 * 结构体名、字段名尽量与 TopSpeed 保持一致,方便以后接科目状态机。
 * ------------------------------------------------------------------
 * 硬件:IMU660RA 挂 SPI_0,引脚见 board_pins.h / 逐飞库 zf_device_imu660ra.h。
 * 采样:200Hz(5ms),必须由 5ms 定时中断稳定驱动 kart_imu_update()。
 * ------------------------------------------------------------------
 */

/* -------------------------------------------------------------------------
 * 送进 Madgwick 算法的一帧传感器数据
 * 说明:m 前缀是 TopSpeed 原名(measure),Gyro 单位 rad/s,Acc 单位 g。
 *       attitude 是这一帧算完后的姿态四元数(输出)。
 * ------------------------------------------------------------------------- */
typedef struct{
    float mGyroX;   // 陀螺 X(rad/s)
    float mGyroY;   // 陀螺 Y(rad/s)
    float mGyroZ;   // 陀螺 Z(rad/s)
    float mAccX;    // 加速度 X(g)
    float mAccY;    // 加速度 Y(g)
    float mAccZ;    // 加速度 Z(g)
    QUAT  attitude; // 解算输出的姿态四元数
}                                                                               sSensorData;

/* -------------------------------------------------------------------------
 * IMU 原始/已换算数据(去零偏后,给调试和以后的航位推算用)
 * Gyro 单位 °/s,Acc 单位 m/s^2
 * ------------------------------------------------------------------------- */
typedef struct{
    float GyroX;
    float GyroY;
    float GyroZ;
    float AccX;
    float AccY;
    float AccZ;
}                                                                               IMU_data_RAW_struct;

/* -------------------------------------------------------------------------
 * 陀螺零偏(静止标定得到,单位:原始 LSB,和陀螺原始读数同量纲)
 * ------------------------------------------------------------------------- */
typedef struct{
    float GYRO_X_bias;
    float GYRO_Y_bias;
    float GYRO_Z_bias;
}                                                                               IMU_CaliData_Struct;

/* -------------------------------------------------------------------------
 * IMU 总句柄(把一个模块的状态打包在一个结构体里,C 里常见的"面向对象"写法)
 * ------------------------------------------------------------------------- */
typedef struct{
    EulerAngles         Attitude;                   // 当前姿态(度):roll/pitch/yaw
    IMU_data_RAW_struct RAW_data;                    // 最近一帧去零偏后的原始量
    IMU_CaliData_Struct IMU_CaliData;                // 陀螺零偏
    uint8               FLAG_enable_running_CALLBACK; // =1 才允许 kart_imu_update 干活(标定时置 0)
}                                                                               IMU_Handle_struct;

/* 让别的文件(比如调试串口)能读到姿态。extern = "这变量在别处定义,这里只是声明" */
extern IMU_Handle_struct IMU_Handle;

/* --- 对外接口 --- */
uint8 kart_imu_init(void);      // 1=初始化成功并完成标定，0=硬件初始化失败
uint8 kart_imu_is_ready(void);  // 1=IMU 可用；自动任务启动前检查
void  kart_imu_set_calib_hook(void (*hook)(void));  // 注册标定期间刷新钩子(点阵扫描),NULL 关闭
void  kart_imu_update(void);    // 读数 + Madgwick 解算,放 5ms 定时中断里调
float kart_imu_get_yaw(void);   // 取当前航向角(度,-180~180)
float kart_imu_get_yaw_filtered(void);  // 取滤波后航向角(5拍滑动平均,抑制抖动)
void  kart_imu_set_yaw(float target_yaw); // 强制设定当前航向(度):保持 roll/pitch、只改 yaw(科目三反向复现用)

/* --- 内部函数(照搬 TopSpeed,一并暴露方便调试) --- */
void reset_attitude(void);              // 把四元数复位成单位四元数(航向清零)
void IMU_check(void);                   // 静止采样求陀螺零偏
void read_IMU(sSensorData *sd);         // 读一帧并换算单位
void get_IMU_RAW(IMU_data_RAW_struct *data);        // 读原始寄存器 + 去零偏 + 换算
void MadgwickAHRSupdateIMU(sSensorData *sd);        // Madgwick 6DOF 姿态更新

/* 只读诊断量,不参与姿态反馈。读者是 VOFA 波形通道:
 * yaw_rate_dps 走 CH11/CH18、dt_us 走 CH24(见 kart_debug_uart.c);
 * yaw_bias_dps 和 acc_norm_g 目前没有任何读者,留着排零偏/振动时手接。 */
float kart_imu_get_yaw_rate_dps(void);
float kart_imu_get_yaw_bias_dps(void);
float kart_imu_get_acc_norm_g(void);
float kart_imu_get_dt_us(void);         // 取本帧实际积分步长(微秒),验证 5ms 中断是否稳定

#endif
