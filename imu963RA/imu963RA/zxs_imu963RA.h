/*
 * init.h
 *
 *  Created on: 2022Äê12ÔÂ16ÈÕ
 *      Author: paper
 */
#ifndef _ZXS_IMU963RA_H_
#define _ZXS_IMU963RA_H_

extern float GyroOffset_Xdata;
extern float GyroOffset_Ydata;
extern float GyroOffset_Zdata;
extern float GyroOffset_Xdata, icm_data_acc_x, icm_data_gyro_x;
extern float GyroOffset_Ydata, icm_data_acc_y, icm_data_gyro_y;
extern float GyroOffset_Zdata, icm_data_acc_z, icm_data_gyro_z;
extern float Q_info_q0, Q_info_q1, Q_info_q2, Q_info_q3;
extern uint8 InitFlag;
extern float Gyro_Yaw;
extern float Yaw;
extern float IMU_Daty_Z;
extern float GPS_Daty_Z;
extern float GPS_IMU_Daty_Z;
extern float Accle_Angle;

extern void IMU_YAW_integral(void);
extern void gyroOffset_init(void);
extern void ICM_getEulerianAngles(void);
extern float eulerAngle_yaw, eulerAngle_pitch, eulerAngle_roll, eulerAngle_yaw_total;


#endif /* CODE_IMU_H_ */
