#include "kart_imu.h"
#include "kart_calc.h"
#include "zf_device_imu963ra.h"
#include "math.h"

/*
 * 卡丁车航向解算 —— 实现
 * ------------------------------------------------------------------
 * 移植自 TopSpeed IMU.c(USE_6DOF_AHRS + USE_IMU963RA)。
 * Madgwick 主体、四元数积分、963RA 读数换算全部一字不改照搬,
 * 只把 GBK 乱码注释换成干净中文,并砍掉:
 *   - IMU_running_CALLBACK 里的 Subject 发射方向偏置、撞击/自由落体检测
 *   - IMU_check 里的 Flash 读写、IPS 屏显示
 * ------------------------------------------------------------------
 */

/* ===== Madgwick 算法参数(照搬 TopSpeed)===== */
#define sampleFreq  200.0f      // 采样频率 200Hz(对应 5ms 一帧)
#define betaDef     0.01f       // 2*比例增益(Madgwick 收敛系数,越大越信加速度计)

/* volatile:告诉编译器"这变量随时可能被中断改",别把它优化进寄存器缓存。
 * 因为 kart_imu_update 在定时中断里跑,这几个四元数会在中断里被改。 */
static volatile float beta = betaDef;                       // Madgwick 增益
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 姿态四元数,初值=单位四元数(不旋转)

static sSensorData IMU_data = {0};      // 当前帧传感器数据(内部缓存)

IMU_Handle_struct IMU_Handle = {0};     // IMU 总句柄(对外)

/* =========================== 读一帧原始数据 + 去零偏 + 换算 =========================== */
/* 移植自 IMU.c 的 USE_IMU963RA 分支 get_IMU_RAW()
 * data->Gyro* 出去是 rad/s,data->Acc* 出去是 g。
 * 换算系数照搬:陀螺 /14.3(2000dps 量程 LSB→°/s)再 *0.01745(°→rad);
 *              加速度 /4098(8G 量程 LSB→g)。
 * 指针参数 data:传进来的是"地址",函数里用 -> 直接改调用者的那块内存(相当于返回多个值)。*/
void get_IMU_RAW(IMU_data_RAW_struct *data){
    imu963ra_get_gyro();    // 读陀螺,结果进全局 imu963ra_gyro_x/y/z
    data->GyroX=((float)imu963ra_gyro_x-IMU_Handle.IMU_CaliData.GYRO_X_bias)/14.3f*0.01745329252f;
    data->GyroY=((float)imu963ra_gyro_y-IMU_Handle.IMU_CaliData.GYRO_Y_bias)/14.3f*0.01745329252f;
    data->GyroZ=((float)imu963ra_gyro_z-IMU_Handle.IMU_CaliData.GYRO_Z_bias)/14.3f*0.01745329252f;

    imu963ra_get_acc();     // 读加速度,结果进全局 imu963ra_acc_x/y/z
    data->AccX=imu963ra_acc_x/(float)4098;
    data->AccY=imu963ra_acc_y/(float)4098;
    data->AccZ=imu963ra_acc_z/(float)4098;

    /* 顺手把去零偏后的量存进句柄(单位:陀螺 °/s,加速度 m/s^2),给调试/以后航位推算用 */
    IMU_Handle.RAW_data.GyroX=((float)imu963ra_gyro_x-IMU_Handle.IMU_CaliData.GYRO_X_bias)/14.3f;
    IMU_Handle.RAW_data.GyroY=((float)imu963ra_gyro_y-IMU_Handle.IMU_CaliData.GYRO_Y_bias)/14.3f;
    IMU_Handle.RAW_data.GyroZ=((float)imu963ra_gyro_z-IMU_Handle.IMU_CaliData.GYRO_Z_bias)/14.3f;

    IMU_Handle.RAW_data.AccX=imu963ra_acc_x*0.00239141;     // 0.00239141 = 9.8/4098,直接换成 m/s^2
    IMU_Handle.RAW_data.AccY=imu963ra_acc_y*0.00239141;
    IMU_Handle.RAW_data.AccZ=imu963ra_acc_z*0.00239141;
}

/* =========================== 读一帧,填进算法输入结构 =========================== */
/* 移植自 IMU.c read_IMU()
 * 注:TopSpeed 这里还有个 IMU_data_filter_update() 均值滤波,源码没随这批一起搬,
 *     先不接(滤波只是平滑,不影响能否解算)。以后要更稳可以补上。 */
void read_IMU(sSensorData *sd){
    IMU_data_RAW_struct data={0};
    get_IMU_RAW(&data);         // 取地址传进去,让它填 data

    /* IMU_data_filter_update(&data);   // 均值滤波(暂缺,先直通) */

    sd->mGyroX=data.GyroX;
    sd->mGyroY=data.GyroY;
    sd->mGyroZ=data.GyroZ;
    sd->mAccX=data.AccX;
    sd->mAccY=data.AccY;
    sd->mAccZ=data.AccZ;
}

/* =========================== Madgwick 6DOF 姿态更新 =========================== */
/* 移植自 IMU.c MadgwickAHRSupdateIMU()——核心算法,一字不改照搬。
 * 干的事:用陀螺积分预测四元数变化率,再用加速度计(重力方向)做梯度下降修正,
 *         最后积分 + 归一化,得到这一帧的姿态四元数。跑一次约 5us。 */
void MadgwickAHRSupdateIMU(sSensorData *sd) {
    float gx = sd->mGyroX;
    float gy = sd->mGyroY;
    float gz = sd->mGyroZ;

    float ax = sd->mAccX;
    float ay = sd->mAccY;
    float az = sd->mAccZ;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // 四元数微分方程(陀螺积分部分)
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 加速度计数据有效才做反馈修正(全 0 说明没读到,跳过)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 预先算好一堆重复用到的量(省乘法)
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 梯度下降算法修正方向
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // 归一化修正量
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用反馈修正
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 积分得到新的四元数
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    sd->attitude.a=q0;
    sd->attitude.b=q1;
    sd->attitude.c=q2;
    sd->attitude.d=q3;
}

/* =========================== 姿态复位 =========================== */
/* 移植自 IMU.c reset_attitude():把四元数拉回单位四元数,相当于把当前航向清成 0 */
void reset_attitude(void){
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    IMU_Handle.Attitude=quaternionToEuler(IMU_data.attitude);
}

/* =========================== 陀螺零偏静止标定 =========================== */
/* 移植自 IMU.c 的 USE_IMU963RA 分支 IMU_check()
 * 车放稳别动,连采 100 次(每次隔 5ms)陀螺读数求平均 = 零偏。
 * 砍掉了原工程的 Flash 读写和 IPS 屏显示,只把结果留在内存。
 * 静止 1s 是为了让传感器上电稳定后再采。 */
void IMU_check(void){
    IMU_CaliData_Struct IMU_CaliData_s={0};
    IMU_Handle.FLAG_enable_running_CALLBACK=0;      // 标定期间禁止 update 抢读数据
    system_delay_ms(1000);                          // 静止等 1s 让传感器稳定
    reset_attitude();
    for(int i=0;i<100;i++){
        imu963ra_get_gyro();
        IMU_CaliData_s.GYRO_X_bias+=imu963ra_gyro_x;
        IMU_CaliData_s.GYRO_Y_bias+=imu963ra_gyro_y;
        IMU_CaliData_s.GYRO_Z_bias+=imu963ra_gyro_z;
        system_delay_ms(5);
    }
    IMU_CaliData_s.GYRO_X_bias/=100.0f;
    IMU_CaliData_s.GYRO_Y_bias/=100.0f;
    IMU_CaliData_s.GYRO_Z_bias/=100.0f;

    IMU_Handle.IMU_CaliData=IMU_CaliData_s;         // 存进句柄(不写 Flash)
    IMU_Handle.FLAG_enable_running_CALLBACK=1;      // 标定完,放行 update
}

/* =========================== 初始化 =========================== */
/* 移植自 IMU.c 的 USE_IMU963RA 分支 IMU_init()
 * 顺序:复位姿态 → 初始化 963RA(内部按 zf_device_imu963ra.h 里的宏配 SPI_0/引脚)
 *       → 标定零偏 → 放行 update。
 * 注:IMU_data_filter_init() 因滤波器暂缺,先不调。 */
void kart_imu_init(void){
    reset_attitude();
    /* IMU_data_filter_init();   // 滤波器初始化(暂缺) */
    imu963ra_init();
    IMU_check();                 // 上电静止标定(车必须放稳)
    IMU_Handle.FLAG_enable_running_CALLBACK=1;
}

/* =========================== 周期更新(放 5ms 中断)=========================== */
/* 精简自 IMU.c IMU_running_CALLBACK():
 *   读数 → Madgwick 解算 → 四元数转欧拉角 → yaw 归一化到 [-180,180]。
 * 砍掉:Subject 发射方向偏置、撞击检测、自由落体检测。 */
void kart_imu_update(void){
    if(IMU_Handle.FLAG_enable_running_CALLBACK){
        read_IMU(&IMU_data);                                    // 读一帧
        MadgwickAHRSupdateIMU(&IMU_data);                       // 解算
        EulerAngles Attitude=quaternionToEuler(IMU_data.attitude);  // 四元数→欧拉角
        IMU_Handle.Attitude.roll  = Attitude.roll;
        IMU_Handle.Attitude.pitch = Attitude.pitch;
        IMU_Handle.Attitude.yaw   = Attitude.yaw;               // 纯航向,暂不加发射方向偏置

        // 航向角归一化到 [-180, 180]
        if(IMU_Handle.Attitude.yaw>180){
            IMU_Handle.Attitude.yaw-=360;
        }
        else if(IMU_Handle.Attitude.yaw<=-180){
            IMU_Handle.Attitude.yaw+=360;
        }
    }
}

/* =========================== 取航向 =========================== */
float kart_imu_get_yaw(void){
    return IMU_Handle.Attitude.yaw;
}
