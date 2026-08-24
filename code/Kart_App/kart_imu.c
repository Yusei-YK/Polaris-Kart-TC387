#include "kart_imu.h"
#include "kart_calc.h"
#include "zf_device_imu660ra.h"
#include "math.h"

#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#endif

/*
 * 卡丁车航向解算 —— 实现
 * ------------------------------------------------------------------
 * Madgwick 主体和四元数积分参考 TopSpeed 6DOF 实现；
 * 传感器读取与换算按本车 IMU660RA 适配，并砍掉:
 *   - IMU_running_CALLBACK 里的 Subject 发射方向偏置、撞击/自由落体检测
 *   - IMU_check 里的 Flash 读写、IPS 屏显示
 * ------------------------------------------------------------------
 */

/* ===== Madgwick 算法参数(照搬 TopSpeed)===== */
#define sampleFreq  200.0f      // 采样频率 200Hz(对应 5ms 一帧)
#define betaDef     0.01f       // 2*比例增益(Madgwick 收敛系数,越大越信加速度计)
#define KART_IMU_CALIB_SAMPLES  (1000)
/* 标定静止判据:采样期间任一陀螺轴极差(max-min)超此阈值即判定晃动,重标。
 * 单位=陀螺原始 LSB(16.4 LSB/°每秒),300 ≈ 18°/s 峰峰,足够区分"人碰了车"与静止噪声。 */
#define KART_IMU_CALIB_MOVE_THRESH  (300.0f)
/* 晃动重标最多重试次数,超了就用最后一次结果放行,避免上电永久卡死。 */
#define KART_IMU_CALIB_MAX_RETRY    (5)

/* volatile:告诉编译器"这变量随时可能被中断改",别把它优化进寄存器缓存。
 * 因为 kart_imu_update 在定时中断里跑,这几个四元数会在中断里被改。 */
static volatile float beta = betaDef;                       // Madgwick 增益
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 姿态四元数,初值=单位四元数(不旋转)

static sSensorData IMU_data = {0};      // 当前帧传感器数据(内部缓存)

IMU_Handle_struct IMU_Handle = {0};     // IMU 总句柄(对外)

/* 真实积分步长 dt(秒):用 system_getval_us() 相邻两拍差分算出,
 * 替换硬编码 1/sampleFreq。中断抖动时四元数积分才不会偏。
 * 首帧和异常值(<=0 或 >20ms)钳到名义 5ms,避免上电/丢拍炸掉积分。 */
#define KART_IMU_DT_NOMINAL   (1.0f / sampleFreq)   // 名义步长 5ms
#define KART_IMU_DT_MAX       (0.020f)              // dt 上限 20ms(丢拍保护)
static volatile float g_imu_dt = KART_IMU_DT_NOMINAL;   // 当前帧实际步长(秒)

/* =========================== 二阶巴特沃斯低通滤波(照搬 NEUQ GET_ANGLE.c)===========================
 * 送进 Madgwick 前先把陀螺/加速度过一遍二阶巴特沃斯,压掉高频抖动。
 * 截止频率:陀螺 7.5Hz、加速度 35Hz,采样 200Hz —— 均取自 NEUQ 已验证方案。
 * 直接 II 型双二阶实现,首帧用当前值填满缓冲避免上电冲激。 */
#define KART_IMU_GYRO_CUTOFF_HZ (7.5f)
#define KART_IMU_ACC_CUTOFF_HZ  (35.0f)
#define KART_IMU_PI_F           (3.14159265358979f)

typedef struct{
    float in[3];        // 输入历史 x(n-2),x(n-1),x(n)
    float out[3];       // 输出历史 y(n-2),y(n-1),y(n)
}kart_butter_buf_t;

typedef struct{
    float a[3];         // 分母系数 a0,a1,a2
    float b[3];         // 分子系数 b0,b1,b2
}kart_butter_param_t;

static kart_butter_param_t g_gyro_butter = {0};
static kart_butter_param_t g_acc_butter  = {0};
static kart_butter_buf_t   g_gyro_buf[3] = {0};     // 陀螺 X/Y/Z 各一个缓冲
static kart_butter_buf_t   g_acc_buf[3]  = {0};     // 加速度 X/Y/Z 各一个缓冲

/* 由采样率与截止频率算二阶巴特沃斯系数(照搬 NEUQ Set_Cutoff_Frequency) */
static void kart_butter_set_cutoff(float sample_hz, float cutoff_hz, kart_butter_param_t *p){
    if(cutoff_hz <= 0.0f) return;
    float fr  = sample_hz / cutoff_hz;
    float ohm = tanf(KART_IMU_PI_F / fr);
    float c   = 1.0f + 2.0f * cosf(KART_IMU_PI_F / 4.0f) * ohm + ohm * ohm;
    p->b[0] = ohm * ohm / c;
    p->b[1] = 2.0f * p->b[0];
    p->b[2] = p->b[0];
    p->a[0] = 1.0f;
    p->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
    p->a[2] = (1.0f - 2.0f * cosf(KART_IMU_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

/* 二阶巴特沃斯滤波一次(照搬 NEUQ LPButterworth,含首帧填充与 NaN 复位保护) */
static float kart_butter_filter(float input, kart_butter_buf_t *buf, kart_butter_param_t *p){
    if(buf->out[0]==0.0f && buf->out[1]==0.0f && buf->out[2]==0.0f &&
       buf->in[0]==0.0f  && buf->in[1]==0.0f  && buf->in[2]==0.0f){
        buf->out[0]=input; buf->out[1]=input; buf->out[2]=input;
        buf->in[0]=input;  buf->in[1]=input;  buf->in[2]=input;
        return input;
    }

    buf->in[2] = input;
    buf->out[2] = p->b[0]*buf->in[2] + p->b[1]*buf->in[1] + p->b[2]*buf->in[0]
                - p->a[1]*buf->out[1] - p->a[2]*buf->out[0];

    buf->in[0]  = buf->in[1];
    buf->in[1]  = buf->in[2];
    buf->out[0] = buf->out[1];
    buf->out[1] = buf->out[2];

    for(uint8 i=0;i<3;i++){
        if(isnan(buf->out[i]) || isnan(buf->in[i])){
            buf->out[0]=input; buf->out[1]=input; buf->out[2]=input;
            buf->in[0]=input;  buf->in[1]=input;  buf->in[2]=input;
            return input;
        }
    }
    return buf->out[2];
}

/* =========================== 读一帧原始数据 + 去零偏 + 换算 =========================== */
/* 参考 TopSpeed get_IMU_RAW()，传感器读取改为 IMU660RA。
 * data->Gyro* 出去是 rad/s,data->Acc* 出去是 g。
 * 换算系数:陀螺 /16.4(2000dps 量程 LSB→°/s)再 *0.01745(°→rad);
 *              加速度 /4098(8G 量程 LSB→g)。
 * 指针参数 data:传进来的是"地址",函数里用 -> 直接改调用者的那块内存(相当于返回多个值)。*/
void get_IMU_RAW(IMU_data_RAW_struct *data){
    imu660ra_get_gyro();
    imu660ra_get_acc();

    /* 物理安装：X轴朝下(地面)，Y轴朝左，Z轴朝前(车头)
     * Madgwick 期望坐标系：X→前，Y→左，Z→上
     * 映射：algo_X = phys_Z(前)，algo_Y = phys_Y(左)，algo_Z = -phys_X(上=-下) */
    float gx = ((float)imu660ra_gyro_x - IMU_Handle.IMU_CaliData.GYRO_X_bias) / 16.4f * 0.01745329252f;
    float gy = ((float)imu660ra_gyro_y - IMU_Handle.IMU_CaliData.GYRO_Y_bias) / 16.4f * 0.01745329252f;
    float gz = ((float)imu660ra_gyro_z - IMU_Handle.IMU_CaliData.GYRO_Z_bias) / 16.4f * 0.01745329252f;

    /* 去掉硬死区:零偏标定 + 二阶巴特沃斯已压噪声,硬死区会吃掉慢速转向角速度导致漂移。
     * (NEUQ / mahney / AIR 三套参考方案均不用硬死区) */
    data->GyroX =  gz;
    data->GyroY =  gy;
    data->GyroZ = -gx;

    float ax = imu660ra_acc_x / 4098.0f;
    float ay = imu660ra_acc_y / 4098.0f;
    float az = imu660ra_acc_z / 4098.0f;

    data->AccX =  az;
    data->AccY =  ay;
    data->AccZ = -ax;

    IMU_Handle.RAW_data.GyroX = ((float)imu660ra_gyro_x - IMU_Handle.IMU_CaliData.GYRO_X_bias) / 16.4f;
    IMU_Handle.RAW_data.GyroY = ((float)imu660ra_gyro_y - IMU_Handle.IMU_CaliData.GYRO_Y_bias) / 16.4f;
    IMU_Handle.RAW_data.GyroZ = ((float)imu660ra_gyro_z - IMU_Handle.IMU_CaliData.GYRO_Z_bias) / 16.4f;

    IMU_Handle.RAW_data.AccX = imu660ra_acc_x * 0.00239141f;
    IMU_Handle.RAW_data.AccY = imu660ra_acc_y * 0.00239141f;
    IMU_Handle.RAW_data.AccZ = imu660ra_acc_z * 0.00239141f;
}

/* =========================== 读一帧,填进算法输入结构 =========================== */
/* 移植自 IMU.c read_IMU()
 * 均值滤波换成二阶巴特沃斯(照搬 NEUQ),陀螺/加速度各自截止频率不同。 */
void read_IMU(sSensorData *sd){
    IMU_data_RAW_struct data={0};
    get_IMU_RAW(&data);         // 取地址传进去,让它填 data

    /* 二阶巴特沃斯滤波:陀螺 7.5Hz、加速度 35Hz(照搬 NEUQ 已验证方案) */
    sd->mGyroX=kart_butter_filter(data.GyroX, &g_gyro_buf[0], &g_gyro_butter);
    sd->mGyroY=kart_butter_filter(data.GyroY, &g_gyro_buf[1], &g_gyro_butter);
    sd->mGyroZ=kart_butter_filter(data.GyroZ, &g_gyro_buf[2], &g_gyro_butter);
    sd->mAccX=kart_butter_filter(data.AccX, &g_acc_buf[0], &g_acc_butter);
    sd->mAccY=kart_butter_filter(data.AccY, &g_acc_buf[1], &g_acc_butter);
    sd->mAccZ=kart_butter_filter(data.AccZ, &g_acc_buf[2], &g_acc_butter);
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

    // 积分得到新的四元数(用真实步长 dt,替代硬编码 1/sampleFreq)
    q0 += qDot1 * g_imu_dt;
    q1 += qDot2 * g_imu_dt;
    q2 += qDot3 * g_imu_dt;
    q3 += qDot4 * g_imu_dt;

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    /* 有限值保护:任一分量变 NaN/Inf(异常输入/除零)会让四元数永久污染,
     * 之后每拍都是 NaN,航向环彻底失效。检测到就复位单位四元数,丢一帧姿态
     * 也比永久卡死强。 */
    if(isnan(q0) || isnan(q1) || isnan(q2) || isnan(q3) ||
       isinf(q0) || isinf(q1) || isinf(q2) || isinf(q3))
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }

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

/* =========================== 强制设定航向 =========================== */
/* 科目三反向复现用:人搬车掉头 180° 后 IMU 已乱,按键时用路径几何重建航向基准。
 * 保持当前 roll/pitch 不变,只改 yaw → 从欧拉角转回四元数覆写 q0~q3。
 * 公式:从 ZYX 欧拉角 (roll,pitch,yaw) 转四元数(标准 3-2-1 序列):
 *   cy=cos(yaw/2), sy=sin(yaw/2)
 *   cp=cos(pitch/2), sp=sin(pitch/2)
 *   cr=cos(roll/2), sr=sin(roll/2)
 *   q0 = cr*cp*cy + sr*sp*sy
 *   q1 = sr*cp*cy - cr*sp*sy
 *   q2 = cr*sp*cy + sr*cp*sy
 *   q3 = cr*cp*sy - sr*sp*cy
 */
void kart_imu_set_yaw(float target_yaw)
{
    EulerAngles euler;
    float cy, sy, cp, sp, cr, sr;
    float yaw_rad, pitch_rad, roll_rad;

    /* 读当前姿态,保留 roll/pitch */
    euler = quaternionToEuler(IMU_data.attitude);

    /* 用目标 yaw 替换 */
    roll_rad  = degree_to_rad(euler.roll);
    pitch_rad = degree_to_rad(euler.pitch);
    yaw_rad   = degree_to_rad(target_yaw);

    /* 欧拉角 → 四元数 */
    cy = cosf(yaw_rad * 0.5f);
    sy = sinf(yaw_rad * 0.5f);
    cp = cosf(pitch_rad * 0.5f);
    sp = sinf(pitch_rad * 0.5f);
    cr = cosf(roll_rad * 0.5f);
    sr = sinf(roll_rad * 0.5f);

    q0 = cr*cp*cy + sr*sp*sy;
    q1 = sr*cp*cy - cr*sp*sy;
    q2 = cr*sp*cy + sr*cp*sy;
    q3 = cr*cp*sy - sr*sp*cy;

    /* 更新句柄 */
    IMU_Handle.Attitude.yaw = target_yaw;
}

/* =========================== 标定期间显示刷新钩子 =========================== */
/* IMU_check 的阻塞 delay 期间(约 6s)周期调用它维持点阵扫描(全亮"标定中"指示)。
 * 默认 NULL 不影响原行为;由 cpu0_main 注册 dot_matrix_screen_scan。 */
static void (*kart_imu_calib_hook)(void) = 0;

void kart_imu_set_calib_hook(void (*hook)(void))
{
    kart_imu_calib_hook = hook;
}

/* 带刷新的毫秒延时:每 1ms 调一次钩子(点阵扫描)维持多路复用刷新率,总计 ms 毫秒。
 * 钩子为空时退化成普通 system_delay_ms,行为与原来一致。 */
static void kart_imu_calib_delay_ms(uint32 ms)
{
    uint32 i;
    if(kart_imu_calib_hook == 0)
    {
        system_delay_ms(ms);
        return;
    }
    for(i = 0; i < ms; i++)
    {
        kart_imu_calib_hook();
        system_delay_ms(1);
    }
}

/* =========================== 陀螺零偏静止标定 =========================== */
/* 参考 TopSpeed IMU_check()。
 * 车放稳别动,连采 KART_IMU_CALIB_SAMPLES 次(每次隔 5ms)陀螺读数求平均 = 零偏。
 * 砍掉了原工程的 Flash 读写和 IPS 屏显示,只把结果留在内存。
 * 静止 1s 是为了让传感器上电稳定后再采。 */
void IMU_check(void){
    IMU_CaliData_Struct IMU_CaliData_s={0};
    uint8 retry;

    IMU_Handle.FLAG_enable_running_CALLBACK=0;      // 标定期间禁止 update 抢读数据
    kart_imu_calib_delay_ms(1000);                  // 静止等 1s 让传感器稳定(期间刷点阵)
    reset_attitude();

    /* 静止检测:采样期间跟踪每轴极差(max-min),超阈值判定被碰、重标。
     * 盲采求平均对晃动毫无免疫——晃一下就把角速度当零偏吃进去,航向从此漂。
     * 最多重试 KART_IMU_CALIB_MAX_RETRY 次,防上电永久卡死。 */
    for(retry=0; retry<KART_IMU_CALIB_MAX_RETRY; retry++){
        float gx_min, gx_max, gy_min, gy_max, gz_min, gz_max;
        uint8 moved = 0;

        IMU_CaliData_s.GYRO_X_bias = 0.0f;
        IMU_CaliData_s.GYRO_Y_bias = 0.0f;
        IMU_CaliData_s.GYRO_Z_bias = 0.0f;

        imu660ra_get_gyro();
        gx_min = gx_max = (float)imu660ra_gyro_x;
        gy_min = gy_max = (float)imu660ra_gyro_y;
        gz_min = gz_max = (float)imu660ra_gyro_z;

        for(int i=0;i<KART_IMU_CALIB_SAMPLES;i++){
            float gx, gy, gz;
            imu660ra_get_gyro();
            gx = (float)imu660ra_gyro_x;
            gy = (float)imu660ra_gyro_y;
            gz = (float)imu660ra_gyro_z;

            IMU_CaliData_s.GYRO_X_bias+=gx;
            IMU_CaliData_s.GYRO_Y_bias+=gy;
            IMU_CaliData_s.GYRO_Z_bias+=gz;

            if(gx<gx_min) gx_min=gx; if(gx>gx_max) gx_max=gx;
            if(gy<gy_min) gy_min=gy; if(gy>gy_max) gy_max=gy;
            if(gz<gz_min) gz_min=gz; if(gz>gz_max) gz_max=gz;

            if((gx_max-gx_min) > KART_IMU_CALIB_MOVE_THRESH ||
               (gy_max-gy_min) > KART_IMU_CALIB_MOVE_THRESH ||
               (gz_max-gz_min) > KART_IMU_CALIB_MOVE_THRESH){
                moved = 1;
                break;                              // 晃动:立即中断本轮,重标
            }
            kart_imu_calib_delay_ms(5);             // 采样间隔(期间刷点阵)
        }

        if(!moved){
            IMU_CaliData_s.GYRO_X_bias/=(float)KART_IMU_CALIB_SAMPLES;
            IMU_CaliData_s.GYRO_Y_bias/=(float)KART_IMU_CALIB_SAMPLES;
            IMU_CaliData_s.GYRO_Z_bias/=(float)KART_IMU_CALIB_SAMPLES;
            break;                                  // 全程静止:本轮结果有效
        }
        kart_imu_calib_delay_ms(500);               // 晃动:等半秒让车重新稳下来再重标(期间刷点阵)
    }

    IMU_Handle.IMU_CaliData=IMU_CaliData_s;         // 存进句柄(不写 Flash)
    IMU_Handle.FLAG_enable_running_CALLBACK=1;      // 标定完,放行 update
}

/* =========================== 初始化 =========================== */
/* 参考 TopSpeed IMU_init()，硬件使用 IMU660RA。
 * 顺序:复位姿态 → 初始化 660RA(内部按 zf_device_imu660ra.h 里的宏配 SPI_0/引脚)
 *       → 标定零偏 → 放行 update。
 * 注:先算好二阶巴特沃斯系数,再初始化传感器。 */
uint8 kart_imu_init(void){
    uint8 retry;

    reset_attitude();
    IMU_Handle.FLAG_enable_running_CALLBACK = 0;
    /* 二阶巴特沃斯系数按 200Hz 采样初始化(陀螺 7.5Hz、加速度 35Hz) */
    kart_butter_set_cutoff(sampleFreq, KART_IMU_GYRO_CUTOFF_HZ, &g_gyro_butter);
    kart_butter_set_cutoff(sampleFreq, KART_IMU_ACC_CUTOFF_HZ,  &g_acc_butter);
    for(retry = 0; retry < 3U; retry++)
    {
        if(0U == imu660ra_init())
        {
            break;
        }
        system_delay_ms(20);
    }
    if(retry >= 3U)
    {
        return 0;
    }
    IMU_check();                 // 上电静止标定(车必须放稳)
    IMU_Handle.FLAG_enable_running_CALLBACK=1;
    return 1;
}

uint8 kart_imu_is_ready(void)
{
    return IMU_Handle.FLAG_enable_running_CALLBACK ? 1U : 0U;
}

/* =========================== 周期更新(放 5ms 中断)=========================== */
/* 精简自 IMU.c IMU_running_CALLBACK():
 *   读数 → Madgwick 解算 → 四元数转欧拉角 → yaw 归一化到 [-180,180]。
 * 砍掉:Subject 发射方向偏置、撞击检测、自由落体检测。 */

/* yaw 滑动平均滤波缓冲区 */
#define YAW_FILTER_SIZE  5
static float yaw_filter_buf[YAW_FILTER_SIZE] = {0};
static uint8 yaw_filter_idx = 0;

void kart_imu_update(void){
    if(IMU_Handle.FLAG_enable_running_CALLBACK){
        /* 真实步长 dt:取相邻两拍 system_getval_us() 之差(微秒→秒)。
         * uint32 差分天然处理回绕;首帧和异常(<=0 或 >20ms)钳到名义 5ms。 */
        static uint8  dt_first = 1;
        static uint32 last_us  = 0;
        uint32 now_us = system_getval_us();
        if(dt_first){
            g_imu_dt = KART_IMU_DT_NOMINAL;
            dt_first = 0;
        }else{
            float dt = (float)(now_us - last_us) * 1e-6f;
            if(dt <= 0.0f || dt > KART_IMU_DT_MAX) dt = KART_IMU_DT_NOMINAL;
            g_imu_dt = dt;
        }
        last_us = now_us;

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

        // 更新滑动平均滤波缓冲区
        yaw_filter_buf[yaw_filter_idx] = IMU_Handle.Attitude.yaw;
        yaw_filter_idx = (yaw_filter_idx + 1) % YAW_FILTER_SIZE;
    }
}

/* =========================== 取航向 =========================== */
float kart_imu_get_yaw(void){
    return IMU_Handle.Attitude.yaw;
}

/* 取滤波后航向（5拍滑动平均，延迟25ms，抑制抖动）*/
float kart_imu_get_yaw_filtered(void){
    float sum = 0;
    for(uint8 i = 0; i < YAW_FILTER_SIZE; i++){
        sum += yaw_filter_buf[i];
    }
    return sum / (float)YAW_FILTER_SIZE;
}

/* 车辆Yaw轴映射为物理X轴的负方向；这些接口仅供日志读取。 */
float kart_imu_get_yaw_rate_dps(void){
    return -IMU_Handle.RAW_data.GyroX;
}

float kart_imu_get_yaw_bias_dps(void){
    return -IMU_Handle.IMU_CaliData.GYRO_X_bias / 16.4f;
}

float kart_imu_get_acc_norm_g(void){
    float ax = IMU_Handle.RAW_data.AccX;
    float ay = IMU_Handle.RAW_data.AccY;
    float az = IMU_Handle.RAW_data.AccZ;
    return sqrtf(ax * ax + ay * ay + az * az) / 9.80665f;
}

/* 本帧实际积分步长(微秒):5ms 中断稳定则应恒在 5000 附近。
 * g_imu_dt 单位秒,乘 1e6 转微秒供 VOFA/日志观察抖动。 */
float kart_imu_get_dt_us(void){
    return g_imu_dt * 1e6f;
}

#if defined(__TASKING__)
#pragma section all restore
#endif
