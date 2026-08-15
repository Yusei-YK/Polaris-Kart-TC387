#ifndef KART_STEER_CTRL_H_
#define KART_STEER_CTRL_H_

#include "zf_common_headfile.h"
#include "kart_calib.h"     /* 软限位 KART_STEER_DELTA_LIMIT_L/R、KART_STEER_LOOP_SIGN */
#include "kart_pid.h"

/*
 * 卡丁车转向控制(串级:航向外环 → 转角内环)
 * ------------------------------------------------------------------
 * 硬件事实:转向是【有刷电机 + SPI 绝对编码器】,不是自动回中的 RC 舵机。
 *   → 有刷电机断电不回中,必须靠"转角位置环"把它按在目标角度上。
 *   → 这是刚需,不是可选项。
 *
 * 架构(两级串级):
 *   外环(航向环 heading):  目标航向 − IMU 实测 yaw(wrap±180) → PID → 目标转角
 *   内环(转角环 angle):    目标转角 − 绝对编码器 center_delta → PID → 转向电机 duty
 *
 * 内环是地基:先只跑内环(手给目标转角,验证能不能稳定按住方向盘),
 *   稳了再套外环。外环关掉时,内环目标转角由串口 a<val> 直接给,方便单测。
 *
 * 调用节拍:必须放【主循环】,紧跟 kart_steer_abs_update() 之后 ——
 *   因为 center_delta 靠 SPI 读、只在主循环刷新,放 5ms 中断会吃到上电那帧的死数据。
 * ------------------------------------------------------------------
 * 量纲:
 *   转角误差单位 = 绝对编码器 raw 计数(4096/圈,约 0.088°/计数)
 *   center_delta 范围见 kart_calib.h 第四节(当前软限 ±1064)
 *   航向误差单位 = 度(IMU yaw ∈ −180~180)
 * ------------------------------------------------------------------
 */

/* 软限位 KART_STEER_DELTA_LIMIT_L/R 与 KART_STEER_LOOP_SIGN → kart_calib.h。
 * 软限位由三个硬限位 raw 派生:换齿轮/换转向编码器只重标那三个 raw,
 * 这里、外环上限、满舵半径会自动跟随,不必逐处手改(以前就是这样对不上的)。 */

/* 内环 PID 默认(悬空验证已冻结)。
 * out_max 先给 4000(不放满 10000),限幅防打飞,上车可再放。
 * 冻结依据:悬空 sa0→I6=3、sa200→I6=203,静差 ~3 计数(≈0.26°),si0.5 顶过静摩擦。
 * Kd=0:阶跃尖峰来自 Kp 比例冲击,加 D 只会更大(D 对阶跃正向踢),故保持 0。 */
#define KART_STEER_KP_DEFAULT           (15.0f)
#define KART_STEER_KI_DEFAULT           (0.5f)
#define KART_STEER_KD_DEFAULT           (0.0f)
#define KART_STEER_IMAX_DEFAULT         (2000.0f)
/* 2026-07-28 赛前:4000 → 6000(40% → 60%),就是上面注释里说的"上车可再放"。
 * 为什么现在必须放:提速后转向速率成了新瓶颈。实测转向电机约 1800 计数/s,
 * 中位→满舵 1103 计数需 0.61s;而 60 脉冲(4.4 m/s)下 1.50m 前视只给 0.34s 预判
 * —— 打角跟不上目标,表现为高速切内/走线滞后。放大输出上限直接提高可用角速度。
 * 为什么不放满 10000:静摩擦死区实测约 950 duty,Kp=15 意味着 6000 对应
 * 400 计数(9.1°)的误差就已饱和 —— 正常跟踪误差远小于此,6000 已经够用;
 * 留 40% 余量是防"坏参考点导致一拍大误差"时把方向盘怼上软限位。 */
#define KART_STEER_OUTMAX_DEFAULT       (6000.0f)

/* 内环 PID —— 倒车专用一组(借 TopSpeed Subject_4 的做法)。
 * 为什么倒车要换增益:轮胎侧偏力在前进时是"把前轮往中位推"(转角环要顶着它),
 *   倒车时同一个力变成"往打死方向推"(自增强,转角环要拽住它)。
 *   前进标定的 Kp 到倒车就偏大 → 过冲、来回振、甚至怼软限位。
 *   对策:比例砍小 + 阻尼拉大(TopSpeed 前进 Kp300/Kd150 → 倒车 Kp100/Kd500)。
 * 只在【真要打角的倒车】起作用:直行倒车锁中位其实无所谓,
 *   但"蛇形后退十米"要真打角,不换增益会振。
 * 【待实车标定】先按前进组的 1/3 比例给,上车看 VOFA 转角波形有无过冲再调。 */
#define KART_STEER_KP_BACK              (5.0f)      /* 前进 15 的 1/3 */
#define KART_STEER_KI_BACK              (0.5f)      /* 与前进同:顶静摩擦用,不动 */
#define KART_STEER_KD_BACK              (2.5f)      /* 前进 0 → 加阻尼压振 */

/* 航向外环 PID 默认。输出是"目标转角计数",限幅到转角软限位量级。
 * 【2026-07-28 赛前复核:30 已验证,不是占位,不动】原注释写"先占位"已作废。
 * 三条独立证据:① 实车科目一/科目三走线已跑通;② 30 计数/度对应内环死区
 * 63 计数 → 0.83° 以下的航向误差不动方向盘,与 kart_motion.h 独立推导的
 * 25/30 量级一致;③ 与轴距 0.62m 算出的 0.86m 航向收敛特征长度自洽。
 * 曾经的坑:一度以为要乘 57.3(deg/rad),那是错的,已在 CLAUDE.md 记录撤回。 */
#define KART_HEAD_KP_DEFAULT            (30.0f)
#define KART_HEAD_KI_DEFAULT            (0.0f)
#define KART_HEAD_KD_DEFAULT            (0.0f)
#define KART_HEAD_IMAX_DEFAULT          (500.0f)
/* 2026-07-28:1000 → 1133(= 左软限)。日志证据:科目一复刻前进段 493 帧里
 * 304 帧(62%)的 target_delta 恰好等于 ±1000.00,而同一路线录制时人手打到了
 * +1103/−985 —— 外环上限比软限位小,等于把绕桩需要的最后 100 计数(2.3°)削掉了,
 * 表现为复刻切内侧锥桶。改成与软限位同值:限角只由软限位一处决定,不再有第二道暗闸。 */
/* 2026-07-30:795 → 1064(跟着软限位走)。外环上限必须 ≤ 软限位。
 * 改为直接引用软限位:重标硬限位 raw 后自动跟随,不会再出现"外环上限
 * 比软限位小、暗中削掉最后一点打角"的情况。 */
#define KART_HEAD_OUTMAX_DEFAULT        ((float)KART_STEER_DELTA_LIMIT_L)

typedef struct
{
    kart_pid_t  angle_pid;          // 内环:转角位置 PID
    kart_pid_t  head_pid;           // 外环:航向 PID

    uint8       angle_enable;       // 内环使能(=1 才驱动转向电机)
    uint8       head_enable;        // 外环使能(=1 时目标转角由航向环给;=0 时由 target_delta 直给)

    float       target_yaw;         // 外环目标航向(度)
    float       meas_yaw;           // 外环实测航向(度)
    float       target_delta;       // 内环目标转角(编码器计数);外环开时被航向环覆写
    float       meas_delta;         // 内环实测转角(center_delta)
    int16       output_duty;        // 转向电机输出 duty(送 power_set_steer_duty)
} kart_steer_ctrl_t;

extern kart_steer_ctrl_t kart_steer;

/* --- 对外接口 --- */
void  kart_steer_ctrl_init(void);           // 初始化两级 PID,默认全不使能
void  kart_steer_ctrl_update(void);         // 串级一拍:放主循环 steer_abs_update 之后

void  kart_steer_set_angle_enable(uint8 en);    // 内环使能
void  kart_steer_set_head_enable(uint8 en);     // 外环使能(会连带开内环)
void  kart_steer_set_target_delta(float delta); // 直给内环目标转角(外环关时用,单测内环)
void  kart_steer_set_target_yaw(float yaw);     // 设外环目标航向

void  kart_steer_set_angle_pid(float kp, float ki, float kd);   // 在线调内环
void  kart_steer_set_angle_outmax(float outmax);                // 在线调内环输出限幅
void  kart_steer_set_head_pid(float kp, float ki, float kd);    // 在线调外环

/* 内环增益组切换(倒车前调 back、动作结束调 fwd 恢复)。
 * 只改三个系数,限幅沿用初始化值;内部会清一次 PID 记忆防跳变。 */
void  kart_steer_use_back_gains(void);      // 切倒车组(KP_BACK/KI_BACK/KD_BACK)
void  kart_steer_use_fwd_gains(void);       // 恢复前进组(KP_DEFAULT/...)

/* --- 给 VOFA/调试读的取值接口 --- */
float kart_steer_get_target_delta(void);
float kart_steer_get_meas_delta(void);
int16 kart_steer_get_output(void);
float kart_steer_get_target_yaw(void);

#endif
