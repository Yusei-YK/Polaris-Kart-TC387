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
 *   【0.088°/计数 是编码器轴,不是前轮转角】前轮那一侧小得多:
 *     满舵 1064 计数对应前轮 atan(0.62/1.39) = 24.0°,即约 0.0226°/计数。
 *     下面 out_max 那段"400 计数(9.1°)"用的就是这个 0.0226,是对的;
 *     谁拿 0.088 乘 1064 会算出 93.5° 这种不存在的满舵角。
 *   center_delta 范围见 kart_calib.h 第四节(当前软限 ±1064)。
 *     1064 是派生值不是手填的:硬限位 raw 1248 与中位 164 → 行程 1084,
 *     再减 KART_STEER_LIMIT_MARGIN(20)= 1064,左右对称。
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
 * 中位→满舵 1064 计数需 0.59s(原文写的"1103 计数 0.61s"不能当现值:
 * 1103 比硬限位行程 1084 还大,那组手录数是改硬限位标定之前的;结论不变),
 * 而 60 脉冲(4.4 m/s)下 1.50m 前视只给 0.34s 预判
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
#define STEER_KP_BACK              (5.0f)      /* 前进 15 的 1/3 */
#define STEER_KI_BACK              (0.5f)      /* 与前进同:顶静摩擦用,不动 */
#define STEER_KD_BACK              (2.5f)      /* 前进 0 → 加阻尼压振 */

/* 航向外环 PID 默认。输出是"目标转角计数",限幅到转角软限位量级。
 * 【30 已验证,不是占位,不动】凭据:
 * ① 实车科目一/科目三走线已跑通。
 * ② 内环静摩擦死区 950/15 = 63 计数,除以 30 计数/度是 2.1°,即 2.1° 以内的
 *    航向误差不动方向盘。1.50m 前视下 2.1° 约 5.5cm 横向偏差,绕桩还吃得住,
 *    所以 30 不动;但再提速后若出现走线滞后,这个死区是第一个该看的地方:
 *    提 KART_HEAD_KP(度→计数)或提内环 Kp(计数→duty)都能压小它,代价都是
 *    过冲变大。
 * ③ 与轴距 0.62m 算出的 0.86m 航向收敛特征长度自洽。 */
#define KART_HEAD_KP_DEFAULT            (30.0f)
#define KART_HEAD_KI_DEFAULT            (0.0f)
#define KART_HEAD_KD_DEFAULT            (0.0f)
#define KART_HEAD_IMAX_DEFAULT          (500.0f)
/* 外环上限必须【等于】软限位,不能是独立填的数。
 * 病因(2026-07-28 日志):科目一复刻前进段 493 帧里有 304 帧(62%)的
 * target_delta 恰好顶在 ±1000.00 —— 当时这里手填 1000,比软限位小,等于在
 * 软限位之外又架了一道暗闸,把绕桩需要的最后几十计数削掉,表现为复刻切内侧锥桶。
 * 现在直接引用 KART_STEER_DELTA_LIMIT_L:重标硬限位 raw 后自动跟随,
 * 限角只由软限位一处决定。
 * (这里原先堆了两段历史,彼此对不上也对不上现值 —— 一段说改成 1133,
 *  一段说从 795 改成 1064。795 已经没有出处;1133 还剩一处化石:
 *  kart_params.c 的 "PB RevCorr" 量程上限就是 1133,那是按当时的软限位填的,
 *  现在比软限位 1064、甚至比硬限位行程 1084 都大 —— 菜单上把纠偏钳位调到
 *  1064 以上是空的,方向盘到不了。量程不动(改槽位量程是红线),知道就行。
 *  本宏的值以下面这行为准,那两个历史数字不必再信。) */
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
void  kart_steer_ctrl_update(void);         // 串级一拍:放主循环 kart_steer_abs_update 之后

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
