#ifndef KART_CONTROL_H_
#define KART_CONTROL_H_

#include "zf_common_headfile.h"
#include "kart_pid.h"
#include "kart_power.h"    /* KART_POWER_MAX_DUTY:下方 KART_SPEED_OUTMAX_DEFAULT 要用 */

/*
 * 卡丁车控制环(速度环先行)
 * ------------------------------------------------------------------
 * 移植自 TopSpeed 速度环(Subject_1 里的 vel_ctrler + Speed_encoder 滤波)。
 * 对照原版的改动(都是为卡丁车硬件适配,不是瞎改):
 *   1. 测速:TopSpeed 靠无刷电机板 UART 回传脉冲;卡丁车编码器直连 TC387,
 *      本地读 delta,固定 5ms 采样(原版是收到帧才算,异步)。
 *   2. duty:TopSpeed 用 += 累加(它同节拍多个环叠加);卡丁车速度环单独跑,
 *      改成直接赋值 duty = output,更直观好调(已跟用户确认)。
 *   3. 下发:TopSpeed 走 send_motor_duty 无刷串口;卡丁车是有刷 DRV8701,
 *      换成 power_set_rear_duty() 出 PWM。
 * 保留照搬的:10 点滑动平均滤波、位置式 PID、目标速度斜坡(后面接规划时用)。
 * ------------------------------------------------------------------
 * 调用节拍:kart_control_speed_update() 必须放 5ms 定时中断里,和 IMU 同拍。
 * 单位说明:当前速度环仍用"编码器脉冲/5ms"在线调初值;
 *          左右距离标定量已分开放在 board_pins.h,后续速度环再统一改成 m/s。
 * ------------------------------------------------------------------
 */

/* 滑动平均滤波窗口长度(照搬 TopSpeed WHEEL_SPD_LPF_TEMP_LEN) */
#define KART_SPEED_LPF_LEN              (10)

/* ===== 目标速度斜坡(治起步/加速顿挫)=====
 * 病因链(2026-07-27 定位):遥控油门/playback 把目标速度阶跃写进速度环,
 *   ① Kp=200,起步误差 40 → P 项一拍就要 8000 duty;
 *   ② power_sync 的 slew 限幅只放 400 duty/拍,PID 想给的给不出去(执行器饱和);
 *   ③ kart_pid 的抗饱和只看自己 output 是否撞 out_max,看不见 slew 造成的饱和,
 *      于是 err_sum 一路攒到 i_max/Ki=3750;
 *   ④ slew 追上后 PID 已饱和 → 车猛窜过目标 → PID 反向刹车 → duty 异号
 *      触发 slew 穿零保护:强制归零 + 驻留 6 拍(30ms 动力硬断)= 用户感到的"顿挫"。
 * 对策:在目标端限速。上层 set_target 写 target_cmd,速度环每拍把实际喂 PID 的
 *   target 朝 target_cmd 挪最多 RAMP_STEP。误差恒小 → PID 输出变化率远低于
 *   slew step → 不饱和 → 不攒积分 → 不超调 → 不触发穿零驻留。整条链一次解开。
 * 【只限加速,不限减速/停车/反向】:减速与急停必须瞬时,与 slew 同样的安全取向。
 * 单位:脉冲/5ms 每拍。运行时可由 kart_params 覆盖(菜单在线调),此宏仅作上电默认值。
 *
 * 【2026-07-28 赛前修正:1.5 → 0.4】原值按"0→满速 200ms 听着够缓"选的,
 * 换成物理量就露馅了:
 *   1.5 脉冲/5ms 每拍 ÷ 0.005s = 300 (脉冲/5ms)/s × 0.0736 = 22 m/s²。
 * 而整车实际能给的:2×150W 驱动 ÷ 22kg,在 4 m/s 处 P/(m·v) ≈ 3.4 m/s²;
 * EVA 硬轮抓地上限 0.5~0.7g ≈ 5~7 m/s²。1.5 的斜坡比车跑得出来的快 5~9 倍 ——
 * 目标速度一瞬间就跑到实际速度前面去,误差照样一次性张开,上面那条病因链
 * (①~④)一条都没被治住,只是被"看起来有斜坡"掩盖了。这才是起步窜车的真因,
 * 不是编码器标定。
 * 0.4 → 0.4/0.005 × 0.0736 = 5.9 m/s²,略高于抓地上限:既不人为拖慢加速
 * (先打滑的是轮胎,斜坡不是瓶颈),又能真正把 PID 误差压在小量。
 * 0→60 脉冲 = 150 拍 = 750ms,与"22kg / 300W 加到 4.4 m/s"的物理时间同量级。 */
/* 速度环默认 PID 参数 —— 2026-07-17 上车调定并冻结。
 * 量纲是"脉冲/5ms"。串口在线调参最终定为 Kp=200 / Ki=0.8 / Kd=0,
 * 实测 I12≈I13 左右脉冲率拉平、稳态误差可接受,底层速度环到此冻结。
 * (左右后轮机械差异由前轮转向的航向角度环处理,不在速度环加电子差速。)
 * i_max/out_max 的 out_max 给 KART_POWER_MAX_DUTY(满量程),让 PID 能用满输出范围。
 * 2026-07-28 从 kart_control.c 搬到本头文件:Kp/Imax 已进 kart_params 表当出厂默认值,
 * 表里要引用这两个宏 —— 位置变了,值一个没动,"宏=出厂默认"的约定不变。 */
#define KART_SPEED_KP_DEFAULT          (200.0f)
#define KART_SPEED_KI_DEFAULT          (0.8f)
#define KART_SPEED_KD_DEFAULT          (0.0f)
#define KART_SPEED_IMAX_DEFAULT        (3000.0f)
#define KART_SPEED_OUTMAX_DEFAULT      ((float)KART_POWER_MAX_DUTY)

#define KART_SPEED_RAMP_STEP_DEFAULT    (0.4f)

#define KART_REAR_GEAR_RATIO           (2.0f)      // 后轮减速比 40:20,标定真实车速时用

/* 电子差速(前轮转向车,左右后轮转弯半径不同→内轮慢外轮快)。
 * 航向环只管前轮转角,消不掉后轮轮速差;同目标喂两独立 PID 会拖滑互顶。
 * 用实测转角(非目标)分配左右目标:内轮乘(1-r),外轮乘(1+r)。
 * 初版保守:GAIN 小、比例上限 12%;倒车关闭(符号未验证)。
 * 【2026-07-28 赛前复核:保持关闭】不是"来不及验",是量程本身就配不上:
 * 满舵 R=1.32m、轮距 0.60m → 内外轮半径 1.02m / 1.62m,轮速偏差 ±23%,
 * 而 MAX_RATIO 只有 ±12% —— 恰恰在最需要差速的满舵处只能补一半,
 * 补一半的效果是"两个 PID 仍在互顶,只是顶得轻些",引入的不确定性大于收益。
 * 要开必须先把 MAX_RATIO 提到 0.25 并实车验符号,赛前不动这条路径。 */
#define KART_EDIFF_ENABLE              (0)
#define KART_EDIFF_GAIN                (0.08f)     // steer_norm→差速比例增益
#define KART_EDIFF_MAX_RATIO           (0.12f)     // 差速比例上限(±12%)

/* -------------------------------------------------------------------------
 * 速度环控制块:把速度环所有状态打包在一起
 * ------------------------------------------------------------------------- */
typedef struct
{
    /* pid 保持为左轮控制器，兼容现有在线调参命令。 */
    kart_pid_t  pid_right;
    kart_pid_t  pid;                            // 速度环 PID(复用 kart_pid)
    uint8       enable;                         // =1 才输出,=0 输出 0(悬空/急停用)

    float       target;                         // 斜坡后的实际 PID 目标(脉冲/5ms)
    float       target_cmd;                     // 上层请求的目标(set_target 写),target 每拍朝它爬
    float       ramp_step;                      // 加速斜坡步长(脉冲/5ms 每拍),0=不限速
    float       left_target;                    // 左轮目标(电子差速后,脉冲/5ms)
    float       right_target;                   // 右轮目标(电子差速后,脉冲/5ms)
    float       meas_raw;                       // 本次实测速度(滤波前,脉冲/5ms)
    float       meas;                           // 滤波后实测速度(脉冲/5ms),PID 反馈用这个

    float       lpf_buf[KART_SPEED_LPF_LEN];    // 滑动平均缓冲区
    int         lpf_idx;                        // 缓冲区写指针(环形)

    int16       output_duty;                    // 左右输出平均值，兼容原 VOFA ch2
    float       meas_left;
    float       meas_right;
    float       lpf_right[KART_SPEED_LPF_LEN];
    int16       output_left;
    int16       output_right;

    /* 开环 duty 支路(科目二固定动作用):=1 时跳过 PID,直接把 open_duty 下发两后轮。
     * 【为什么放在本模块内、不在上层直接写 PWM】
     *   enable 是"后轮有没有主人"的唯一开关,全工程有三处冗余检查它并清零后轮:
     *     kart_control.c(本模块) / isr.c:70(硬写 PWM) / cpu0_main.c:103
     *   上游改这个开关的是遥控挡位(SW3_H 开 / SW3_M 关)、遥控失联、mission 切模式。
     *   上层若绕过本模块直接 power_set_rear_duty(),会被这三处里的某一处抹成 0。
     *   所以开环也走本模块:enable 照样置 1(三处检查放行、遥控急停链完全不变),
     *   只把输出源从 PID 换成固定值。 */
    uint8       open_loop;
    int16       open_duty;
} kart_speed_ctrl_t;

/* 让调试输出能读到速度环内部状态(VOFA 波形要用) */
extern kart_speed_ctrl_t kart_speed;

/* --- 对外接口 --- */
void  kart_control_init(void);                  // 初始化速度环(填默认 PID 参数)
void  kart_control_speed_update(void);          // 速度环一拍:读编码器→滤波→PID→下发。放 5ms 中断
void  kart_control_set_enable(uint8 en);        // 使能/关闭速度环输出
void  kart_control_set_target(float target);    // 设目标速度(脉冲/5ms)
void  kart_control_set_pid(float kp, float ki, float kd);   // 在线改 PID 参数
/* 速度环限幅/增益在线调(菜单 Spd Imax / Spd Kp)。都不清 PID 记忆:
 * 行驶中调参必须无级平滑,reset 会让积分掉 0 → 掉速一拍。 */
void  kart_control_set_speed_imax(float imax);
void  kart_control_set_speed_kp(float kp);

/* 设加速斜坡步长(脉冲/5ms 每拍)。0 = 关闭斜坡(恢复旧的阶跃行为,A/B 对照用)。
 * 只影响"目标幅值增大"方向;减速/停车/反向恒为瞬时,不受此值影响。 */
void  kart_control_set_ramp_step(float step);
float kart_control_get_ramp_step(void);
/* 读上层请求的目标(斜坡前)。get_target 返回的是斜坡后的实际 PID 目标。 */
float kart_control_get_target_cmd(void);

/* 开环 duty:设固定后轮 duty 并进开环支路(不跑 PID)。仍需自行 set_enable(1)。
 * 内部先写 duty 再置标志,保证 5ms 中断不会读到"已开环但 duty 还是旧值"的中间态。 */
void  kart_control_set_open_duty(int16 duty);
/* 退出开环支路,回到速度环 PID(会清 PID 记忆)。 */
void  kart_control_clear_open_loop(void);

/* --- 给调试/VOFA 读的取值接口 --- */
float kart_control_get_target(void);            // 目标速度
float kart_control_get_meas(void);              // 滤波后实测速度
int16 kart_control_get_output(void);            // 当前输出 duty
uint8 kart_control_is_enabled(void);            // 速度环是否使能(主循环用它仲裁:谁来管后轮 duty)

float kart_control_get_left_meas(void);
float kart_control_get_right_meas(void);
int16 kart_control_get_left_output(void);
int16 kart_control_get_right_output(void);

#endif
