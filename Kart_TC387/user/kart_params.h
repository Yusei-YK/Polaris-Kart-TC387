#ifndef KART_PARAMS_H_
#define KART_PARAMS_H_

#include "zf_common_headfile.h"

/*
 * 现场可调参数表(菜单在线调 + DFlash 持久化)
 * ------------------------------------------------------------------
 * 【为什么要它】比赛现场无上位机、无编译环境。原先这些值都是宏,改一个要
 *   重新编译烧写(07-26 就出过 KART_REMOTE_MAX_SPEED 被改回 20 导致速度只剩
 *   1/3 的事故)。放进本表后:菜单调 → 立即生效 → 存 Flash → 断电保留。
 *
 * 【与宏的关系】各模块的宏保留为"上电默认值",本表的值在 kart_params_init()
 *   里从 Flash 载入(无有效数据则用宏的默认),再由 params_apply() 推给各模块。
 *   即:宏 = 出厂值,本表 = 现场值。改宏依旧有效(Load Default 后生效)。
 *
 * 【Flash 页】占 DFlash 页 127(路径槽位用页 0~64,不冲突)。
 * ------------------------------------------------------------------
 * 存储布局(uint32 word):
 *   word[0] = MAGIC, word[1] = KART_PARAM_MAX, word[2..] = 各参数 float 位模式
 * 读回时 count 不符即整表判废用默认,避免加参数后读到错位的旧数据。
 * ------------------------------------------------------------------
 */

#define KART_PARAMS_MAGIC       (0x4B505241u)   /* 'KPRA' */
#define KART_PARAMS_PAGE        (127)           /* DFlash 页号(路径用 0~64) */

typedef enum
{
    KART_PARAM_RAMP = 0,        /* 速度斜坡步长(脉冲/5ms 每拍),0=关 —— 治起步顿挫 */
    KART_PARAM_PB_PROF,         /* 1=用几何速度剖面(出厂,配 Vmax=30 保守钳位),0=照抄录制速度 */
    KART_PARAM_PB_SCALE,        /* 剖面整体倍率 —— 提速就调它(仅 PB Prof=1 时有效) */
    KART_PARAM_PB_VMAX,         /* 复现速度上限(脉冲/5ms) */
    KART_PARAM_PB_VMIN,         /* 复现速度地板(脉冲/5ms) —— 防低速录制段爬行 */
    KART_PARAM_PB_ALAT,         /* 弯道横向加速度上限(m/s²) —— 越小过弯越慢越稳 */
    KART_PARAM_PB_LDGAIN,       /* 前视距离速度增益(m per 脉冲/5ms) */
    /* ---- 2026-07-28 赛前:三个原先只能改宏的量搬进菜单 ----
     * 现场调参的实际顺序是"先看跑得住不住,再决定钳多少/刹多早/看多远",
     * 这三个正是需要一趟一趟试的量,留在宏里等于现场不可调。 */
    KART_PARAM_PB_CLAMP,        /* 复现速度总钳位(脉冲/5ms) —— 原 KART_PLAYBACK_SPEED_MAX 宏。
                                 * 与 PB Vmax 的区别:Vmax 只管剖面算出来的目标,
                                 * 本项是下发给速度环之前的最后一道闸,录制速度回放也走它 */
    KART_PARAM_PB_ABRAKE,       /* 剖面反向传播减速度(m/s²) —— 原 KART_PLAYBACK_ABRAKE 宏。
                                 * 终点冲出去/倒车段前没减速就加大它 */
    KART_PARAM_PB_LDMAX,        /* 前视距离上限(m) —— 原 KART_PLAYBACK_LD_MAX 宏。
                                 * 绕桩削顶(切内侧锥桶)就调小,高速画龙就调大 */
    KART_PARAM_RC_VMAX,         /* 遥控满油门速度(脉冲/5ms) —— 录制手感,调低防打滑 */
    KART_PARAM_HEAD_KP,         /* 航向外环 Kp —— 提速后走线画龙/发飘调它 */
    /* ---- 下面两项当前【调了不起作用】,菜单里画成灰字 ----
     * kart_mission.h 的 KART_S1_AUTO_REVERSE=0 → 绕桩复现跑完直接判 S1_FINISHED,
     * 唯一切进 S1_GARAGE_APPROACH 的那行在 #if 里(kart_mission.c:200)。
     * 读这两个值的 S1_GARAGE_APPROACH / S1_REVERSE_IN 分支代码还在、也照样编译,
     * 但没有任何路径走得到 —— 倒库现在是录在轨迹里由 playback 一起复现的。
     * 要复活就把 KART_S1_AUTO_REVERSE 改 1(需重新烧写),菜单灰字会跟着变亮
     * (灰显判据在 kart_menu.c menu_param_is_active,改宏时记得同步)。 */
    KART_PARAM_S1_REV_SPD,      /* 科目一倒库速度(脉冲/5ms,负) */
    KART_PARAM_S1_REV_STOP,     /* 科目一倒库停车里程(m) */
    KART_PARAM_S4_OL_SPD,       /* 科目四倒车速度(脉冲/5ms,负) */
    /* ---- 科目四倒车方案切换(2026-07-28)。出厂 0 = 已实车验证能完赛的老路径 ----
     * 老方案(0):里程查表索引 + 航向 P 纠偏。实车结论:不撞筒,15m 走完终点横向
     *   偏 0.5~1m。误差有界线性累积,不发散 —— 这是能完赛的方案,故设为默认。
     * 新方案(1):最近点索引 + 航向 P + 横向位置 P。老路径代码一行不删,
     *   现场发现新方案不行,菜单打回 0 即恢复,不必重新烧写。 */
    KART_PARAM_S4_OL_MODE,      /* 0=里程查表(默认,已验证) 1=位置闭环 */
    KART_PARAM_S4_OL_KE,        /* 横向偏差 P 增益(编码器计数/米)。0=退化成纯航向纠偏;
                                 * 越纠越歪就取负(倒车横向反馈符号只有两种可能) */
    KART_PARAM_S4_OL_KH,        /* 倒车航向 P 增益(编码器计数/度)。原为宏,现场不可调,
                                 * 搬进菜单。两方案共用,老方案也跟着可调 */
    /* ---- 2026-07-28 第二批:把"限制项"本身搬进菜单 ----
     * 起因:科一复刻日志前进段 493 帧目标恒定 60.00(一帧不差),说明剖面整段顶穿了
     * 钳位,PB Scale 再往上加数值上是空的。真正卡住车速的是下面这些限幅本身 ——
     * 它们过去只能改宏,现场等于不可调。要把车逼到极限就得能动它们。 */
    KART_PARAM_SPD_IMAX,        /* 速度环积分限幅(duty)。日志实测 duty 全程没超过
                                 * 7300/10000:稳态 = Kp*e + I = 200*19.4 + 3000 = 6876,
                                 * 剩下 27% 量程被 i_max=3000 锁死 —— 提速第一道墙 */
    KART_PARAM_SPD_KP,          /* 速度环 Kp。与 Imax 共同决定稳态误差:目标 60
                                 * 实测只有 40.9,差的 19 脉冲就是这里来的 */
    KART_PARAM_STR_OUTMAX,      /* 转角内环输出限幅(duty)。提速后转向速率是硬瓶颈
                                 * (实测 1800 计数/s),放大它直接提高可用角速度 */
    KART_PARAM_PB_REVSCL,       /* 倒车段速度倍率。倒车段照抄的是【录制实测速度】,
                                 * 而录制指令 -28 实测只有 -16.4(59%),回放再拿 -16.4
                                 * 当目标 → 每过一代掉一档,过去没有旋钮能补 */
    KART_PARAM_SLEW_REAR,       /* 后轮 duty 每拍升幅上限。400 = 0→满约 125ms,
                                 * 起步/出弯加速被它限速 */
    KART_PARAM_PB_REVCORR,      /* 倒车段航向纠偏钳位(计数)。倒车打角 =
                                 * 录制打角 + clamp(纠偏, ±本值)。
                                 * 【只管一处】前进复现里【夹着的】倒车段
                                 * (kart_playback.c:477,靠录制速度符号判段)。
                                 * 科目四那种"整段独立开环倒车"不看它 ——
                                 * 两条倒车路径(里程查表 / 位置闭环)都用死宏
                                 * KART_PLAYBACK_OL_CORR_MAX=400,菜单调不动。
                                 * 所以科四倒车拐不进去,调本项【没用】。 */
    KART_PARAM_MAX
} kart_param_id_t;

/* 每个参数的元数据:菜单靠它通用渲染+步进,不必为每个参数写一遍界面代码。 */
typedef struct
{
    const char *name;       /* 菜单显示名(≤11 字符,IPS200 一行放得下) */
    float       min;
    float       max;
    float       step;       /* UP/DOWN 单次步进 */
    float       def;        /* 出厂默认(取自各模块宏) */
    uint8       decimals;   /* 显示小数位 0/1/2 */
} kart_param_meta_t;

/* 上电调一次:载入 Flash(无效则用默认)并 apply 到各模块。
 * 必须在 kart_control_init/kart_steer_ctrl_init 之后调(apply 会覆写它们的默认值)。 */
void  kart_params_init(void);

float kart_params_get(uint8 id);
/* 设值(内部钳到 [min,max])并立即 apply 到对应模块。只写 RAM,不写 Flash。 */
void  kart_params_set(uint8 id, float v);
/* 按 step 增减(dir=+1/-1),内部钳位 + apply。菜单 UP/DOWN 用。 */
void  kart_params_step(uint8 id, int8 dir);
/* 一次走 mul 个 step(粗调)。菜单长按 UP/DOWN 或快旋旋钮时传 10:
 * 步长本身按"最小可分辨改动"定,粗调不另设一套步长表,直接乘倍率。 */
void  kart_params_step_mul(uint8 id, int8 dir, uint8 mul);

const kart_param_meta_t* kart_params_meta(uint8 id);

/* 全表存 Flash(阻塞擦写一页,只能停车静止时调)。0=成功。 */
uint8 kart_params_save(void);
/* 有未落盘改动才存,无改动直接跳过。返回 1=本次写了 Flash,0=没写。
 * 菜单退出参数界面/就绪界面时调:调完的值断电、reset 重置 IMU 都不丢。 */
uint8 kart_params_save_if_dirty(void);
/* 是否有改过但还没存的值(菜单用它显示 * 提示)。 */
uint8 kart_params_is_dirty(void);
/* 全表恢复出厂默认并 apply(不自动存 Flash)。 */
void  kart_params_load_default(void);

#endif
