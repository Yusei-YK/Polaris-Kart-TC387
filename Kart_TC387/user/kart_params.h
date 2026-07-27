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
    KART_PARAM_PB_PROF,         /* 1=用几何速度剖面,0=用录制速度(出厂 0,行为与改动前一致) */
    KART_PARAM_PB_SCALE,        /* 剖面整体倍率 —— 提速就调它(仅 PB Prof=1 时有效) */
    KART_PARAM_PB_VMAX,         /* 复现速度上限(脉冲/5ms) */
    KART_PARAM_PB_VMIN,         /* 复现速度地板(脉冲/5ms) —— 防低速录制段爬行 */
    KART_PARAM_PB_ALAT,         /* 弯道横向加速度上限(m/s²) —— 越小过弯越慢越稳 */
    KART_PARAM_PB_LDGAIN,       /* 前视距离速度增益(m per 脉冲/5ms) */
    KART_PARAM_RC_VMAX,         /* 遥控满油门速度(脉冲/5ms) —— 录制手感,调低防打滑 */
    KART_PARAM_HEAD_KP,         /* 航向外环 Kp —— 提速后走线画龙/发飘调它 */
    KART_PARAM_S1_REV_SPD,      /* 科目一倒库速度(脉冲/5ms,负) */
    KART_PARAM_S1_REV_STOP,     /* 科目一倒库停车里程(m) */
    KART_PARAM_S4_OL_SPD,       /* 科目四开环倒车速度(脉冲/5ms,负) */
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
