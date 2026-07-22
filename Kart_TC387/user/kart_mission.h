#ifndef KART_MISSION_H_
#define KART_MISSION_H_

#include "zf_common_headfile.h"

/*
 * 科目状态机(最小骨架)
 * ------------------------------------------------------------------
 * 顶层模式机 + 科目一阶段骨架。参考 TopSpeed Mission/Subject 的分层思想,
 * 但不照搬业务代码,也不先造通用事件框架。各科目用自己的非阻塞状态与计时。
 *
 * 顶层模式:
 *   IDLE       安全待机(车/转向/蜂鸣器全部无输出)
 *   SUBJECT_1  科目一自动驾驶(当前只搭空骨架,不自行启动)
 *   SUBJECT_2  科目二人车交互(语音识别 + 鸣笛在此轮询)
 *   FAULT      故障锁止
 *
 * 开发阶段用 VOFA 命令 m0/m1/m2 切模式(见 kart_debug_uart)。
 * 最终交互(IPS200 菜单 / 实体按键)待定,不使用语音选科目。
 *
 * 任意模式切换统一经 kart_mission_set_mode(): 先 exit 旧模式(统一停机),
 * 再 enter 新模式。保证互切无后轮/转向/蜂鸣器残留输出。
 *
 * 调用位置:
 *   kart_mission_init()  —— cpu0_main.c 初始化段(voice/horn init 之后)
 *   kart_mission_poll()  —— 主循环每拍(替代原先直接调 voice/horn)
 *   kart_mission_set_mode() —— VOFA m 命令 / 后续菜单
 * ------------------------------------------------------------------
 */

typedef enum
{
    MISSION_IDLE = 0,
    MISSION_SUBJECT_1,
    MISSION_SUBJECT_2,
    MISSION_REMOTE,             /* SBUS 遥控接管(调试/手动,VOFA m3 进入) */
    MISSION_FAULT,
} kart_mission_mode_t;

/* 科目一阶段。 */
typedef enum
{
    S1_WAIT_START = 0,      /* 等待发车(START 键下降沿触发) */
    S1_CONE_ROUTE,          /* 前向复现绕桩路线(kart_playback) */
    S1_GARAGE_APPROACH,     /* 绕桩终点(A 点)停稳,记倒车基准航向 */
    S1_REVERSE_IN,          /* 方向回中 + 负速直线倒车入库,里程判据停车 */
    S1_FINISHED,            /* 停车结束 */
    S1_FAULT,               /* 超时/传感器异常/急停 */
} kart_subject1_stage_t;

/* -------------------- 倒库参数(第一版,实车再修)-------------------- */
/* 倒车速度(脉冲/5ms,负值=倒退)。前向 playback 用 +50,倒车取半速求稳。 */
#define KART_S1_REVERSE_SPEED       (-25.0f)
/* 减速点(倒车里程增量,米):过此点降到慢速轻靠。GPT 建议 1.05m。 */
#define KART_S1_REVERSE_SLOW_DIST   (1.05f)
/* 慢速段速度(脉冲/5ms,负值)。 */
#define KART_S1_REVERSE_SLOW_SPEED  (-12.0f)
/* 停车点(倒车里程增量,米):到此点刹车收车。GPT 建议 1.40m(库深2m,车长约1.3m)。 */
#define KART_S1_REVERSE_STOP_DIST   (1.40f)
/* A 点停稳判据:车体测速幅值低于此值算停稳(脉冲/5ms)。 */
#define KART_S1_STOP_SPEED_EPS      (2.0f)

/* 初始化:模式置 IDLE,并执行一次统一停机(保证上电无残留输出)。 */
void kart_mission_init(void);

/* 切换顶层模式:内部 exit 旧模式 → enter 新模式。 */
void kart_mission_set_mode(kart_mission_mode_t mode);

/* 主循环每拍调:按当前模式分发 loop。 */
void kart_mission_poll(void);

/* 读当前模式(点阵屏显示 / 调试用)。 */
kart_mission_mode_t kart_mission_get_mode(void);

/* 读科目一当前阶段(调试用)。 */
kart_subject1_stage_t kart_mission_get_subject1_stage(void);

#endif
