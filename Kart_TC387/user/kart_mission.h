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
    MISSION_SUBJECT_4,          /* 科目四迷宫录制+反向复现(车头不掉转,回放录制打角) */
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

/* 科目二阶段(2026-07-29 加,只为语音返回的两步时序)。
 * 科目二绝大多数时间是 S2_IDLE:语音来一条执行一条,由 dispatch 直接派发,
 * 【没有】固定流程可言(现场随机抽取口令),所以这里不做全流程状态机。
 * 唯一需要跨命令记状态的就是返回:GOTO 摆位 → 复现返回路径,两步之间要交接。 */
typedef enum
{
    S2_IDLE = 0,            /* 待命:听口令,dispatch 直接派发单条动作 */
    S2_RETURN_GOTO,         /* 返回第①步:GOTO 把车摆到返回路径录制起点附近 */
    S2_RETURN_PLAYBACK,     /* 返回第②步:按录制原点复现返回路径(穿门洞回发车区) */
    S2_RETURN_DONE,         /* 返回完成:已停在发车区 */
    S2_RETURN_FAULT,        /* 返回失败(槽位空/GOTO 超时),已停机 */
} kart_subject2_stage_t;

/* 科目四阶段(反向复现:车头不掉转,直接倒车原路返回)。 */
typedef enum
{
    S4_PHASE1_RECORD = 0,   /* 第一阶段:遥控走迷宫+录制(开到停车区停住即自动停录) */
    S4_PHASE2_REVERSE,      /* 第二阶段:倒车原路返回(判停自动触发,不用按键) */
    S4_FINISHED,            /* 返回完成 */
    S4_FAULT,
} kart_subject4_stage_t;

/* -------------------- 科目四自动停录→倒车参数 --------------------
 * 不用按 START:人把车开到停车区松杆停住,车自己判"停住"就停录并立即开倒车。
 * 判据 = 已走够 ARM_DIST(防发车前静止就误触发)+ 左右轮测速幅值连续低于 EPS
 * 达 HOLD 拍(10ms/拍 → 15 拍 = 150ms)。
 *
 * 不再判打角:遥控阶段人本来就会"把车开直回正再停车",故录制末点打角≈0,
 * 倒车起步不带角度,不需要额外的 SETTLE 摆角等待(那一段最多要 3s,比手按还慢)。
 * 前提:回正必须在车还在动的时候做完 —— 录制是按里程采样的,停住之后再回正
 * 不会被记进 steer_buf,末点打角仍是回正前的角度,倒车会照着它往回打。
 *
 * 注意:若人一直压着油门但车被卡住(轮子空转除外),也会判成停住 → 自动倒车。
 * START 键仍保留为手动提前触发,不想等这 150ms 时可以按。 */
#define KART_S4_AUTOSTOP_ARM_DIST   (1.0f)      /* 解锁自动判停的最小行驶里程(m) */
/* 2026-07-30:EPS 2.0→4.0、HOLD 50→15,削真空期(原来滑停+确认要 1~2s)。
 * 倒车目标本身会主动刹住前进惯量,不用干等滑停到 2.0。
 * 副作用:过了 ARM_DIST 后中途停顿 >150ms 会提前触发倒车。 */
#define KART_S4_AUTOSTOP_SPEED_EPS  (4.0f)      /* 停住判据(脉冲/5ms) */
#define KART_S4_AUTOSTOP_HOLD_TICKS (15U)       /* 连续停住拍数(10ms/拍 → 150ms) */

/* -------------------- 倒库参数(第一版,实车再修)-------------------- */
/* 倒车速度(脉冲/5ms,负值=倒退)。前向 playback 用 +50,倒车取半速求稳。 */
/* 科目一自动倒库总开关(2026-07-28 关)。
 * 0 = 倒车入库整段由手动录制的轨迹回放覆盖,playback 跑完即结束;
 * 1 = 旧行为:绕桩复现结束后走 S1_GARAGE_APPROACH + S1_REVERSE_IN 写死直线倒车。
 * 关掉的理由:写死的 1.4m 直线倒车不是轨迹复现,且 playback 结束时转向断电、
 * 前轮停在绕桩末角度,下一拍就给倒车速度 → 起步带角度。整段录制没有这个交界。
 * 老路径代码一行不删,现场要退回旧行为把这里改 1 重烧即可。 */
#define KART_S1_AUTO_REVERSE        (0)

#define KART_S1_REVERSE_SPEED       (-25.0f)
/* 减速点(倒车里程增量,米):过此点降到慢速轻靠。GPT 建议 1.05m。 */
#define KART_S1_REVERSE_SLOW_DIST   (1.05f)
/* 慢速段速度(脉冲/5ms,负值)。 */
#define KART_S1_REVERSE_SLOW_SPEED  (-12.0f)
/* 停车点(倒车里程增量,米):到此点刹车收车。GPT 建议 1.40m(库深2m,车长约1.3m)。 */
#define KART_S1_REVERSE_STOP_DIST   (1.40f)
/* A 点停稳判据:车体测速幅值低于此值算停稳(脉冲/5ms)。 */
#define KART_S1_STOP_SPEED_EPS      (2.0f)

/* 语音返回 GOTO 段的【第二道】超时闸(10ms/拍 → 3500 拍 = 35s)。
 * GOTO 自己已经有超距(25m)+ 超时(3000 拍)兜底,这里再加一层是因为:
 * 那两个判据都在 kart_motion 内部,若 GOTO 状态机因为某个几何出口写错而卡在
 * 某段不推进(状态既不是 DONE 也不是 FAULT),kart_mission 会永远停在
 * S2_RETURN_GOTO 等一个不会来的结果 —— 车在场地里一直开。
 * 取 3500 > 3000:正常情况永远由 GOTO 自己的闸先响,这一道只在"GOTO 失灵"时生效。 */
#define KART_S2_RETURN_GOTO_TICKS   (3500U)

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

/* 读科目四当前阶段(调试用)。 */
kart_subject4_stage_t kart_mission_get_subject4_stage(void);

/* 读科目二当前阶段(调试/菜单显示返回进度用)。 */
kart_subject2_stage_t kart_mission_get_subject2_stage(void);

/* 科目二返程模式。比赛中按一次 Voice 进场后就【不能再碰板子】,所以两套方案
 * 必须在同一次语音会话里选定,进场前用菜单 Voice A / Voice B 定死。
 *   0 = Voice A:返回口令走方案A(GOTO 自动摆位 + 复现)。
 *   1 = Voice B:S2_IDLE 空闲时遥控常驻接管(人开车回集结点),返回口令走方案B
 *                (按当前位姿复现)。固定动作期间遥控自动让位给运动状态机。 */
void  kart_mission_subject2_set_manual_return(uint8 en);
uint8 kart_mission_subject2_get_manual_return(void);

/* 【方案A:自动返回】由 kart_voice_dispatch() 在收到返回口令(0x1A~0x1E)时调:
 * 载入返程槽 6+slot_index,取其录制起点(= 集结点)作为 GOTO 目标位姿并进
 * S2_RETURN_GOTO,GOTO 到位后由 subject2_loop 接着启动复现。
 * slot_index = 0~4(对应返回槽 6~10)。
 * 返回 1=已受理,0=拒绝(槽空/参数越界/正忙)。
 *
 * 【场上必须知道】这条链路的摆位段离线仿真 261/304(≈86%)。失败模式是 GOTO
 * 判 FAULT 停机 —— 车停在场地里不动,【不会】带着错位姿放出复现路径去撞桩。
 * 抽到那 14% 就改用下面的方案B。 */
uint8 kart_mission_subject2_start_return(uint8 slot_index);

/* 【方案B:人工摆位后直接复现】兜底通道,不跑 GOTO。
 * 用法:遥控把车开回集结点、车头照去程方向摆正,再调这里(菜单 S2 Return
 * → Playback,或调试口)。载入槽 6+slot_index 后按【当前位姿】启动复现。
 * slot_index = 0~4。返回 1=已受理,0=拒绝(槽空/参数越界/正忙)。
 *
 * 【为什么它比方案A 更可靠】摆位由人眼完成,精度好于 GOTO,而且一眼看得出摆没摆正;
 * 按当前位姿启动等于把 odom 的累计漂移一次性归零。代价是要人动手、慢。
 * 【前提】车必须真的摆正 —— 摆位误差会原样变成整条路径的平移误差。 */
uint8 kart_mission_subject2_start_return_here(uint8 slot_index);

#endif
