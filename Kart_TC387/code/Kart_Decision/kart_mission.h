#ifndef KART_MISSION_H_
#define KART_MISSION_H_

#include "zf_common_headfile.h"
#include "board_pins.h"   /* KART_PERSON_LINK_ENABLE：下面 KART_S3_FOLLOW_SRC 的默认值要用 */

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
 * 运行模式由菜单或对外接口切换。
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
    MISSION_SUBJECT_3,          /* 科目三如影随形:跟随录轨 + 反向复现(车头不掉转,回放录制打角) */
    MISSION_REMOTE,             /* SBUS 遥控接管(调试/手动,VOFA m3 进入) */
    MISSION_FAULT,
} kart_mission_mode_t;

/* 科目一阶段。 */
typedef enum
{
    S1_WAIT_START = 0,      /* 等待发车(START 键下降沿触发) */
    S1_CONE_ROUTE,          /* 前向复现绕桩路线(kart_playback) */
    S1_FINISHED,            /* 停车结束 */
    S1_FAULT,               /* 当前仅用于复现启动失败；急停会直接回 IDLE */
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

/* -------------------------- 科目三阶段 --------------------------
 * 【2026-08 改名说明】本模式原名 MISSION_SUBJECT_4 / S4_*,是省赛时期的叫法。
 * 按最新国赛规则,代码里这套"跟着人走一段 + 原路自动返回"就是竞赛的【科目三】,
 * 故整体改名 S3_*。改名是纯标识符替换,没动任何控制逻辑/参数值/Flash 布局
 * (kart_params 按下标存,标签串改了但索引没动,老参数照样读得回来)。
 *
 * 阶段1 有两个可选的控制源,由 KART_S3_FOLLOW_SRC 编译期选:
 *   0 = 遥控(REMOTE):省赛已实车验证的老路子。人用遥控开一段,同时录轨。
 *   1 = 视觉跟随(VISION):摄像头认黄色引导板,kart_follow 出速度+打角,同时录轨。
 * 阶段2 两者完全相同 —— 都是 kart_playback 的开环反向复现,车头不掉转直接倒回。
 * 这样"如影随形"只替换了阶段1的输入源,把已经跑到 2.64m/s 的返程段整段复用,
 * 而且视觉一旦不灵,把 SRC 改回 0 重烧就退回能完赛的状态。 */
typedef enum
{
    S3_PHASE1_FOLLOW = 0,   /* 第一阶段:跟随(遥控或视觉)+录轨，等 START 键收尾 */
    S3_PHASE2_REVERSE,      /* 第二阶段:按 START 后倒车原路返回 */
    S3_SIGNAL,              /* 第三阶段:已回发车区,等语音口令做灯光/鸣笛 */
    S3_FINISHED,            /* 全流程结束 */
    S3_FAULT,
} kart_subject3_stage_t;

/* 阶段1 控制源。0=遥控(已验证) 1=视觉跟随(新增,待实车)。
 * 现场出问题改这里重烧即可整段退回省赛行为,不用动状态机。 */
#define KART_S3_FOLLOW_SRC_REMOTE   (0)
#define KART_S3_FOLLOW_SRC_VISION   (1)
/* 2 = TC4D7 人体视觉链路（PLINK）。387 不做检测，只收 4D7 送来的 25 字节帧。
 * 与 VISION 的区别只在“目标从哪来”：VISION 是本地摄像头认黄色引导板，
 * PLINK 是 4D7 认人。后面的 kart_follow、录轨、倒车返程三段完全复用。
 * 为何新增一个枚举而不直接把 VISION 分支改成读 4D7：
 * 摄像头那条路是现成的退路，改掉就没了；三个值并存，
 * 现场哪条不灵改一个宏重烧就能切。 */
#define KART_S3_FOLLOW_SRC_PLINK    (2)
/* 用 #ifndef 包起来是为了能从编译选项 -D 覆盖(离线验证三条分支都要能编)。
 * 2026-08-12：默认跟着 KART_PERSON_LINK_ENABLE 走。
 * 为何要联动而不写死：两个宏各自手改就有四种组合，其中两种是陷阱 ——
 *   链路开了但 SRC 还是 VISION：车拿没启用的摄像头结果跟随，永远丢目标；
 *   SRC 选了 PLINK 但链路没开：vtrack 永远 valid=0，车原地不动。
 * 两种都能编过、都不报错，只表现为“车不跟人”，现场很难往宏上想。
 * 要手动覆盖仍然可以：在本文件前面或 -D 定义 KART_S3_FOLLOW_SRC 即可。 */
#ifndef KART_S3_FOLLOW_SRC
#if KART_PERSON_LINK_ENABLE
#define KART_S3_FOLLOW_SRC          (KART_S3_FOLLOW_SRC_PLINK)
#else
#define KART_S3_FOLLOW_SRC          (KART_S3_FOLLOW_SRC_VISION)
#endif
#endif

/* “阶段1 是自动跟随”的统一判据。VISION 与 PLINK 只差在目标从哪来，
 * 后续那一堆共有逻辑（deadman 急停、进倒车前清打角、进模式时 follow_reset）
 * 对两者完全一样。没有它就要把每个 #if 写成两个比较的或，
 * 漏一处就是少一道急停 —— 那是会出事的那种漏。 */
#define KART_S3_FOLLOW_IS_AUTO      ((KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_VISION) || (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_PLINK))

/* 语音信号阶段超时(10ms/拍 → 6000 拍 = 60s)。
 * 超时不算故障:直接进 S3_FINISHED 收车,不让车/灯无限等下去。 */
#define KART_S3_SIGNAL_TIMEOUT_TICKS (6000U)

/* 视觉结果最大可复用拍数(10ms/拍)。相机 30FPS 约 3.3 拍一帧,连续 10 拍
 * (100ms)没有新帧说明采集链已经掉了,再用旧快照就是拿过期方位角打方向。
 * 旧代码只看相机状态兜底,而 RUNNING 判据来自 VSYNC,DMA 停了它仍是 RUNNING。 */
#define KART_S3_VISION_MAX_AGE_TICKS (10U)

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

/* 读科目三当前阶段(调试用)。 */
kart_subject3_stage_t kart_mission_get_subject3_stage(void);

/* 科目三视觉快照年龄(拍)与累计帧序号。日志用,判视觉链路是否还在更新。 */
uint16 kart_mission_get_s3_vision_age(void);
uint32 kart_mission_get_s3_vision_seq(void);

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
