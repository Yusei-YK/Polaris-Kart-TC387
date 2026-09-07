#ifndef KART_MISSION_H_
#define KART_MISSION_H_
#include "zf_common_headfile.h"
#include "board_pins.h"   /* PERSON_LINK_ENABLE：下面 S3_FOLLOW_SRC 的默认值要用 */

/*
 * 科目状态机(最小骨架)
 * ------------------------------------------------------------------
 * 顶层模式机 + 科目一阶段骨架。参考 TopSpeed Mission/Subject 的分层思想,
 * 但不照搬业务代码,也不先造通用事件框架。各科目用自己的非阻塞状态与计时。
 *
 * 顶层模式(与下面的枚举一一对应,七个):
 *   IDLE       安全待机(车/转向/蜂鸣器全部无输出)
 *   SUBJECT_1  科目一绕桩:START 键触发 kart_playback 前向复现,是活的流程
 *   SUBJECT_2  科目二人车交互(语音识别 + 鸣笛在此轮询)
 *   SUBJECT_3  科目三如影随形:跟随录轨 → 开环反向复现 → 盲盒固定动作
 *   REMOTE     SBUS 遥控接管(调试/手动)
 *   FAULT      故障锁止
 *   PEDAL      踏板驾驶:人踩油门/刹车,方向盘走机械连杆,转向电机全程不使能
 *
 * 运行模式由 IPS200 菜单切换:kart_menu.c 里非注释的 kart_mission_set_mode
 * 调用点有 21 处(踏板页、五条科目入口、遥控页、退出路径各有几处)。改菜单
 * 结构时要意识到这个数量级 —— 模式切换不是集中在一个地方做的。
 * 调试口的 VOFA m 命令是备用入口。不使用语音选科目。
 *
 * 任意模式切换统一经 kart_mission_set_mode(): 先 exit 旧模式(统一停机),
 * 再 enter 新模式。保证互切无后轮/转向/蜂鸣器残留输出。
 *
 * 调用位置:
 *   kart_mission_init()  —— cpu0_main.c 初始化段(kart_voice/kart_horn init 之后)
 *   kart_mission_poll()  —— 【10ms 一拍】cpu0_main.c 的 kart_task_10ms() 里,
 *       由 5ms 调度器每两拍派发一次(替代原先直接调 kart_voice/kart_horn)。
 *       本文件所有 *_TICKS 宏都按 10ms/拍折算成秒,换调用点就得连它们一起改,
 *       否则每个超时的实际秒数都会跟着变 —— 这是本模块最容易漏的一处耦合。
 *   kart_mission_set_mode() —— IPS200 菜单为主入口,VOFA m 命令为备用
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
    /* 【为什么加在最后,而不是插在 MISSION_REMOTE 后面】模式号会原样上 VOFA
     * 第 0 路(kart_debug_uart.c ch[0] = (float)kart_mission_get_mode()),
     * 插在中间会把之前所有日志里 FAULT=5 的含义改掉,历史数据就对不上了。 */
    MISSION_PEDAL,              /* 踏板驾驶:人踩油门/刹车,方向盘走机械连杆,转向电机全程不使能 */
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
 * 这套"跟着人走一段 + 原路自动返回"按国赛规则就是竞赛的【科目三】,所以标识符
 * 一律用 S3_*。kart_params 按下标存,标签串换了索引没动,老参数照样读得回来。
 *
 * 阶段1 有两个可选的控制源,由 S3_FOLLOW_SRC 编译期选:
 *   0 = 遥控(REMOTE):省赛已实车验证的老路子。人用遥控开一段,同时录轨。
 *   1 = 视觉跟随(VISION):摄像头认黄色引导板,kart_follow 出速度+打角,同时录轨。
 * 阶段2 两者完全相同 —— 都是 kart_playback 的开环反向复现,车头不掉转直接倒回。
 * 这样"如影随形"只替换了阶段1的输入源,把已经跑到 2.64m/s 的返程段整段复用,
 * 而且视觉一旦不灵,把 SRC 改回 0 重烧就退回能完赛的状态。 */
typedef enum
{
    S3_PHASE1_FOLLOW = 0,   /* 第一阶段:跟随(遥控或视觉)+录轨，等 START 键收尾 */
    S3_PHASE2_REVERSE,      /* 第二阶段:按 START 后倒车原路返回 */
    S3_FIXED_ACT,           /* 第三阶段:自动执行固定动作(出库→倒回),不等 START */
    S3_SIGNAL,              /* 第三阶段:已回发车区,等语音口令做灯光/鸣笛 */
    S3_FINISHED,            /* 全流程结束 */
    S3_FAULT,
} kart_subject3_stage_t;

/* 阶段1 控制源。0=遥控(已验证) 1=视觉跟随(新增,待实车)。
 * 现场出问题改这里重烧即可整段退回省赛行为,不用动状态机。 */
#define S3_FOLLOW_SRC_REMOTE   (0)
#define S3_FOLLOW_SRC_VISION   (1)
/* 2 = TC4D7 人体视觉链路（PLINK）。387 不做检测，只收 4D7 送来的 25 字节帧。
 * 与 VISION 的区别只在“目标从哪来”：VISION 是本地摄像头认黄色引导板，
 * PLINK 是 4D7 认人。后面的 kart_follow、录轨、倒车返程三段完全复用。
 * 为何新增一个枚举而不直接把 VISION 分支改成读 4D7：
 * 摄像头那条路是现成的退路，改掉就没了；三个值并存，
 * 现场哪条不灵改一个宏重烧就能切。 */
#define S3_FOLLOW_SRC_PLINK    (2)
/* 用 #ifndef 包起来是为了能从编译选项 -D 覆盖(离线验证三条分支都要能编)。
 * 2026-08-12：默认跟着 PERSON_LINK_ENABLE 走。
 * 为何要联动而不写死：两个宏各自手改就有四种组合，其中两种是陷阱 ——
 *   链路开了但 SRC 还是 VISION：车拿没启用的摄像头结果跟随，永远丢目标；
 *   SRC 选了 PLINK 但链路没开：kart_vtrack 永远 valid=0，车原地不动。
 * 两种都能编过、都不报错，只表现为“车不跟人”，现场很难往宏上想。
 * 要手动覆盖仍然可以：在本文件前面或 -D 定义 S3_FOLLOW_SRC 即可。
 * 【当前出厂值是 VISION,不是 PLINK】board_pins.h 里 PERSON_LINK_ENABLE 是 0,
 * 所以下面这个 #if 走 #else 分支:阶段1 用本地摄像头认黄色引导板。
 * 要换成 4D7 认人,只改 board_pins.h 那一个宏,这里自动跟上。 */
#ifndef S3_FOLLOW_SRC
#if PERSON_LINK_ENABLE
#define S3_FOLLOW_SRC          (S3_FOLLOW_SRC_PLINK)
#else
#define S3_FOLLOW_SRC          (S3_FOLLOW_SRC_VISION)
#endif
#endif

/* “阶段1 是自动跟随”的统一判据。VISION 与 PLINK 只差在目标从哪来，
 * 后续那一堆共有逻辑（deadman 急停、进倒车前清打角、进模式时 kart_follow_reset）
 * 对两者完全一样。没有它就要把每个 #if 写成两个比较的或，
 * 漏一处就是少一道急停 —— 那是会出事的那种漏。 */
#define S3_FOLLOW_IS_AUTO      ((S3_FOLLOW_SRC == S3_FOLLOW_SRC_VISION) || (S3_FOLLOW_SRC == S3_FOLLOW_SRC_PLINK))

/* 语音信号阶段超时(10ms/拍 → 6000 拍 = 60s)。
 * 超时不算故障:直接进 S3_FINISHED 收车,不让车/灯无限等下去。 */
#define S3_SIGNAL_TIMEOUT_TICKS (6000U)

/* -------------------- 科目三盲盒任务阶段(S3_FIXED_ACT)--------------------
 * 倒车复现完成后【不等 START、不等语音】自动执行:出库前进 2.8m → 倒回 2.8m,
 * 距离与形状在 kart_motion.h 的 KART_MOTION_S3_OUT_DIST / IN_DIST。
 *
 * 【这一段做完直接进 S3_FINISHED,不碰语音串口】用户确认:科目三【没有】语音
 * 环节,语音任务是走菜单进科目二做的。所以原来"倒车完成 → S3_SIGNAL 等口令"
 * 这条路对科目三是纯负担 —— 进 S3_SIGNAL 会把 UART10 从 460800 切到 115200
 * 让给语音模块,VOFA 日志当场断掉,而盲盒任务恰恰是最需要日志的一段。
 * S3_SIGNAL 那个 case 保留但已不可达(ENABLE=0 时才走回去)。
 *
 * ENABLE 置 0 即退回改动前的行为(倒车完成进 S3_SIGNAL 抢语音串口)。 */
#define S3_FIXED_ACT_ENABLE      (1)

/* 科目三到底有没有持有语音串口 —— kart_mission_set_mode 退出时据此决定要不要
 * kart_voice_uart_release() + 重开日志闸。
 * ENABLE=1 的流程里【从来没有 acquire 过】,所以恒为假:无条件 release 会去
 * 重新初始化一个本来就是 460800 的串口,还会把日志闸再"打开"一次,
 * 都是没必要的动作,而且正好落在退模式这一拍上。 */
#if S3_FIXED_ACT_ENABLE
#define S3_HOLDS_VOICE_UART(stage)   (0)
#else
#define S3_HOLDS_VOICE_UART(stage)   ((stage) >= S3_SIGNAL)
#endif

/* 起步前等车停稳:滤波实测速度(脉冲/5ms)进 EPS 且连续保持 TICKS 拍
 * (30 拍 = 0.3s;MAX 300 拍 = 3s,都按 10ms/拍)。
 * 倒车复现 stop 后车不是立刻静止,带着残速起步会多冲出去一截。
 * MAX 是保底,等不到停稳也要往下走 —— 灯光/鸣笛那一段还得做。
 * 【这三个值在 kart_motion.h 还有一份同值副本】MOTION_S3_PAUSE_EPS /
 * _TICKS / _MAX_TICKS 同样是 4.0 / 30 / 300,干的是同一件事(等车停稳),
 * 区别只在谁用:那份给 kart_motion 内部固定动作的段间停顿,这份给
 * kart_mission 的阶段交接。两份都在跑,改停稳判据必须两处一起改,
 * 只改一处的后果是两段等待悄悄不一致,现场表现为"有时多冲一截"。 */
#define S3_FIXED_SETTLE_EPS      (4.0f)
#define S3_FIXED_SETTLE_TICKS    (30U)
#define S3_FIXED_SETTLE_MAX_TICKS (300U)

/* 整段固定动作的超时(10ms/拍 → 3000 拍 = 30s)。
 * 2.8m 来回 @0.9m/s 约 7s,含停稳和回正给到 30s。
 * 【超时不进 S3_FAULT】动作没做完也要放行到 S3_SIGNAL:
 * 灯光/鸣笛是规则要分的项,不能被这一段拖没。 */
#define S3_FIXED_TICKS           (3000U)

/* 视觉结果最大可复用拍数(10ms/拍)。兜底目标是采集链断了还拿旧方位
 * 角打方向 —— 相机状态不能代替它,RUNNING 判据来自 VSYNC,DMA 停了仍是 RUNNING。
 *
 * 【为什么给到 100U】10U(100ms) 是按"视觉同步跑、相机 30FPS 约 3.3 拍一帧"定的。
 * 现在识别搬到 core3,单帧要 ~360ms = ~36 拍,而年龄是按【结果帧】计的,
 * 再给 10U 就是每一帧都必定超龄 → vis 恰好在大部分拍变 NULL → kart_follow
 * 每帧掉一次 LOST,转向反而比搬核前更乱。
 * 100U = 1s 约为单帧耗时的 3 倍,既能容下 core3 正常节奏,又能在采集链
 * 真断时 1s 内兜底。【若日后把单帧耗时降下来,这个值要跟着降】——
 * 判据看 VOFA CH5(core3 单帧 ms),本值取它的 3 倍除以 10。
 * 【看 CH5 之前先确认日志分档】CH5 = core3 单帧耗时只在出厂 43 通道档
 * (LOG_PROFILE_S3=1)成立;51 通道全量档的 CH5 是左轮实测速度,拿它算这个值
 * 会得出完全无关的数。与 kart_follow.h / kart_playback.h 那两处是同一个坑。 */
#define S3_VISION_MAX_AGE_TICKS (100U)

/* 语音返回 GOTO 段的【第二道】超时闸(10ms/拍 → 3500 拍 = 35s)。
 * GOTO 自己已经有超距(25m)+ 超时(3000 拍)兜底,这里再加一层是因为:
 * 那两个判据都在 kart_motion 内部,若 GOTO 状态机因为某个几何出口写错而卡在
 * 某段不推进(状态既不是 DONE 也不是 FAULT),kart_mission 会永远停在
 * S2_RETURN_GOTO 等一个不会来的结果 —— 车在场地里一直开。
 * 取 3500 > 3000:正常情况永远由 GOTO 自己的闸先响,这一道只在"GOTO 失灵"时生效。 */
#define S2_RETURN_GOTO_TICKS   (3500U)

/* 初始化:模式置 IDLE,并执行一次统一停机(保证上电无残留输出)。 */
void kart_mission_init(void);

/* 切换顶层模式:内部 exit 旧模式 → enter 新模式。 */
void kart_mission_set_mode(kart_mission_mode_t mode);

/* 10ms 一拍调(cpu0_main.c 的 kart_task_10ms):按当前模式分发 loop。 */
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
 * 按当前位姿启动等于把 kart_odom 的累计漂移一次性归零。代价是要人动手、慢。
 * 【前提】车必须真的摆正 —— 摆位误差会原样变成整条路径的平移误差。 */
uint8 kart_mission_subject2_start_return_here(uint8 slot_index);

#endif
