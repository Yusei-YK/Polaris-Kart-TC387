#include "kart_motion.h"
#include "kart_voice.h"
#include "kart_odom.h"
#include "kart_imu.h"
#include "kart_calc.h"
#include "kart_steer_ctrl.h"
#include "kart_control.h"
#include "kart_remote.h"
#include <math.h>

/*
 * 科目二语音运动控制实现 —— 见 kart_motion.h 头注释。
 * 状态机每拍推进,只设目标(转角/航向/速度)+使能,实际输出由主循环控制环产生。
 */

/* -------------------- 内部状态机 -------------------- */
typedef enum
{
    MOTION_IDLE = 0,
    MOTION_FWD,             /* 直行前进(航向环保向) */
    MOTION_BACK,            /* 直行后退(内环锁中位) */
    MOTION_SNAKE_FWD,       /* 蛇形前进 */
    MOTION_SNAKE_BACK,      /* 蛇形后退 */
    MOTION_SNAKE_END,       /* 蛇形收尾:停止摆动,把航向拉回起始值再停 */
    MOTION_CIRCLE,          /* 转圈(固定打角,累计 yaw 到 360) */
    MOTION_TURN_APPROACH,   /* 左右转:先直行 approach */
    MOTION_TURN_ROTATE,     /* 左右转:原地转向到目标角 */
    MOTION_GOTO_REV,        /* 摆位①:目标在车后方,先倒一段把车头拧过来 */
    MOTION_GOTO_TURN,       /* 摆位②:前进打死,把前置点转到车前方 */
    MOTION_GOTO_DRIVE,      /* 摆位③:追前置点 */
    MOTION_GOTO_AXIS,       /* 摆位④:沿目标轴线纯跟踪,横向+航向同时收敛 */
    MOTION_S3_OUT,          /* 科目三盲盒任务①:出库直行前进(航向环保向) */
    MOTION_S3_PAUSE,        /* 科目三盲盒任务②:停稳(硬换向会冲过头,详见 .h) */
    MOTION_S3_IN,           /* 科目三盲盒任务③:倒车回库(同 MOTION_BACK 那套修正) */
    MOTION_CENTER,          /* 收车:后轮已停,原地把方向盘拉回中位再关内环 */
} motion_phase_t;

static motion_phase_t motion_phase = MOTION_IDLE;

static float motion_dist0   = 0.0f;     /* 起始累计路程基准(米) */
static float motion_yaw0    = 0.0f;     /* 起始航向(度,直行保向用) */
static float motion_yaw_prev = 0.0f;    /* 上一拍航向(累计 yaw 用) */
static float motion_yaw_accum = 0.0f;   /* 累计转过角度(度,绝对值判完成) */

static float motion_speed   = 0.0f;     /* 本动作速度(带符号,脉冲/5ms) */
static float motion_turn_delta  = 0.0f; /* 转弯打角(带方向符号) */

static uint8 motion_reverse       = 0;  /* 当前动作方向:1=后退(航向修正符号要反) */
static float motion_snake_end_d0  = 0.0f;/* 收尾段起始路程(保底超时判据用) */

static float motion_snake_sign    = 1.0f;/* 蛇形当前摆向:+1=打左、-1=打右 */
static float motion_snake_half_d0 = 0.0f;/* 本摆起始路程(米),保底翻转用 */

static uint16 motion_center_ticks = 0;   /* 回正段已等拍数(超时兜底) */

/* 科目三盲盒任务段间停车:pause 已等拍数(超时兜底)与"速度已进 EPS"的连续拍数。 */
static uint16 motion_s3_pause_ticks = 0;
static uint16 motion_s3_hold_ticks  = 0;

/* -------------------- GOTO 摆位状态 --------------------
 * 目标位姿 + 前置点(目标沿 tyaw 反方向退 LEAD 米)在 start 时算一次就固定,
 * 不每拍重算 —— 它们是场地上的固定几何,重算只会把 kart_odom 抖动引进来。 */
static float motion_goto_tx   = 0.0f;   /* 目标位置 x(米,kart_odom 世界系) */
static float motion_goto_ty   = 0.0f;
static float motion_goto_tyaw = 0.0f;   /* 目标航向(度,IMU 绝对值) */
static float motion_goto_lx   = 0.0f;   /* 前置点 x */
static float motion_goto_ly   = 0.0f;
static float motion_goto_lead_s = 0.0f; /* 前置点沿目标轴线的纵坐标(= -lead) */
static uint8 motion_goto_behind = 0;    /* 已经"退到前置点后方"过(见 DRIVE 段) */
static float motion_goto_rev_d0 = 0.0f; /* 倒车预摆段起始路程(米) */
static uint16 motion_goto_ticks = 0;    /* 摆位已跑拍数(超时兜底) */
static kart_motion_goto_state_t motion_goto_state = KART_MOTION_GOTO_NONE;

/* -------------------- 转角下发:唯一出口 --------------------
 * 全模块所有打角都必须走这里,统一叠转向中位偏置。
 * 【当前偏置是 0,本函数等于直通】MOTION_DELTA_CENTER_OFS 引到 kart_calib.h 的
 * KART_STEER_CENTER_OFS = 0.0f —— 2026-07-30 重标中位后新中值正好是硬限位
 * 几何中点,左右软限对称 ±1064,不需要补。
 * 【那也别删这个函数】重标出非零偏置的那天只改 kart_calib.h 一处、全模块跟随;
 * 绕过它直接调 kart_steer_set_target_delta() 就漏一个偏置。
 * 下面是重标之前(偏置约 -25)为什么必须补的记录,机制没变,只是值成了 0:
 * 不补的话所有【左右对称的动作】都会一边大一边小:
 *   转圈 ±950 变成 左975/右925 => 顺时针半径比逆时针大 7cm(用户实车观察到了);
 *   蛇形左右摆幅不等 => 整条轨迹净漂,收尾再修也压不住。
 * 【别在别处直接调 kart_steer_set_target_delta()】—— 漏一处就漏一个偏置。 */
static void motion_set_delta(float delta)
{
    kart_steer_set_target_delta(delta + MOTION_DELTA_CENTER_OFS);
}

/* 航向误差(度,规整 ±180),定义为【目标 - 实测】= yaw0 - yaw,与航向环同惯例。
 * 【只用相对量】比赛中途不能 reset,绝对 yaw 会漂几十度;本函数只比 yaw0,
 * 而 yaw0 是本条动作启动那拍 latch 的,所以长时间漂移完全不影响。
 * get_relative_angle(now, aim) 返回 aim-now,所以参数顺序是 (yaw, yaw0)。
 * 【2026-09-06 订正】函数在 kart_calc.c 里,原注释写的 :93 只是那一节的分隔线,
 * 函数本体在 :97 —— 改成按函数名找,行号会漂。 */
static float motion_yaw_err(void)
{
    return get_relative_angle(kart_imu_get_yaw(), motion_yaw0);
}

/* 航向修正打角(比例),本地算,【不动共享航向环 head_pid】。
 * 前进段也走这里(不再用 head_pid)。理由见 .h 的保向修正那节 —— 注意那节
 * 已复核过:当年"绕开中位偏置 + 常驻 0.83° 落在死区里"那套算例随中位偏置
 * 归零而作废,现在维持本地修正靠的是"head_pid 为科目一/四/遥控共用、不能为
 * 科目二动它"加实车已跑通。别再复述作废的那套数。
 *
 * 符号推导(err = yaw0 - yaw):
 *   前进 Δyaw = +k*delta*Δs。err>0 表示车头还差 err 才回到目标,需要 Δyaw>0,
 *              => delta = +KP*err。
 *   后退 Δs<0,同一打角的航向变化整体反号 => delta = -KP*err = SIGN*KP*err。
 *   直接把前进那套用到倒车 = 正反馈,越倒越歪。
 * 【实车若发现越修越歪】只翻 MOTION_REV_YAW_SIGN,别动 KP。
 * 限幅后叠中位偏置仍远在软限位(对称 ±1064)内。 */
static float motion_yaw_corr_delta(uint8 reverse)
{
    float corr = MOTION_YAW_KP * motion_yaw_err();

    if(reverse)
    {
        corr *= MOTION_REV_YAW_SIGN;
    }

    if(corr >  MOTION_YAW_LIMIT) corr =  MOTION_YAW_LIMIT;
    if(corr < -MOTION_YAW_LIMIT) corr = -MOTION_YAW_LIMIT;
    return corr;
}

/* 蛇形一拍:打死到 ±DELTA 不动,【等航向摆够角度】才翻,不看走了多少路。
 * 为什么不能用路程取模:方向盘是有刷电机,从一边打死转到另一边要约 1 秒
 * (实测 1800 计数/秒),半周期 1.6m 里一半以上耗在"方向盘还在路上",
 * 时间一到就翻的话越摆越小、左右还不对称 => 净漂。实车现象就是
 * "第一次摆幅大,后边小,摆不回来,就又走斜了"。详见 .h 的推导。
 *
 * 【判据必须是"绕本条动作起始航向 yaw0 的 ±SNAKE_YAW 带",不是"每摆各转 SNAKE_YAW"】
 *   每摆各转 SNAKE_YAW 的话:0→+15,翻,+15→0,翻,0→+15 …… 航向只在 0..+15
 *   之间摆、均值 +7.5° => 整条轨迹单向漂。这正是上一版 fmodf 相位那个坑的等价形式。
 *   用固定带就是 -15..+15、均值 0,自然不漂。第一摆只从 0 摆到 +15(半个幅度,
 *   仿真 0.62m),之后每摆走满 1.65m —— 这是对的,不是缺陷。
 * 保底:某个摆万一因打滑/顶限位一直摆不到角度,走够 MAX_HALF 也强制翻,
 *   防单摆卡死变成一路大弧线。 */
static void motion_snake_step(uint8 reverse)
{
    /* 相对本条动作起始航向已摆过的角度(度)。motion_yaw_err() = yaw0 - yaw,取反即 yaw - yaw0。
     * 后退时同一打角产生的航向变化整体反号,乘 SIGN 归一成
     * "朝着当前打角驱动的方向为正",这样翻转判据前进/后退共用一套。 */
    float swung = -motion_yaw_err();

    if(reverse)
    {
        swung *= MOTION_REV_YAW_SIGN;
    }

    if((swung * motion_snake_sign) >= MOTION_SNAKE_YAW ||
       (kart_odom_get_dist() - motion_snake_half_d0) >= MOTION_SNAKE_MAX_HALF)
    {
        motion_snake_sign    = -motion_snake_sign;
        motion_snake_half_d0 = kart_odom_get_dist();
    }

    motion_set_delta(motion_snake_sign * KART_MOTION_SNAKE_DELTA);
}

/* 后轮出力:两种模式收在一处,避免 8 个 case 各写一遍 #if。
 *   开环(MOTION_OPENLOOP_REAR=1):固定 duty 直下发,不跑速度环 PID。
 *   闭环(=0):走速度环,行为与改造前完全一致(A/B 对照回退用)。
 * 两种模式都必须 set_enable(1) —— enable 是"后轮有没有主人"的唯一开关,
 * 全工程三处仲裁都判它:kart_control.c 的速度环本体、isr.c 的 5ms 中断
 * cc60_pit_ch0_isr、cpu0_main.c 的 kart_task_5ms()。
 * 【2026-09-06 订正】原注释在 isr.c 那处写的 :70、cpu0_main.c 那处写的 :103,
 * 两个行号都已失效(真实位置是 :74 和 :91,而 :70 那行现在是
 * kart_multicore_imu_update)。三处仲裁这件事本身没变,只是改成按函数定位。
 * 不置 1 后轮会被清成 0(遥控挡位/失联/deadman 也是改这个开关来急停的)。
 * duty 符号 = 方向:正前进、负后退。 */
static void motion_set_rear(int16 open_duty, float closed_speed)
{
#if MOTION_OPENLOOP_REAR
    kart_control_set_open_duty(open_duty);
    (void)closed_speed;
#else
    kart_control_set_target(closed_speed);
    (void)open_duty;
#endif
    kart_control_set_enable(1);
}

/* 累计 yaw:把本拍航向增量(wrap 到 ±180)累加,不受 ±180 回绕影响。 */
static void motion_accum_yaw(void)
{
    float now = kart_imu_get_yaw();
    motion_yaw_accum += get_relative_angle(motion_yaw_prev, now);   /* now-prev,规整±180 */
    motion_yaw_prev = now;
}

/* -------------------- GOTO 摆位:几何小工具 --------------------
 * 方位角误差:从【车头】到指定点还要转多少度(+ = 点在车头左边、需左转)。
 * get_angle(now,aim) 用的就是 kart_odom 的坐标约定(x东y北、y轴当0度基准、
 * 顺时针为负),与 kart_imu_get_yaw() 同一套,所以两者可以直接相减。
 * get_relative_angle(now,aim)=aim-now 已规整 ±180 => 参数顺序 (yaw, bearing)。
 * 【位姿必须由调用方传进来】同一拍里要算方位角、距离、轴线投影三样,
 *   各自再取一次快照就会拿到被 5ms 中断改过的不同帧,判据之间自相矛盾。 */
static float motion_goto_bearing_err(const kart_odom_snapshot_t *kart_odom, float px, float py)
{
    Point_2D cur, aim;

    cur.x = kart_odom->x;
    cur.y = kart_odom->y;
    aim.x = px;
    aim.y = py;

    /* 点几乎就在脚下时方位角无意义(atan2(0,0)),返回 0 让调用方靠距离判据退出。 */
    if(get_distance(cur, aim) < 0.02f)
    {
        return 0.0f;
    }

    return get_relative_angle(kart_odom->yaw, get_angle(cur, aim));
}

/* 车到指定点的距离(米)。 */
static float motion_goto_dist_to(const kart_odom_snapshot_t *kart_odom, float px, float py)
{
    Point_2D cur, aim;

    cur.x = kart_odom->x;
    cur.y = kart_odom->y;
    aim.x = px;
    aim.y = py;
    return get_distance(cur, aim);
}

/* 方位角 → 打角。前进段:点在左(err>0)要打左(delta>0),故 delta=+KP*err。
 * 倒车段整体反号(与 motion_yaw_corr_delta 同一套推导)。
 * 限幅用 GOTO_STEER_MAX(950,允许打死)而不是走直线用的 YAW_LIMIT(300)。 */
static float motion_goto_steer(float bearing_err, uint8 reverse)
{
    float delta = MOTION_GOTO_KP * bearing_err;

    if(reverse)
    {
        delta *= MOTION_REV_YAW_SIGN;
    }

    if(delta >  MOTION_GOTO_STEER_MAX) delta =  MOTION_GOTO_STEER_MAX;
    if(delta < -MOTION_GOTO_STEER_MAX) delta = -MOTION_GOTO_STEER_MAX;
    return delta;
}

/* 当前位置沿【目标轴线】的纵坐标 s:以目标点为原点、tyaw 方向为正。
 *   车还没走到目标 → s<0;越过目标 → s>0 => 到位判据就是 s >= -AXIS_STOP。
 * 轴线单位向量按 kart_odom 约定(dx=-sin*ds, dy=+cos*ds)由 tyaw 给出:fwd=(-sin,+cos)。
 * 【不需要横向偏差 lat】AXIS 段瞄的是轴线上的前视点,方位角误差里已经同时
 *   包含了横向偏差和航向偏差,再单独算一个 lat 是死代码。 */
static float motion_goto_axis_s(const kart_odom_snapshot_t *kart_odom)
{
    float rad = degree_to_rad(motion_goto_tyaw);
    float dx  = kart_odom->x - motion_goto_tx;
    float dy  = kart_odom->y - motion_goto_ty;

    return -sinf(rad) * dx + cosf(rad) * dy;
}

/* 轴线上的前视点:从车在轴线上的投影脚点再往前 LD 米,取轴线上那个点。
 * 【为什么瞄轴线上的点而不是直接瞄目标点】瞄目标点是"追点",到点时航向由
 *   来路决定、和 tyaw 没关系;瞄轴线前视点就是标准 Pure Pursuit,横向偏差
 *   和航向偏差【同时】按 e^(-s/Ld) 收敛,到点时车已经躺在轴线上朝着 tyaw。
 * s 是当前投影(目标点为 0、车在其后为负),所以前视点纵坐标 = s + LD。 */
static void motion_goto_axis_aim(float s, float *ax, float *ay)
{
    float rad = degree_to_rad(motion_goto_tyaw);
    float sn  = sinf(rad);
    float cs  = cosf(rad);
    float sa  = s + MOTION_GOTO_LD;

    /* 【2026-07-29 删掉了这里的 if(sa > 0.0f) sa = 0.0f;】
     * 那句话把前视点夹在目标点上,后果是【前视距离随 s→0 一起塌到 0】:
     * 车快到位时前视点就压在车轮底下,motion_goto_bearing_err() 的 <0.02m 保护
     * 直接返回方位角 0,打角冻结在最后一次的值 —— 恰恰在"对齐"最要紧的最后
     * 1.2m 失去控制。离线仿真(304 个起始位姿,判据 pos<=0.5m 且 yaw<=10deg)
     * 只改这一处:76/304 → 186/304,再配 LEAD=6.0 到 261/304。
     * 不夹会怎样:越过目标(s>0)后前视点落在目标【前方】,车继续往前追。这没问题,
     * 因为出口判据 s >= -AXIS_STOP 在同一拍就成立、AXIS 段立刻结束,那个前视点
     * 最多被用一拍。夹住反而是在换取一个不存在的好处。 */
    *ax = motion_goto_tx + (-sn) * sa;
    *ay = motion_goto_ty + ( cs) * sa;
}

/* -------------------- 对外接口 -------------------- */
void kart_motion_init(void)
{
    motion_phase = MOTION_IDLE;
    motion_goto_state = KART_MOTION_GOTO_NONE;
}

uint8 kart_motion_is_busy(void)
{
    return (motion_phase != MOTION_IDLE) ? 1 : 0;
}

/* 硬停机:后轮停 + 转向内环关 + 状态机立刻回 IDLE。
 * 对外语义不变(kart_mission.c:43 的 mission_stop_all 和 deadman 都要这个):
 *   切模式/急停必须【立刻】交还转向、且立刻不 busy,否则残留 busy 会阻塞语音分发。
 * 【方向盘的物理位置这里管不了】—— angle_enable 一关,kart_steer_ctrl_update()
 *   直接 power_set_steer_duty(0) 并 return(kart_steer_ctrl.c:58),转向电机断电,
 *   有刷电机不自动回中,方向盘就停在当前位置。急停/切模式时这是对的(人要接管);
 *   动作【正常做完】要回正,走 motion_finish() 的 MOTION_CENTER 段。 */
void kart_motion_stop(void)
{
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);          /* 内部一并清 open_loop,后轮回闭环默认态 */
    kart_control_clear_open_loop();      /* 再显式清一次:不依赖 set_enable 的内部实现 */
    kart_steer_use_fwd_gains();          /* 恢复前进增益组,别把倒车组漏给科目一/四/遥控 */
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);
    /* 这里【故意不走 motion_set_delta】:停机是把转向交还给其他模块(科目一/四/遥控),
     * 要留干净的 0,不能把科目二的中位偏置漏出去。此时 angle_enable 已为 0,不下发。 */
    kart_steer_set_target_delta(0.0f);

    /* 摆位【还在跑】就被硬停(deadman/切模式/急停)=> 判 FAULT,绝不能让
     * kart_mission 事后读到 DONE 去启动复现 —— 那会在错误的位姿上放出一整条
     * 返程路径。已经 DONE/NONE 的状态不动:DONE 要留给下一拍的交接读走。 */
    if(motion_goto_state == KART_MOTION_GOTO_RUNNING)
    {
        motion_goto_state = KART_MOTION_GOTO_FAULT;
    }

    motion_reverse = 0;
    motion_phase = MOTION_IDLE;
}

kart_motion_goto_state_t kart_motion_get_goto_state(void)
{
    return motion_goto_state;
}

uint8 kart_motion_start_goto(float tx, float ty, float tyaw)
{
    kart_odom_snapshot_t kart_odom;
    float rad, bear;

    if(kart_motion_is_busy())
    {
        return 0;                       /* 忙不打断(与 kart_motion_start 同约定) */
    }

    kart_odom_get_snapshot(&kart_odom);

    motion_goto_tx   = tx;
    motion_goto_ty   = ty;
    motion_goto_tyaw = tyaw;

    /* 前置点 = 目标位姿沿 tyaw 【反】方向退 lead 米。kart_odom 约定下车头方向
     * 单位向量是 (-sin, +cos),所以往后退是 (+sin, -cos)*lead。
     *
     * 【lead 不是常数,要保证前置点落在车的"轴线后方"】
     *   记 s0 = 车此刻沿目标轴线的纵坐标(目标点为 0,车在其后为负)。
     *   若直接取 lead=LEAD 而车已经站在 s0 > -LEAD(挤在引入段里、甚至已越过
     *   目标),前置点就在车【前方】,DRIVE 段一进去距离判据可能已经满足,
     *   AXIS 段拿不到引入段直接判到位 => 交出去的是一个横向偏差没收敛的位姿。
     *   那比 FAULT 更糟:kart_mission 会照着这个歪位姿放出一整条返程路径。
     *   所以 s0 > -LEAD 时把前置点再往后推 (s0 + LEAD) 米,让车永远"从后方
     *   沿轴线进场",引入段长度也永远 >= LEAD。代价是要绕回去(s0=0 绕 5m,
     *   s0=+3 绕 11m),这正是阿克曼车该走的小回环,MAX_DIST=25m 包得住。 */
    rad = degree_to_rad(tyaw);
    {
        float s0   = motion_goto_axis_s(&kart_odom);
        float lead = MOTION_GOTO_LEAD;

        /* 【2026-07-29 把 > 改成 >=,并给推后量加 1 个 AXIS_STOP 的余量】
         * 原来 s0 == -LEAD 这个【边界】上条件不成立、一点不推 => 引入段长度
         * 恰好 0…LEAD 之间都可能出现,极端情况引入段是零长的:车站在前置点上,
         * DRIVE 段的距离判据当拍就满足,AXIS 段没有任何收敛距离就判到位。
         * 交出去的是横向偏差原样保留的位姿,而 kart_mission 会照着它放出一整条
         * 返程路径。所以边界必须算作"要推",且推完再多留一点,别卡在等号上。 */
        if(s0 >= -MOTION_GOTO_LEAD)
        {
            lead += (s0 + MOTION_GOTO_LEAD) + MOTION_GOTO_AXIS_STOP;
        }
        motion_goto_lx   = tx + sinf(rad) * lead;
        motion_goto_ly   = ty - cosf(rad) * lead;
        motion_goto_lead_s = -lead;     /* 前置点在轴线上的纵坐标 */
        motion_goto_behind = 0;
    }

    /* 路程基准:超距兜底(MAX_DIST)用。
     * 【GOTO 不用 yaw0/yaw_accum】那两个是"保持起始航向 / 累计转了多少度"的判据,
     * 摆位的判据全是【相对目标位姿的几何量】(方位角、轴线投影),与起始航向无关。
     * 仍然刷一遍是为了别把上一条动作的残值留给下一条动作复用。 */
    motion_dist0     = kart_odom_get_dist();
    motion_yaw0      = kart_imu_get_yaw();
    motion_yaw_prev  = motion_yaw0;
    motion_yaw_accum = 0.0f;
    motion_goto_ticks = 0;
    motion_goto_state = KART_MOTION_GOTO_RUNNING;

    /* 已经站在目标位姿上(位置进 ARRIVE、航向进 TURN_TOL)→ 直接判 DONE,车不动。
     * 【为什么要这条】不加的话会先跑去 2.5m 外的前置点再开回来,白走 5m;
     * 更糟的是那 5m 本身要消耗 kart_odom 精度预算,等于为"已经对好了"付出误差。
     * 台架上原地测这条命令时也是这个分支(车不该乱动)。 */
    if(motion_goto_dist_to(&kart_odom, tx, ty) <= MOTION_GOTO_ARRIVE &&
       fabsf(get_relative_angle(kart_odom.yaw, tyaw)) <= MOTION_GOTO_TURN_TOL)
    {
        motion_goto_state = KART_MOTION_GOTO_DONE;
        motion_phase = MOTION_IDLE;     /* 没启动电机,不必走 CENTER 回正 */
        return 1;
    }

    bear = motion_goto_bearing_err(&kart_odom, motion_goto_lx, motion_goto_ly);

    /* 前置点在车【后方】很多 → 先倒车预摆(三点掉头前半程),比在前方画一个
     * 直径 3.1m 的圆更省场地。EN=0 时直接跳过,退化成纯前进掉头。 */
#if MOTION_GOTO_REV_EN
    if(fabsf(bear) >= MOTION_GOTO_REV_BEAR)
    {
        kart_steer_use_back_gains();    /* 真要打角的倒车必须换增益组,否则转角环振 */
        kart_steer_set_head_enable(0);
        kart_steer_set_angle_enable(1);
        motion_reverse = 1;
        motion_goto_rev_d0 = motion_dist0;
        motion_set_delta(motion_goto_steer(bear, 1));
        motion_set_rear(MOTION_DUTY_GOTO_REV, -KART_MOTION_SPEED);
        motion_phase = MOTION_GOTO_REV;
        return 1;
    }
#endif

    kart_steer_use_fwd_gains();
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(1);
    motion_reverse = 0;
    motion_set_delta(motion_goto_steer(bear, 0));
    motion_set_rear(MOTION_DUTY_GOTO, +KART_MOTION_SPEED);

    /* 方位角已经进容差就不用先转了,直接进 DRIVE(省掉一次无意义的打死)。 */
    motion_phase = (fabsf(bear) <= MOTION_GOTO_TURN_TOL) ? MOTION_GOTO_DRIVE
                                                             : MOTION_GOTO_TURN;
    return 1;
}

/* 动作正常做完:先停后轮,再【原地】把方向盘拉回中位,到位后才真正 stop()。
 * 【为什么必须有这一段】实车反馈"左转右转顺时针逆时针转结束后轮胎还是打死的
 *   状态,没有回正":原来做完直接 stop(),转向电机断电停在打死位。后果是下一条
 *   命令带着打死的方向盘起步(仿真前进十米终点横偏 0.21m,方向盘在中位只有
 *   0.001m),反方向命令更要先跑 2000 计数 ≈1.1s 才开始对。连续下命令必踩。
 * 后轮此刻已经停了,所以是原地回轮,车不会走出去。
 * 回正期间 motion_phase != IDLE => is_busy()=1 => 下一条语音命令在队列里等,
 *   不会踩着歪轮子起步 —— kart_voice.c 的 kart_voice_dispatch() 开头就按
 *   is_busy 串行排队。【2026-09-06 订正】原注释写的 :265 现在是
 *   kart_voice_get_frame_count() 的左花括号,真实位置在 :290。 */
static void motion_finish(void)
{
    /* 后轮立刻停:回正段绝不能还带着出力。 */
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);
    kart_control_clear_open_loop();

    /* 转向留着:关外环、保内环,目标给中位,让内环自己把轮子拽回来。
     * 增益回前进组 —— 回正是原地转方向盘,不是倒车工况,用倒车组(Kp=5)会很慢。 */
    kart_steer_use_fwd_gains();
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(1);
    motion_set_delta(0.0f);

    motion_center_ticks = 0;
    motion_phase = MOTION_CENTER;
}

/* GOTO 摆位一拍(四段共用一个入口:三段之间的切换只是几何条件,分散到
 * kart_motion_update 的 switch 里会把同一套投影算三遍)。
 * dist = 本次摆位已走的总路程(米),由调用方算好传进来。 */
static void motion_goto_update(float dist)
{
    kart_odom_snapshot_t kart_odom;
    float bear, s, ax, ay;

    /* 兜底:超距/超时 → FAULT 停机。GOTO 的出口全是几何条件,若 kart_odom 漂到
     * 离谱或 tyaw 给反了,几何条件可能【永远】不满足 => 车在场地里一直绕。
     * 宁可停机让人接管,也不能变成一条没有终点的动作。 */
    motion_goto_ticks++;
    if(dist >= MOTION_GOTO_MAX_DIST || motion_goto_ticks >= MOTION_GOTO_TICKS)
    {
        motion_goto_state = KART_MOTION_GOTO_FAULT;
        kart_motion_stop();             /* 已是 FAULT,stop 里的 RUNNING 分支不会再覆盖 */
        return;
    }

    /* 整拍一致位姿:方位角、距离、轴线投影三个判据必须同源,否则互相矛盾。 */
    kart_odom_get_snapshot(&kart_odom);

    switch(motion_phase)
    {
        case MOTION_GOTO_REV:
            /* ① 倒车预摆:目标在车后方,先倒着把车头拧过来(三点掉头前半程)。
             * 倒车时打角对【车头】的作用整体反号,motion_goto_steer(.,1) 已处理。
             * 两个出口:方位角拧进 REV_EXIT(正常),或倒够 REV_DIST(保底,
             * 防符号不对时越倒越偏 —— 那时把 GOTO_REV_EN 改 0 即可绕开本段)。 */
            bear = motion_goto_bearing_err(&kart_odom, motion_goto_lx, motion_goto_ly);
            motion_set_delta(motion_goto_steer(bear, 1));
            if(fabsf(bear) <= MOTION_GOTO_REV_EXIT ||
               (kart_odom.dist_sum - motion_goto_rev_d0) >= MOTION_GOTO_REV_DIST)
            {
                /* 转前进:增益换回前进组,后轮换向。 */
                kart_steer_use_fwd_gains();
                motion_reverse = 0;
                motion_set_delta(motion_goto_steer(bear, 0));
                motion_set_rear(MOTION_DUTY_GOTO, +KART_MOTION_SPEED);
                motion_phase = MOTION_GOTO_TURN;
            }
            break;

        case MOTION_GOTO_TURN:
            /* ② 前进打死转,把前置点转到车前方。大角度旋转【集中在这一段】,
             * 之后就不再需要转大角度 —— 这是"到点再转头会毁位置"的规避方式。 */
            bear = motion_goto_bearing_err(&kart_odom, motion_goto_lx, motion_goto_ly);
            motion_set_delta(motion_goto_steer(bear, 0));
            if(fabsf(bear) <= MOTION_GOTO_TURN_TOL)
            {
                motion_phase = MOTION_GOTO_DRIVE;
            }
            break;

        case MOTION_GOTO_DRIVE:
            /* ③ 追前置点。这段只管把车开到目标轴线的【后段延长线】上,不管航向
             * —— 航向交给 ④。到 ARRIVE 半径内就切轴线跟踪。
             *
             * 【出口为什么不能只判距离】车若绕着前置点画圈(打死半径 1.56m,
             *   而 ARRIVE 只有 0.5m,几何上完全可能永远进不去这个圈),
             *   就会一直转到 MAX_DIST 判 FAULT。所以补一条"已经越过前置点
             *   (s >= lead_s)"的兜底。
             * 【为什么要 behind 这个闩】start 那一拍车可能就在前置点前方
             *   (s > lead_s),此时若直接用 s >= lead_s 判,第一拍就跳 AXIS,
             *   引入段长度为 0 => 横向偏差压根没收敛就交出去,比 FAULT 更糟。
             *   所以先要求车真的退到过前置点后方一次,这条兜底才生效。
             *   起点就在后方(正常情况)时 behind 第一拍即置 1,行为不变。 */
            bear = motion_goto_bearing_err(&kart_odom, motion_goto_lx, motion_goto_ly);
            motion_set_delta(motion_goto_steer(bear, 0));
            s = motion_goto_axis_s(&kart_odom);
            if(s <= motion_goto_lead_s)
            {
                motion_goto_behind = 1;
            }
            if(motion_goto_dist_to(&kart_odom, motion_goto_lx, motion_goto_ly) <= MOTION_GOTO_ARRIVE ||
               (motion_goto_behind && s >= motion_goto_lead_s))
            {
                motion_phase = MOTION_GOTO_AXIS;
            }
            break;

        case MOTION_GOTO_AXIS:
            /* ④ 沿目标轴线纯跟踪:瞄"轴线上前视 LD 米"的点,横向偏差与航向偏差
             * 【同时】按 e^(-s/Ld) 收敛。走到 s >= -AXIS_STOP(沿轴线到达目标)
             * 就 motion_finish():停车 + 原地回正方向盘,再由 kart_mission 交接。
             * 【DONE 在这里置,不在 CENTER 段末】CENTER 是回轮,车已经停了,
             * 摆位精度此刻就已定死;等到 CENTER 结束再置只会让交接晚 0.x 秒。
             * 但 is_busy 要到 CENTER 走完才落 => kart_mission 必须【同时】
             * 判 DONE 和 !is_busy 才启动复现(否则会踩着歪轮子起步)。 */
            s = motion_goto_axis_s(&kart_odom);
            motion_goto_axis_aim(s, &ax, &ay);
            bear = motion_goto_bearing_err(&kart_odom, ax, ay);
            motion_set_delta(motion_goto_steer(bear, 0));
            if(s >= -MOTION_GOTO_AXIS_STOP)
            {
                motion_goto_state = KART_MOTION_GOTO_DONE;
                motion_finish();
            }
            break;

        default:
            kart_motion_stop();
            break;
    }
}

/* 科目三盲盒任务入口:出库 2.8m → 停稳 → 倒回 2.8m → 回正。
 * 前半段和 kart_motion_start 的公共基准完全一样(路程/航向基准 + 作废 GOTO 结果),
 * 后半段等于 KART_VOICE_CMD_FWD_10M 那条,只把结束判据换成 S3 的距离宏。
 * 【不做成语音命令码】它不由语音触发,挂进 kart_motion_start 的 switch 只会
 * 让语音有机会误触发这条动作。 */
uint8 kart_motion_start_s3_fixed(void)
{
    if(kart_motion_is_busy())
    {
        return 0;
    }

    motion_dist0     = kart_odom_get_dist();
    motion_yaw0      = kart_imu_get_yaw();
    motion_yaw_prev  = motion_yaw0;
    motion_yaw_accum = 0.0f;
    motion_goto_state = KART_MOTION_GOTO_NONE;

    motion_speed = +KART_MOTION_SPEED;
    kart_steer_use_fwd_gains();
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(1);
    motion_reverse = 0;
    motion_set_delta(motion_yaw_corr_delta(0));
    motion_set_rear(MOTION_DUTY_FWD, motion_speed);

    motion_s3_pause_ticks = 0;
    motion_s3_hold_ticks  = 0;
    motion_phase = MOTION_S3_OUT;
    return 1;
}

uint8 kart_motion_start(uint8 voice_cmd)
{
    if(kart_motion_is_busy())
    {
        return 0;                       /* 忙不打断,交给队列串行 */
    }

    /* 公共基准:路程/航向清基准,累计 yaw 归零。 */
    motion_dist0     = kart_odom_get_dist();
    motion_yaw0      = kart_imu_get_yaw();
    motion_yaw_prev  = motion_yaw0;
    motion_yaw_accum = 0.0f;

    /* 上一次摆位的结果作废:又做了一条固定动作,车已经离开摆好的位姿,
     * 残留的 DONE 绝不能被后来的交接逻辑读到当成"还站在目标位姿上"。 */
    motion_goto_state = KART_MOTION_GOTO_NONE;

    switch(voice_cmd)
    {
        case KART_VOICE_CMD_FWD_10M:
            motion_speed = +KART_MOTION_SPEED;
            /* 【不用共享航向环 head_pid】改用本地保向修正,和倒车那条同一套。
             * 原因(2026-07-26 实车"修的很少、后边走斜")见 .h 的保向修正那节
             * —— 那里已复核:原来"delta=-25 => 常驻 0.83° => 落在 63 计数死区里"
             * 这套算例随中位偏置归零而作废。现在的理由是 head_pid 为科目一/四/
             * 遥控共用不能动,加本地这套实车已跑通。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse = 0;
            motion_set_delta(motion_yaw_corr_delta(0));
            motion_set_rear(MOTION_DUTY_FWD, motion_speed);
            motion_phase = MOTION_FWD;
            break;

        case KART_VOICE_CMD_BACK_10M:
            motion_speed = -KART_MOTION_SPEED;
            /* 倒车同样本地保向(用户已验证"倒车十米挺稳的")。
             * 增益切倒车组:锁中位这条其实不敏感,但保持"倒车必换组"的一致性。 */
            kart_steer_use_back_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse = 1;
            motion_set_delta(motion_yaw_corr_delta(1));
            motion_set_rear(MOTION_DUTY_BACK, motion_speed);
            motion_phase = MOTION_BACK;
            break;

        case KART_VOICE_CMD_SNAKE_FWD_10M:
            motion_speed = +KART_MOTION_SPEED;
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse       = 0;
            motion_snake_sign    = +1.0f;       /* 第一摆打左 */
            motion_snake_half_d0 = motion_dist0;
            motion_set_delta(motion_snake_sign * KART_MOTION_SNAKE_DELTA);
            motion_set_rear(+MOTION_DUTY_SNAKE, motion_speed);
            motion_phase = MOTION_SNAKE_FWD;
            break;

        case KART_VOICE_CMD_SNAKE_BACK_10M:
            motion_speed = -KART_MOTION_SPEED;
            /* 蛇形后退是【真要打角的倒车】—— 增益必须切倒车组,否则转角环会振
             * (侧偏力在倒车时变自增强,前进标定的 Kp 偏大)。 */
            kart_steer_use_back_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse       = 1;
            motion_snake_sign    = +1.0f;
            motion_snake_half_d0 = motion_dist0;
            motion_set_delta(motion_snake_sign * KART_MOTION_SNAKE_DELTA);
            motion_set_rear(-MOTION_DUTY_SNAKE, motion_speed);
            motion_phase = MOTION_SNAKE_BACK;
            break;

        case KART_VOICE_CMD_CCW_CIRCLE:
            motion_speed = +KART_MOTION_SPEED;
            /* 逆时针:固定打左角(+),边走边累计 yaw 到 360°。
             * 注意"打死"= 把目标转角设到接近软限位、让转角内环把它稳住,
             * 不是给转向电机固定 duty(那会一路怼机械硬限位堵转)。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_set_delta(+KART_MOTION_CIRCLE_DELTA);
            motion_set_rear(MOTION_DUTY_CIRCLE, motion_speed);
            motion_phase = MOTION_CIRCLE;
            break;

        case KART_VOICE_CMD_CW_CIRCLE:
            motion_speed = +KART_MOTION_SPEED;
            /* 顺时针:固定打右角(-)。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_set_delta(-KART_MOTION_CIRCLE_DELTA);
            motion_set_rear(MOTION_DUTY_CIRCLE, motion_speed);
            motion_phase = MOTION_CIRCLE;
            break;

        case KART_VOICE_CMD_TURN_LEFT:
            motion_speed = +KART_MOTION_SPEED;
            motion_turn_delta = +KART_MOTION_TURN_DELTA;    /* 左转打左角(+) */
            /* 先直行 approach(>2m):保向和 MOTION_FWD 同一套本地修正,不用 head_pid。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse = 0;
            motion_set_delta(motion_yaw_corr_delta(0));
            motion_set_rear(MOTION_DUTY_TURN, motion_speed);
            motion_phase = MOTION_TURN_APPROACH;
            break;

        case KART_VOICE_CMD_TURN_RIGHT:
            motion_speed = +KART_MOTION_SPEED;
            motion_turn_delta = -KART_MOTION_TURN_DELTA;    /* 右转打右角(-) */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse = 0;
            motion_set_delta(motion_yaw_corr_delta(0));
            motion_set_rear(MOTION_DUTY_TURN, motion_speed);
            motion_phase = MOTION_TURN_APPROACH;
            break;

        default:
            return 0;                   /* 非运动命令 */
    }

    return 1;
}

void kart_motion_update(void)
{
    if(motion_phase == MOTION_IDLE)
    {
        return;
    }

    /* deadman 急停:遥控失联或三段拨低挡 → 立即停(与 kart_playback 一致)。 */
    if(!kart_remote_is_online() ||
       kart_remote_get_sw3() == KART_REMOTE_SW3_L)
    {
        kart_motion_stop();
        return;
    }

    float dist = kart_odom_get_dist() - motion_dist0;

    switch(motion_phase)
    {
        case MOTION_FWD:
            /* 【实车现象】原来这里靠共享航向环 head_pid 保向,结果"前行十米修的很少,
             * 导致后边走斜了"。现象是真的;当年记的定因(常驻 0.83° 落在 63 计数
             * 死区里)已随中位偏置归零而作废,见 .h 的保向修正那节。
             * 【顺带纠一处】"KP=25 让 2.5° 就出死区"不是优点:63 计数 ÷ 25 是 2.5°,
             * 而 head_pid 的 30 计数/度只要 2.1° —— 本地这套的死区角反而更大。
             * 现在维持本地修正,靠的是 head_pid 不能为科目二调,以及实车已跑通。 */
            motion_set_delta(motion_yaw_corr_delta(0));
            if(dist >= KART_MOTION_FWD_DIST)
            {
                motion_finish();
            }
            break;

        case MOTION_BACK:
            /* 【实车现象】原来这里每拍写死 target_delta=0,结果倒车十米朝左歪、
             * 方向盘就停在偏的位置不修 —— 因为当时"真正走直"的 delta 不是 0 而是
             * 约 -25(重标中位后这个值已是 0,下面是当时那笔账):
             * 常驻 25 计数偏角 => R≈60m,十米下来转 9°、横向偏 0.7m。
             * 前进那条没这个毛病,是因为它开了航向环把偏置吃掉了;倒车这条
             * 原本【完全没有任何航向反馈】,偏置就一路积出去。
             * 现在:中位偏置由 motion_set_delta 统一补,再叠倒车航向比例修正。 */
            motion_set_delta(motion_yaw_corr_delta(1));
            if(dist >= KART_MOTION_BACK_DIST)
            {
                motion_finish();
            }
            break;

        case MOTION_S3_OUT:
            /* 出库段:与 MOTION_FWD 同一套本地保向修正。 */
            motion_set_delta(motion_yaw_corr_delta(0));
            if(dist >= KART_MOTION_S3_OUT_DIST)
            {
                /* 只断后轮出力,转向内环留着(下一拍就要把轮子往倒车修正位摆)。
                 * 不走 motion_finish():那会进 MOTION_CENTER 结束整条动作。 */
                kart_control_set_target(0.0f);
                kart_control_set_enable(0);
                kart_control_clear_open_loop();
                motion_s3_pause_ticks = 0;
                motion_s3_hold_ticks  = 0;
                motion_phase = MOTION_S3_PAUSE;
            }
            break;

        case MOTION_S3_PAUSE:
            /* 后轮已断出力,车靠惯性滑停。这一段【提前】按倒车符号摆方向盘:
             * 方向盘是有刷电机,从出库修正位摆到倒车修正位要百毫秒级,
             * 借滑停这段时间摆完,倒车一起步就是对的角度。 */
            motion_set_delta(motion_yaw_corr_delta(1));
            motion_s3_pause_ticks++;
            if(fabsf(kart_control_get_meas()) <= MOTION_S3_PAUSE_EPS)
            {
                motion_s3_hold_ticks++;
            }
            else
            {
                motion_s3_hold_ticks = 0;
            }
            if(motion_s3_hold_ticks >= MOTION_S3_PAUSE_TICKS ||
               motion_s3_pause_ticks >= MOTION_S3_PAUSE_MAX_TICKS)
            {
                /* 回库距离从"停稳这一刻"重新起算:里程是单调路程,
                 * 滑停多走的那几十厘米不能算进回库的 2.8m,否则回不到库里。
                 * motion_yaw0 【不重新 latch】:要回的是出发时的车头方向。 */
                motion_dist0 = kart_odom_get_dist();
                motion_speed = -KART_MOTION_SPEED;
                kart_steer_use_back_gains();
                kart_steer_set_head_enable(0);
                kart_steer_set_angle_enable(1);
                motion_reverse = 1;
                motion_set_delta(motion_yaw_corr_delta(1));
                motion_set_rear(MOTION_DUTY_BACK, motion_speed);
                motion_phase = MOTION_S3_IN;
            }
            break;

        case MOTION_S3_IN:
            /* 回库段:与 MOTION_BACK 同一套倒车保向修正。 */
            motion_set_delta(motion_yaw_corr_delta(1));
            if(dist >= KART_MOTION_S3_IN_DIST)
            {
                motion_finish();
            }
            break;

        case MOTION_SNAKE_FWD:
        case MOTION_SNAKE_BACK:
            /* 打死到 ±DELTA,【摆够航向角度】才翻(不是走够路程)。
             * 详见 motion_snake_step():路程取模顶不住方向盘的转速上限,
             * 会越摆越小 + 净漂,正是实车"第一次摆幅大、后边小、摆不回来"。 */
            motion_snake_step(motion_reverse);
            if(dist >= KART_MOTION_SNAKE_DIST)
            {
                /* 【不直接停】停那一刻航向还残留几度(仿真 -4.9°),车是斜着结束的。
                 * 转 SNAKE_END 先摆正。 */
                motion_snake_end_d0 = kart_odom_get_dist();
                motion_phase = MOTION_SNAKE_END;
            }
            break;

        case MOTION_SNAKE_END:
            /* 收尾:停止摆动,只把航向拉回 yaw0。
             * 两个出口:误差进 TOL(正常),或再走 MAX_DIST(保底,防符号/增益
             * 不对时修不回来变成没有终点的动作)。 */
            motion_set_delta(motion_yaw_corr_delta(motion_reverse));
            if(fabsf(motion_yaw_err()) <= MOTION_SNAKE_END_TOL ||
               (kart_odom_get_dist() - motion_snake_end_d0) >= MOTION_SNAKE_END_MAX_DIST)
            {
                motion_finish();
            }
            break;

        case MOTION_CIRCLE:
            /* 固定打角边走边累计 yaw,到一整圈停。 */
            motion_accum_yaw();
            if(fabsf(motion_yaw_accum) >= KART_MOTION_CIRCLE_ANGLE)
            {
                motion_finish();
            }
            break;

        case MOTION_TURN_APPROACH:
            /* approach 段保向走直线(同 MOTION_FWD 的本地修正),到距离切原地转向段。 */
            motion_set_delta(motion_yaw_corr_delta(0));
            if(dist >= KART_MOTION_TURN_APPROACH)
            {
                motion_yaw_prev  = kart_imu_get_yaw();
                motion_yaw_accum = 0.0f;
                motion_set_delta(motion_turn_delta);
                motion_phase = MOTION_TURN_ROTATE;
            }
            break;

        case MOTION_TURN_ROTATE:
            /* 保持转弯打角,累计 yaw 到目标角(方向转正)停。 */
            motion_accum_yaw();
            motion_set_delta(motion_turn_delta);
            if(fabsf(motion_yaw_accum) >= KART_MOTION_TURN_ANGLE)
            {
                motion_finish();
            }
            break;

        /* ===== GOTO 摆位四段(几何推导见 kart_motion.h) ===== */
        case MOTION_GOTO_REV:
        case MOTION_GOTO_TURN:
        case MOTION_GOTO_DRIVE:
        case MOTION_GOTO_AXIS:
            motion_goto_update(dist);
            break;

        case MOTION_CENTER:
            /* 收车回正:后轮已停,只等转向内环把方向盘拉回中位。
             * 目标每拍重下发一次(不依赖 stop 那一拍的残留),两个出口:
             *   到位(实测转角进 TOL)—— 正常;
             *   等够 TICKS —— 保底,防卡住/顶限位时永远 busy 阻塞后续语音命令。
             * 【比较的是 meas_delta 和中位偏置】motion_set_delta(0) 下发的是 OFS,
             * 所以"到位"是 |meas - OFS| 小,不是 |meas| 小。 */
            motion_set_delta(0.0f);
            motion_center_ticks++;
            if(fabsf(kart_steer_get_meas_delta() - MOTION_DELTA_CENTER_OFS)
                   <= MOTION_CENTER_TOL ||
               motion_center_ticks >= MOTION_CENTER_TICKS)
            {
                kart_motion_stop();     /* 真正关内环、回 IDLE */
            }
            break;

        default:
            kart_motion_stop();
            break;
    }
}
