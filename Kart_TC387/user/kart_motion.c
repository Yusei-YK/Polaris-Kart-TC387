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

/* -------------------- 转角下发:唯一出口 --------------------
 * 全模块所有打角都必须走这里,统一叠转向中位偏置。
 * 实测"真正走直"的 delta 不在 0 而在约 -25(见 KART_MOTION_DELTA_CENTER_OFS),
 * 不补的话所有【左右对称的动作】都会一边大一边小:
 *   转圈 ±950 变成 左975/右925 => 顺时针半径比逆时针大 7cm(用户实车观察到了);
 *   蛇形左右摆幅不等 => 整条轨迹净漂,收尾再修也压不住。
 * 【别在别处直接调 kart_steer_set_target_delta()】—— 漏一处就漏一个偏置。 */
static void motion_set_delta(float delta)
{
    kart_steer_set_target_delta(delta + KART_MOTION_DELTA_CENTER_OFS);
}

/* 航向误差(度,规整 ±180),定义为【目标 - 实测】= yaw0 - yaw,与航向环同惯例。
 * 【只用相对量】比赛中途不能 reset,绝对 yaw 会漂几十度;本函数只比 yaw0,
 * 而 yaw0 是本条动作启动那拍 latch 的,所以长时间漂移完全不影响。
 * get_relative_angle(now, aim) 返回 aim-now(kart_calc.c:93),所以参数顺序是 (yaw, yaw0)。 */
static float motion_yaw_err(void)
{
    return get_relative_angle(kart_imu_get_yaw(), motion_yaw0);
}

/* 航向修正打角(比例),本地算,【不动共享航向环 head_pid】。
 * 前进段也走这里(不再用 head_pid):head_pid 的输出被直接写进 target_delta,
 * 绕开 motion_set_delta 拿不到中位偏置,而且纯 P + 内环静摩擦死区叠起来
 * 会让小误差压根不修 —— 这就是"前行十米修的很少"的成因,详见 .h。
 *
 * 符号推导(err = yaw0 - yaw):
 *   前进 Δyaw = +k*delta*Δs。err>0 表示车头还差 err 才回到目标,需要 Δyaw>0,
 *              => delta = +KP*err。
 *   后退 Δs<0,同一打角的航向变化整体反号 => delta = -KP*err = SIGN*KP*err。
 *   直接把前进那套用到倒车 = 正反馈,越倒越歪。
 * 【实车若发现越修越歪】只翻 KART_MOTION_REV_YAW_SIGN,别动 KP。
 * 限幅后叠中位偏置仍远在软限位(+1103/-1048)内。 */
static float motion_yaw_corr_delta(uint8 reverse)
{
    float corr = KART_MOTION_YAW_KP * motion_yaw_err();

    if(reverse)
    {
        corr *= KART_MOTION_REV_YAW_SIGN;
    }

    if(corr >  KART_MOTION_YAW_LIMIT) corr =  KART_MOTION_YAW_LIMIT;
    if(corr < -KART_MOTION_YAW_LIMIT) corr = -KART_MOTION_YAW_LIMIT;
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
        swung *= KART_MOTION_REV_YAW_SIGN;
    }

    if((swung * motion_snake_sign) >= KART_MOTION_SNAKE_YAW ||
       (kart_odom_get_dist() - motion_snake_half_d0) >= KART_MOTION_SNAKE_MAX_HALF)
    {
        motion_snake_sign    = -motion_snake_sign;
        motion_snake_half_d0 = kart_odom_get_dist();
    }

    motion_set_delta(motion_snake_sign * KART_MOTION_SNAKE_DELTA);
}

/* 后轮出力:两种模式收在一处,避免 8 个 case 各写一遍 #if。
 *   开环(KART_MOTION_OPENLOOP_REAR=1):固定 duty 直下发,不跑速度环 PID。
 *   闭环(=0):走速度环,行为与改造前完全一致(A/B 对照回退用)。
 * 两种模式都必须 set_enable(1) —— enable 是"后轮有没有主人"的唯一开关,
 * 全工程三处仲裁(kart_control.c / isr.c:70 / cpu0_main.c:103)都判它,
 * 不置 1 后轮会被清成 0(遥控挡位/失联/deadman 也是改这个开关来急停的)。
 * duty 符号 = 方向:正前进、负后退。 */
static void motion_set_rear(int16 open_duty, float closed_speed)
{
#if KART_MOTION_OPENLOOP_REAR
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

/* -------------------- 对外接口 -------------------- */
void kart_motion_init(void)
{
    motion_phase = MOTION_IDLE;
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
    motion_reverse = 0;
    motion_phase = MOTION_IDLE;
}

/* 动作正常做完:先停后轮,再【原地】把方向盘拉回中位,到位后才真正 stop()。
 * 【为什么必须有这一段】实车反馈"左转右转顺时针逆时针转结束后轮胎还是打死的
 *   状态,没有回正":原来做完直接 stop(),转向电机断电停在打死位。后果是下一条
 *   命令带着打死的方向盘起步(仿真前进十米终点横偏 0.21m,方向盘在中位只有
 *   0.001m),反方向命令更要先跑 2000 计数 ≈1.1s 才开始对。连续下命令必踩。
 * 后轮此刻已经停了,所以是原地回轮,车不会走出去。
 * 回正期间 motion_phase != IDLE => is_busy()=1 => 下一条语音命令在队列里等,
 *   不会踩着歪轮子起步(kart_voice.c:265 已经按 is_busy 串行排队)。 */
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

    switch(voice_cmd)
    {
        case KART_VOICE_CMD_FWD_10M:
            motion_speed = +KART_MOTION_SPEED;
            /* 【不用共享航向环 head_pid】改用本地保向修正,和倒车那条同一套。
             * 原因(2026-07-26 实车"修的很少、后边走斜"):head_pid 的输出被直接
             * 写进 target_delta、绕开 motion_set_delta 拿不到中位偏置;又是纯 P,
             * 稳态必须站在 delta=-25 => 常驻航向误差 25/30≈0.83°,只要 25 计数,
             * 而转向内环静摩擦死区是 63 计数 => 方向盘压根不动。详见 .h。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_reverse = 0;
            motion_set_delta(motion_yaw_corr_delta(0));
            motion_set_rear(KART_MOTION_DUTY_FWD, motion_speed);
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
            motion_set_rear(KART_MOTION_DUTY_BACK, motion_speed);
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
            motion_set_rear(+KART_MOTION_DUTY_SNAKE, motion_speed);
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
            motion_set_rear(-KART_MOTION_DUTY_SNAKE, motion_speed);
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
            motion_set_rear(KART_MOTION_DUTY_CIRCLE, motion_speed);
            motion_phase = MOTION_CIRCLE;
            break;

        case KART_VOICE_CMD_CW_CIRCLE:
            motion_speed = +KART_MOTION_SPEED;
            /* 顺时针:固定打右角(-)。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            motion_set_delta(-KART_MOTION_CIRCLE_DELTA);
            motion_set_rear(KART_MOTION_DUTY_CIRCLE, motion_speed);
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
            motion_set_rear(KART_MOTION_DUTY_TURN, motion_speed);
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
            motion_set_rear(KART_MOTION_DUTY_TURN, motion_speed);
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

    /* deadman 急停:遥控失联或三段拨低挡 → 立即停(与 playback 一致)。 */
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
             * 导致后边走斜了"。定因见 .h:head_pid 绕开中位偏置 + 纯 P 常驻 0.83° 误差,
             * 而这点误差只要 25 计数,落在转向内环 63 计数的静摩擦死区里 => 不修。
             * 现在改成和倒车同一套本地修正(用户已验证倒车稳),KP=25 让 2.5° 就出死区。 */
            motion_set_delta(motion_yaw_corr_delta(0));
            if(dist >= KART_MOTION_FWD_DIST)
            {
                motion_finish();
            }
            break;

        case MOTION_BACK:
            /* 【实车现象】原来这里每拍写死 target_delta=0,结果倒车十米朝左歪、
             * 方向盘就停在偏的位置不修 —— 因为"真正走直"的 delta 不是 0 而是约 -25,
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
            if(fabsf(motion_yaw_err()) <= KART_MOTION_SNAKE_END_TOL ||
               (kart_odom_get_dist() - motion_snake_end_d0) >= KART_MOTION_SNAKE_END_MAX_DIST)
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

        case MOTION_CENTER:
            /* 收车回正:后轮已停,只等转向内环把方向盘拉回中位。
             * 目标每拍重下发一次(不依赖 stop 那一拍的残留),两个出口:
             *   到位(实测转角进 TOL)—— 正常;
             *   等够 TICKS —— 保底,防卡住/顶限位时永远 busy 阻塞后续语音命令。
             * 【比较的是 meas_delta 和中位偏置】motion_set_delta(0) 下发的是 OFS,
             * 所以"到位"是 |meas - OFS| 小,不是 |meas| 小。 */
            motion_set_delta(0.0f);
            motion_center_ticks++;
            if(fabsf(kart_steer_get_meas_delta() - KART_MOTION_DELTA_CENTER_OFS)
                   <= KART_MOTION_CENTER_TOL ||
               motion_center_ticks >= KART_MOTION_CENTER_TICKS)
            {
                kart_motion_stop();     /* 真正关内环、回 IDLE */
            }
            break;

        default:
            kart_motion_stop();
            break;
    }
}
