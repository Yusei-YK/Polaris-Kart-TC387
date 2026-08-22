#include "kart_playback.h"
#include "kart_record.h"
#include "kart_odom.h"
#include "kart_calc.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"
#include "kart_remote.h"
#include "kart_imu.h"
#include "kart_params.h"
#include <math.h>

static uint8  playback_running = 0;
static kart_playback_result_t playback_result = KART_PLAYBACK_RESULT_NONE;
static uint16 playback_index = 0;
static float  playback_target_yaw = 0.0f;

/* 方案B:正向复现里当前分段模式(带死区滞回)。0=前进段(Pure Pursuit),
 * 1=倒车段(开环回放录制打角)。死区内维持上一拍值,防边界抖动翻转使能位。 */
static uint8  playback_seg_reverse = 0;

/* ===== 科目三反向复现状态(不走 Pure Pursuit)=====
 * 命名沿用 openloop 是历史包袱:方案 0 现在已有【航向闭环】(航向 P 纠偏),
 * 真正开环的只剩【横向位置】—— 位置误差从头到尾没被测量,也没进反馈环,
 * 所以航向再准也会带着 0.5~1m 的横向偏移走到终点。方案 1 补的就是这一环。
 * 变量/函数名不改:改名要动 8 处调用点,收益只有可读性,风险不划算。 */
static uint8  playback_openloop_mode = 0;   /* =1 时 poll 走科目三倒车分支 */
static float  playback_ol_dist0 = 0.0f;     /* 倒车起点累计里程基准 */
static float  playback_ol_total = 0.0f;     /* 原路全程总里程 */
static uint16 playback_ol_index = 0;        /* 里程查表游标(单调递减) */

static float play_origin_x = 0.0f;
static float play_origin_y = 0.0f;
static float play_origin_yaw = 0.0f;

/* 诊断快照：poll 每拍写入，仅供日志读出，不参与控制。 */
static float playback_cur_x = 0.0f;
static float playback_cur_y = 0.0f;
static float playback_aim_x = 0.0f;
static float playback_aim_y = 0.0f;

/* ===== 速度剖面缓冲 =====
 * 与 record_buf 同索引、同长度。放 cpu2_dsram 与 record_buf 同域(1500*4=6KB)。
 * 单位:最终存"脉冲/5ms"(与速度环同量纲),生成过程中间态是 m/s(见 build)。 */
#if defined(__TASKING__)
#pragma section all "cpu2_dsram"
#endif
static float playback_prof[KART_RECORD_MAX_WAYPOINTS];
#if defined(__TASKING__)
#pragma section all restore
#endif

static uint16 playback_prof_n = 0;      /* 剖面有效点数,0=未生成(退回录制速度) */

/* 倒车复现结束的主动刹停状态(见 kart_playback.h 的 PLAYBACK_BRAKE_* 注释)。
 * playback_braking=1 期间 playback_running 保持 1,所以任务层等的 is_running
 * 会自动把刹停等完 —— kart_mission.c 一行不用改。 */
static uint8  playback_braking    = 0;
static uint16 playback_brake_tick = 0;
static uint16 playback_brake_hold = 0;

static float kart_playback_wrap180(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle <= -180.0f) angle += 360.0f;
    return angle;
}

static void kart_playback_poll_openloop(void);      /* 科目三倒车方案0:里程查表(下方实现) */
static void kart_playback_poll_closedloop(void);    /* 科目三倒车方案1:位置闭环(下方实现) */
static void kart_playback_poll_brake(void);         /* 倒车结束主动刹停(下方实现) */

static void kart_playback_complete(void)
{
    /* 开环倒车(科目三)单独走刹停:直接 stop() 是断电滑行,见 PLAYBACK_BRAKE_* 注释。
     * 其余科目(正向复现)行为完全不变,仍是原来的 stop() + 置完成。 */
    if(playback_openloop_mode)
    {
        playback_braking    = 1;
        playback_brake_tick = 0;
        playback_brake_hold = 0;
        kart_control_set_target(0.0f);
        return;                     /* 保持 playback_running=1,下一拍进刹停分支 */
    }
    kart_playback_stop();
    playback_result = KART_PLAYBACK_RESULT_COMPLETED;
}

/* 刹停一拍:持续命令 0 速,等实测进死区并连续保持,或超时兜底,然后才真正关环。
 * 转向故意不动:保持最后一拍的 delta 和已使能的内环,让车刹直线;
 * 回中/切回正向增益统一由最后那声 kart_playback_stop() 做。 */
static void kart_playback_poll_brake(void)
{
    kart_control_set_target(0.0f);

    playback_brake_tick++;
    if(fabsf(kart_control_get_meas()) <= PLAYBACK_BRAKE_EPS) playback_brake_hold++;
    else                                                     playback_brake_hold = 0;

    if(playback_brake_hold >= PLAYBACK_BRAKE_HOLD ||
       playback_brake_tick >= PLAYBACK_BRAKE_MAX_TICKS)
    {
        playback_braking = 0;
        kart_playback_stop();                       /* 到这里才断电 + 回默认状态 */
        playback_result = KART_PLAYBACK_RESULT_COMPLETED;
    }
}

/* 该点录制速度(脉冲/5ms):左右取平均。只用于判前进/倒车段,不决定快慢。 */
static float kart_playback_rec_v(const kart_waypoint_t *wp, uint16 i)
{
    return 0.5f * (wp[i].v_left + wp[i].v_right);
}

/* 第 i 段弦长(m),i..i+1。不用 kart_record 的 dist_buf:剖面要的是【相邻两点直线距离】,
 * 而 dist_buf 记的是沿行驶轨迹的累计里程 —— 打滑/原地摆头时它增长而 x/y 不动,
 * 拿它当弦长会把曲率算小(kappa=dyaw/arc 的分母虚高),弯道限速就偏松。 */
static float kart_playback_seg_len(const kart_waypoint_t *wp, uint16 i)
{
    float dx = wp[i + 1].x - wp[i].x;
    float dy = wp[i + 1].y - wp[i].y;
    return sqrtf(dx * dx + dy * dy);
}

/* ===== 生成速度剖面(只在 kart_playback_start 调一次)=====
 * 见 kart_playback.h 顶部"速度剖面"注释。三步:
 *   ① 曲率限速 v=sqrt(a_lat/|kappa|),kappa=dyaw/ds 由 wp[].yaw 沿弧长中心差分;
 *   ② 端点锚 0:最后一点必须能停下,否则满速撞终点;
 *   ③ 反向传播 v[i]=min(v[i], sqrt(v[i+1]²+2*a_brake*ds)) —— 入弯/进终点前提前减速。
 * 出弯加速不在这里限,交给 kart_control 的目标速度斜坡(时间域)。
 * 倒车段(录制速度为负)照抄录制速度,且作为传播屏障:前进段的刹车不跨过换向点传。 */
static void kart_playback_build_profile(const kart_waypoint_t *wp, uint16 n)
{
    float scale = kart_params_get(PARAM_PB_SCALE);
    float vmax  = kart_params_get(PARAM_PB_VMAX)  * PLAYBACK_V_TO_MS;   /* m/s */
    float vmin  = kart_params_get(PARAM_PB_VMIN)  * PLAYBACK_V_TO_MS;   /* m/s */
    float alat  = kart_params_get(PARAM_PB_ALAT);                            /* m/s² */
    uint16 i;

    playback_prof_n = 0;
    if(n < 2 || n > KART_RECORD_MAX_WAYPOINTS) return;
    /* 总开关(菜单 PB Prof):关 → 不生成,poll 退回取录制速度的旧行为。 */
    if(kart_params_get(PARAM_PB_PROF) < 0.5f) return;
    if(alat < 0.1f) alat = 0.1f;

    /* ---- ① 曲率限速 ---- */
    for(i = 0; i < n; i++)
    {
        float rec = kart_playback_rec_v(wp, i);
        uint16 lo, hi, k;
        float arc = 0.0f, dyaw, kappa, v;

        if(rec < -KART_PLAYBACK_REV_SPEED_EPS)
        {
            /* 倒车段:曲率限速那套(横向抓地)对倒车没意义,只按录制速度乘倍率。
             * 【为什么需要倍率】照抄的 rec 是录制时的【实测】速度,不是当时的指令:
             * 07-28 科一日志里录制指令 -28 实测只有 -16.4(59%,静摩擦+速度环稳态误差),
             * 回放再拿 -16.4 当目标又只跑出 -14.6 —— 每过一代掉一档,过去没有旋钮能补。
             * 出厂 1.0 = 原样照抄,行为与改动前逐位相同;要快现场推 PB RevScl。
             * 【代价】倒车是开环回放打角,提速会按比例放大里程/横向漂移,一次加一档。 */
            playback_prof[i] = rec * PLAYBACK_V_TO_MS
                               * kart_params_get(PARAM_PB_REVSCL);
            continue;
        }

        lo = (i > PLAYBACK_KAPPA_WIN) ? (uint16)(i - PLAYBACK_KAPPA_WIN) : 0u;
        hi = (uint16)(i + PLAYBACK_KAPPA_WIN);
        if(hi > n - 1) hi = n - 1;

        for(k = lo; k < hi; k++) arc += kart_playback_seg_len(wp, k);

        dyaw = kart_playback_wrap180(wp[hi].yaw - wp[lo].yaw) * 0.01745329252f;
        if(dyaw < 0.0f) dyaw = -dyaw;

        if(arc < 0.02f)
        {
            /* 窗内几乎没走动。两种成因,处理相反,必须分开:
             *   ① 原地转了角度(起步/换向前后原地摆头):kappa→∞,是真正的最急处,
             *      不能给 vmax,否则原地打满方向再全油门;
             *   ② 位置和角度都没变(静止段重复采样):曲率无信息,放满,让斜坡去管起步。 */
            v = (dyaw > 0.017f) ? vmin : vmax;      /* 0.017rad ≈ 1° */
        }
        else
        {
            kappa = dyaw / arc;                                  /* 1/m,已取正 */
            v = (kappa < 1e-3f) ? vmax : sqrtf(alat / kappa);     /* 近直线:不限 */
        }

        v *= scale;                          /* 整体胆量旋钮:形状对了,只缩放幅值 */
        if(v > vmax) v = vmax;
        if(v < vmin) v = vmin;               /* 地板放在传播前,才不会抹掉终点收油 */
        playback_prof[i] = v;
    }

    /* ---- ② 端点锚 0 ---- */
    playback_prof[n - 1] = 0.0f;

    /* ---- ②b 换向点减速(2026-07-29)----
     * 换向前必须减到接近停:车不可能不经过零速就从前进变倒车。
     * 【要修的缺陷】③ 的换向点屏障(vn<0||vc<0 → continue)只防"倒车段的负速被
     * 当成刹车目标",但顺带把"前进段最后一点该刹下来"也一起跳过了 —— 换向点前那
     * 一点保留着曲率算出来的速度,车以该速度冲到换向点,才开始把目标从 +X 拉到负,
     * 而 kart_power 的穿零保护还要先归零+驻留 6 拍,合计约 92ms 才有制动力。
     * 【表现】"倒车有时进得去有时进不去":冲过头多少取决于当次速度/地面,每次落在
     * 换向点之后不同的位置,而倒车段是开环回放录制打角(无位置反馈),起点差 20cm
     * 打同样的角度库就进不去。速度越大飞越远 —— 实车已确认。
     *
     * 【为什么不能直接钉 0】钉 0 会卡死:进度点靠 find_nearest 推进,而它只在车
     * 移动时才会往前挪。目标速度 0 → 车停 → 进度不动 → 目标还是 0,永远换不到
     * 倒车段(换向点是路径中段,不触发 FINISH_DIST 那条终点判定)。
     * 故这里留一个爬行下限 CREEP:慢到冲过头可忽略,又能保证进度点越过换向点。
     * 0.25 m/s × 92ms ≈ 2.3cm 滑行,比开环倒车本身的漂移小一个量级。 */
    for(i = 1; i < n; i++)
    {
        if(playback_prof[i] < 0.0f && playback_prof[i - 1] > 0.0f)
            playback_prof[i - 1] = PLAYBACK_REV_CREEP;
    }

    /* ---- ③ 反向传播(提前刹车)---- */
    for(i = n - 1; i > 0; i--)
    {
        float vn = playback_prof[i];
        float vc = playback_prof[i - 1];
        float ds, lim;

        /* 换向点两侧不互传:倒车段的负速与前进段的正速不在同一条刹车链上。 */
        if(vn < 0.0f || vc < 0.0f) continue;

        ds  = kart_playback_seg_len(wp, (uint16)(i - 1));
        /* 减速度走 kart_params(菜单 PB ABrake):终点/入弯前刹不住就加大。
         * 在循环内读是有意的 —— 剖面只在 kart_playback_start 生成一次,不是热路径。 */
        lim = sqrtf(vn * vn + 2.0f * kart_params_get(PARAM_PB_ABRAKE) * ds);
        if(vc > lim) playback_prof[i - 1] = lim;
    }

    /* ---- 换算回速度环量纲 ---- */
    for(i = 0; i < n; i++) playback_prof[i] *= PLAYBACK_MS_TO_V;

    playback_prof_n = n;
}

void kart_playback_init(void)
{
    playback_running = 0;
    playback_result = KART_PLAYBACK_RESULT_NONE;
    playback_openloop_mode = 0;
    playback_index = 0;
    playback_target_yaw = 0.0f;
    playback_prof_n = 0;
}

/* 正向复现启动的公共部分。两个入口只差【路径钉在哪个原点上】:
 *   use_rec_origin=0(kart_playback_start)          → 钉当前位姿:车在哪路径就从哪开始;
 *   use_rec_origin=1(..._start_at_recorded_origin) → 钉录制原点:路径在场地上位置固定。
 * 除这三个 play_origin_* 之外的一切(剖面/索引/使能/外环初值)完全一致。 */
static uint8 playback_start_common(uint8 use_rec_origin)
{
    kart_odom_snapshot_t kart_odom;
    uint16 n = kart_record_get_count();

    if(n < 2)
    {
        playback_running = 0;
        return 0;                       /* 路径无效，调用者不得推进科目状态。 */
    }

    /* 正向模式 */
    playback_openloop_mode = 0;
    playback_seg_reverse = 0;           /* 方案B:从前进段起步(第3点通常为正速) */

    /* 速度剖面按几何现场生成(约 n*(2*WIN+3) 次 sqrtf,1500 点量级 ~ 几 ms,
     * 只在按下启动键那一刻算一次,不在 poll 里算。生成失败(点数异常)则
     * playback_prof_n=0,poll 自动退回"取录制速度"的旧行为。 */
    kart_playback_build_profile(kart_record_get_waypoints(), n);

    if(use_rec_origin)
    {
        /* 录制原点来自 kart_record(载入槽位时已从 Flash 页头还原)。
         * 不取当前位姿 → poll 里的 cur = 当前位置在【录制坐标系】里的投影,
         * 车没摆到录制起点上时 cur 就不是 (0,0),Pure Pursuit 立刻看到真实的
         * 横向偏差并把车往路径上收。这正是"路径钉在场地上"的含义。 */
        play_origin_x   = kart_record_get_origin_x();
        play_origin_y   = kart_record_get_origin_y();
        play_origin_yaw = kart_record_get_origin_yaw();
    }
    else
    {
        /* 起点位姿一次性取整帧,x/y/yaw 必须同帧,否则复现坐标系原点就是歪的。 */
        kart_odom_get_snapshot(&kart_odom);
        play_origin_x = kart_odom.x;
        play_origin_y = kart_odom.y;
        play_origin_yaw = kart_odom.yaw;
    }

    /* 起点数值退化修复:发车时 cur≈(0,0) 恰是录制第0点,find_nearest 会把
     * index 钉死在 0,速度取起点静止段≈0 → 永远卡原点起不了步。故跳过起点
     * 静止段,从第3点起步给非零初速;不足4点则退回0(短路径由完成判定收尾)。 */
    playback_index = (n > 3) ? 3 : 0;
    playback_result = KART_PLAYBACK_RESULT_NONE;
    playback_running = 1;

    /* 外环目标初始化为当前航向,消除发车瞬间用残留 target_yaw 乱打方向。 */
    playback_target_yaw = kart_imu_get_yaw();
    kart_steer_set_target_yaw(playback_target_yaw);

    /* 正向回放的起始段固定用前进转角增益，不继承上一次倒车状态。 */
    kart_steer_use_fwd_gains();
    kart_steer_set_head_enable(1);
    kart_control_set_enable(1);
    return 1;
}

uint8 kart_playback_start(void)
{
    return playback_start_common(0);
}

uint8 kart_playback_start_at_recorded_origin(void)
{
    return playback_start_common(1);
}

void kart_playback_stop(void)
{
    if(playback_running)
    {
        playback_result = KART_PLAYBACK_RESULT_ABORTED;
    }
    playback_running = 0;
    playback_openloop_mode = 0;
    playback_braking = 0;       /* 刹停态一并清:急停/中途 abort 不能把它漏给下一次 start */
    /* 剖面判废:下次 start 会重算(可能改了 Vmax/Alat),不让旧剖面漏用。 */
    playback_prof_n = 0;
    kart_control_set_enable(0);
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);     /* 开环倒车只开内环,停机一并关掉 */
    kart_steer_use_fwd_gains();          /* 给遥控/下一个正向任务留下统一默认状态 */
    kart_control_set_target(0.0f);
}

/* 最近点前向搜索窗口(点数):只在进度点之后有限窗口内找最近点,
 * 脱线时不会跳到路径远端,且进度单调不后退。 */
#define KART_PB_NEAREST_FORWARD     (40)

/* 从当前进度向前找最近点:在 [playback_index, playback_index+窗口] 内
 * 取距 cur 最近的点作为新进度。单调不后退,保证进度只前进。返回最近点索引。 */
static uint16 kart_playback_find_nearest(Point_2D cur, const kart_waypoint_t *wp, uint16 n)
{
    uint16 i, best = playback_index;
    uint16 end = playback_index + KART_PB_NEAREST_FORWARD;
    float best_d, d;
    Point_2D p;

    if(end >= n) end = n - 1;
    p.x = wp[playback_index].x;
    p.y = wp[playback_index].y;
    best_d = get_distance(cur, p);
    for(i = playback_index + 1; i <= end; i++)
    {
        p.x = wp[i].x;
        p.y = wp[i].y;
        d = get_distance(cur, p);
        if(d < best_d)
        {
            best_d = d;
            best = i;
        }
    }
    return best;
}

/* 从最近点沿路径累加弧长前视:逐段累加相邻点距离,累计到 >= 前视距离 ld
 * 或到末点为止,返回瞄准点索引。ld 由调用者按当前速度自适应算出后传入。
 * 脱线时瞄准点仍沿路径推进,不会卡死。 */
static uint16 kart_playback_find_lookahead(uint16 nearest, const kart_waypoint_t *wp, uint16 n, float ld)
{
    uint16 i;
    float acc = 0.0f;
    Point_2D a, b;

    for(i = nearest; i + 1 < n; i++)
    {
        a.x = wp[i].x;
        a.y = wp[i].y;
        b.x = wp[i + 1].x;
        b.y = wp[i + 1].y;
        acc += get_distance(a, b);
        if(acc >= ld) return i + 1;
    }
    return n - 1;
}

/* 按当前目标速度自适应前视距离:LD = clamp(BASE + GAIN*|v|, MIN, MAX)。
 * GAIN 走 kart_params(菜单可调):提速后如果切内就加大它,画龙就减小。 */
static float kart_playback_lookahead_dist(float v)
{
    float av = (v < 0.0f) ? -v : v;
    float ld = KART_PLAYBACK_LD_BASE + kart_params_get(PARAM_PB_LDGAIN) * av;
    if(ld < KART_PLAYBACK_LD_MIN) ld = KART_PLAYBACK_LD_MIN;
    {
        /* 上限走 kart_params(菜单 PB LdMax):绕桩削顶就调小,高速画龙就调大。
         * 现场必须能改的原因:上限一旦被顶住,PB LdGain 就完全失效(旋钮转了没反应),
         * 只调 GAIN 是调不出来的。 */
        float ldmax = kart_params_get(PARAM_PB_LDMAX);
        if(ld > ldmax) ld = ldmax;
    }
    return ld;
}

/* 纯跟踪:最近点(进度单调) + 弧长前视 + 按进度点判完成 + 速度取进度点。 */
void kart_playback_poll(void)
{
    Point_2D cur;
    uint16 n, nearest, aim_index;
    const kart_waypoint_t *wp;
    float lookahead_yaw;
    float target_v, rec_v;
    float dx, dy;
    float origin_rad, origin_sin, origin_cos;
    kart_odom_snapshot_t kart_odom;

    if(!playback_running) return;

    /* 遥控器是科目一deadman：失联或低挡立即停止复现。 */
    if(!kart_remote_is_online() || kart_remote_get_sw3() == KART_REMOTE_SW3_L)
    {
        kart_playback_stop();
        return;
    }

    /* ===== 科目三倒车分支(两套方案,菜单 S3 OLMode 选)=====
     * 0 = 里程查表索引 + 航向 P 纠偏。已实车验证能完赛(不撞筒,终点横向偏
     *     0.5~1m),故为出厂默认。不做定位/最近点/坐标投影。
     * 1 = 最近点索引 + 航向 P + 横向位置 P(位置闭环)。推导见 kart_playback.h。
     * 两条路径完全独立,方案 1 出问题现场把 OLMode 打回 0 即恢复已验证行为。 */
    if(playback_openloop_mode)
    {
        /* 刹停优先:完成判据已经触发,这一拍只做刹车,不再跑纠偏。 */
        if(playback_braking)
        {
            kart_playback_poll_brake();
            return;
        }
        if(kart_params_get(PARAM_S3_OL_MODE) > 0.5f)
            kart_playback_poll_closedloop();
        else
            kart_playback_poll_openloop();
        return;
    }

    n = kart_record_get_count();
    wp = kart_record_get_waypoints();

    /* 一致快照:x/y 一次性取整帧,避免被 5ms 中断插到半路取到撕裂位置。 */
    kart_odom_get_snapshot(&kart_odom);
    dx = kart_odom.x - play_origin_x;
    dy = kart_odom.y - play_origin_y;

    /* 与录制一致：把世界坐标投影到本次播放起点车体系，x向右、y向前。 */
    origin_rad = play_origin_yaw * 0.01745329252f;
    origin_sin = sinf(origin_rad);
    origin_cos = cosf(origin_rad);
    cur.x =  origin_cos * dx + origin_sin * dy;
    cur.y = -origin_sin * dx + origin_cos * dy;
    playback_cur_x = cur.x;
    playback_cur_y = cur.y;

    /* 最近点:进度单调前进,不后退。 */
    nearest = kart_playback_find_nearest(cur, wp, n);
    playback_index = nearest;

    /* 只有实际进入终点距离圈才算完成。索引到末点但位置仍远，说明末段
     * 跳点或车辆脱线：异常停车，不能把“搜索到末点”冒充“车到终点”。 */
    {
        Point_2D last;
        float last_dist;
        last.x = wp[n - 1].x;
        last.y = wp[n - 1].y;
        last_dist = get_distance(cur, last);
        if(last_dist < KART_PLAYBACK_FINISH_DIST)
        {
            kart_playback_complete();
            return;
        }
        if(nearest >= n - 1)
        {
            kart_playback_stop();       /* result=ABORTED，任务层进入对应FAULT */
            return;
        }
    }

    /* 速度取进度点(最近点),而非前视点:速度跟当前所在弧段,弯道不提前拉满。
     * 剖面已生成 → 用剖面速度(几何算出的,与录制快慢无关);
     * 未生成(点数异常)→ 退回旧行为取录制速度,保证任何情况下都能跑。 */
    rec_v = 0.5f * (wp[nearest].v_left + wp[nearest].v_right);
    target_v = (playback_prof_n == n) ? playback_prof[nearest] : rec_v;
    {
        /* 总钳位走 kart_params(菜单 PB Clamp):剖面速度和录制速度都要过这道闸。
         * 与 PB Vmax 的分工:Vmax 只在生成剖面时限制曲率算出来的目标,
         * 本项是下发速度环前的最后一道,剖面没生成退回录制速度时也管得住。 */
        float clamp = kart_params_get(PARAM_PB_CLAMP);
        if(target_v >  clamp) target_v =  clamp;
        if(target_v < -clamp) target_v = -clamp;
    }

    /* ===== 方案B:按进度点【录制】速度符号分段选转向来源(带死区滞回)=====
     * 录制含倒车段时,Pure Pursuit 航向环对车尾正反馈会打圈。故:
     *   前进段(v> +eps):开航向外环,走 Pure Pursuit;
     *   倒车段(v< -eps):关航向外环,只开转角内环,开环回放录制打角;
     *   死区内(|v|<=eps):维持上一拍模式,防边界反复翻转使能位。
     * 判据必须用 rec_v(录制速度)而非 target_v:剖面里前进段最后几点会被
     * 反向传播压到接近 0(终点收油),若拿它判段会误判成"死区/倒车"翻掉航向环。
     * steer_buf 与 record_buf 同索引,倒车段可直接取录制打角开环回放。 */
    if(rec_v > KART_PLAYBACK_REV_SPEED_EPS)
    {
        if(playback_seg_reverse)
        {
            playback_seg_reverse = 0;
            kart_steer_use_fwd_gains();
        }
    }
    else if(rec_v < -KART_PLAYBACK_REV_SPEED_EPS)
    {
        if(!playback_seg_reverse)
        {
            playback_seg_reverse = 1;
            kart_steer_use_back_gains();
        }
    }

    if(!playback_seg_reverse)
    {
        /* --- 前进段:Pure Pursuit 航向外环(原逻辑) --- */
        /* 前视距离按当前目标速度自适应:高速拉长减小偏移,低速回落防切内。 */
        float ld = kart_playback_lookahead_dist(target_v);
        aim_index = kart_playback_find_lookahead(nearest, wp, n, ld);
        {
            Point_2D aim;
            aim.x = wp[aim_index].x;
            aim.y = wp[aim_index].y;
            playback_aim_x = aim.x;
            playback_aim_y = aim.y;
            lookahead_yaw = get_angle(cur, aim);
        }
        /* get_angle得到局部航向；转回当前里程计/IMU使用的世界航向后再送外环。 */
        playback_target_yaw = kart_playback_wrap180(play_origin_yaw + lookahead_yaw);
        kart_steer_set_head_enable(1);          /* 开外环(连带开内环) */
        kart_steer_set_target_yaw(playback_target_yaw);
    }
    else
    {
        /* --- 倒车段:开环回放录制打角 + 轻量航向 P 纠偏 ---
         * 纯开环打角不保持航向(实测漂 11°),故在录制打角基准上叠加航向纠偏:
         *   参考航向 = play_origin_yaw + wp[nearest].yaw(录制点航向转世界系)
         *   误差 = wrap180(参考 - IMU实测),经 P 增益+反向符号转成打角修正量。
         * 不开航向外环(那是前进阿克曼负反馈),自己算反向纠偏喂内环。 */
        const int16 *kart_steer = kart_record_get_steer();
        float delta = (float)kart_steer[nearest];
        float ref_yaw = kart_playback_wrap180(play_origin_yaw + wp[nearest].yaw);
        float head_err = kart_playback_wrap180(ref_yaw - kart_imu_get_yaw());
        float corr = KART_PLAYBACK_REV_HEAD_SIGN * KART_PLAYBACK_REV_HEAD_KP * head_err;

        /* 钳位走菜单(PB RevCorr),出厂 400 = 原宏值。这是倒车打角能偏离录制值的
         * 全部余量:拐不到位就加,左右摆头就减。现场不必重新烧写。 */
        float corr_max = kart_params_get(PARAM_PB_REVCORR);
        if(corr >  corr_max) corr =  corr_max;
        if(corr < -corr_max) corr = -corr_max;
        delta += corr;

        if(delta > KART_STEER_DELTA_LIMIT_L) delta = KART_STEER_DELTA_LIMIT_L;
        if(delta < KART_STEER_DELTA_LIMIT_R) delta = KART_STEER_DELTA_LIMIT_R;

        kart_steer_set_head_enable(0);          /* 关前进航向外环 */
        kart_steer_set_angle_enable(1);         /* 只开转角内环 */
        kart_steer_set_target_delta(delta);

        /* 诊断:cur 通道记航向误差/纠偏,aim 通道记进度点/最终打角,VOFA 可观测。 */
        playback_cur_x = head_err;
        playback_cur_y = corr;
        playback_aim_x = (float)nearest;
        playback_aim_y = delta;
    }

    kart_control_set_target(target_v);
}

uint8 kart_playback_is_running(void)
{
    return playback_running;
}

kart_playback_result_t kart_playback_get_result(void)
{
    return playback_result;
}

uint16 kart_playback_get_index(void)
{
    return playback_index;
}

float kart_playback_get_target_yaw(void)
{
    return playback_target_yaw;
}

float kart_playback_get_cur_x(void) { return playback_cur_x; }
float kart_playback_get_cur_y(void) { return playback_cur_y; }
float kart_playback_get_aim_x(void) { return playback_aim_x; }
float kart_playback_get_aim_y(void) { return playback_aim_y; }

float kart_playback_get_profile_v(uint16 i)
{
    if(playback_prof_n == 0 || i >= playback_prof_n) return 0.0f;
    return playback_prof[i];
}

uint8 kart_playback_profile_valid(void) { return (playback_prof_n > 0) ? 1u : 0u; }

/* =========================== 科目三反向复现 =========================== */
/* 启动倒车:车头不掉转,直接挂倒挡回放录制打角(索引方式由菜单 S3 OLMode 定)。
 * 前置:车已停在原路终点(停车区),kart_odom 仍在录制世界系(未 reset)。
 * 基准:total=录制全程里程,dist0=当前 kart_odom 累计里程(倒车起点)。
 * 倒车中 kart_odom.dist_sum 是标量(只增不减),故倒退里程 d=kart_odom-dist0 单调增,
 * 映射回原路弧长 s=total-d 单调减,查表游标 playback_ol_index 单调递减。
 * 只开转角内环 + 速度环负速,不开航向外环/Pure Pursuit。1=成功,0=路径无效。 */
uint8 kart_playback_start_openloop_reverse(void)
{
    uint16 n = kart_record_get_count();

    if(n < 2)
    {
        playback_running = 0;
        return 0;                       /* 路径无效,调用者不得推进科目状态。 */
    }

    playback_ol_total = kart_record_get_total_dist();   /* 原路全程总里程 */
    playback_ol_dist0 = kart_odom_get_dist();           /* 倒车起点里程基准 */
    playback_ol_index = n - 1;                          /* 从末点(停车点)倒查 */

    /* 航向纠偏要用的参考系原点:录制起点航向(世界系)。
     * 注意本函数最早不设 play_origin_yaw(纯回放打角不需要位姿),但航向 P 纠偏要把
     * wp[k].yaw(相对录制起点的增量)转回世界系比 IMU,故这里必须补上。
     * 取 kart_record_get_origin_yaw() 而不是当前 kart_odom.yaw:后者是"倒车起点"的
     * 航向,不是"录制起点"的航向,两者差多少就等于整段参考航向偏多少。
     * 2026-07-28:kart_steer/dist/origin_* 已随路径一起进 Flash,故从槽位载入的路径
     * 这三样也是对的,不再限于"倒车紧接同一次录制"。仍要求发车朝向与录制那次一致
     * (kart_odom 的 yaw 零点由上电 IMU 姿态定,重启后世界系会变)。 */
    play_origin_yaw = kart_record_get_origin_yaw();
    /* 位置闭环方案(OLMode=1)还要平移基准:把当前 kart_odom 位置投影回录制坐标系,
     * 才能与 wp[].x/y 同系比横向偏差。同理必须取【录制起点】而非倒车起点。
     * 方案 0 用不到这两个值,设了也无害(它不读 x/y),故不加分支。 */
    play_origin_x = kart_record_get_origin_x();
    play_origin_y = kart_record_get_origin_y();

    playback_openloop_mode = 1;
    playback_result = KART_PLAYBACK_RESULT_NONE;
    playback_running = 1;

    /* 只开内环(锁打角),不开航向外环。初始目标转角=末点录制打角。 */
    kart_steer_use_back_gains();
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(1);
    {
        const int16 *kart_steer = kart_record_get_steer();
        kart_steer_set_target_delta((float)kart_steer[n - 1]);
    }

    kart_control_set_enable(1);
    kart_control_set_target(kart_params_get(PARAM_S3_OL_SPD));
    return 1;
}

/* 把当前 kart_odom 世界位置投影到录制坐标系(与 wp[].x/y 同系)。
 * 与 kart_record_poll / 正向 kart_playback_poll 用的是同一套变换,三处必须一致:
 *   right  = ( cos, sin)   forward = (-sin, cos)
 * 单独抽出来是因为倒车位置闭环要用,而它不在正向 poll 的执行路径上。 */
static Point_2D kart_playback_project_to_record(const kart_odom_snapshot_t *kart_odom)
{
    Point_2D p;
    float rad = play_origin_yaw * 0.01745329252f;
    float s   = sinf(rad);
    float c   = cosf(rad);
    float dx  = kart_odom->x - play_origin_x;
    float dy  = kart_odom->y - play_origin_y;

    p.x =  c * dx + s * dy;
    p.y = -s * dx + c * dy;
    return p;
}

/* 倒车最近点【反向】搜索:倒车时进度从 n-1 走向 0,故窗口开在游标之前。
 * 与正向 find_nearest 的区别只有方向 —— 单调递减、窗口 [k-WIN, k]。
 * 单调性是防脱线跳点的关键:路径自交(迷宫来回穿)时,全局最近点可能落在
 * 路径另一段上,跟着跑就会抄近道或原地打转。 */
static uint16 kart_playback_ol_find_nearest(Point_2D cur, const kart_waypoint_t *wp)
{
    uint16 i, best = playback_ol_index;
    uint16 lo = (playback_ol_index > PLAYBACK_OL_NEAR_WIN)
              ? (uint16)(playback_ol_index - PLAYBACK_OL_NEAR_WIN) : 0u;
    float best_d, d;
    Point_2D p;

    p.x = wp[playback_ol_index].x;
    p.y = wp[playback_ol_index].y;
    best_d = get_distance(cur, p);

    for(i = lo; i < playback_ol_index; i++)
    {
        p.x = wp[i].x;
        p.y = wp[i].y;
        d = get_distance(cur, p);
        if(d < best_d) { best_d = d; best = i; }
    }
    return best;
}

/* 带符号横向偏差(米):当前点到"路径在 k 处的切线"的垂距。
 * 切线取 wp[k] → wp[k-1] 方向(倒车前进方向),向量叉积定符号:
 *   e = (t × r).z / |t|,  t=切向, r=从 wp[k] 指向当前点
 * 符号约定跟着录制坐标系(x 右 / y 前),正负的物理朝向不必推准 ——
 * 实车若发现越纠越歪,菜单把 Ke 取负即可,一次实验就能定。
 * 取切线而不是取两点连线中垂距:切线对采样疏密不敏感(录制阈值 5cm/2°,
 * 直道点稀弯道点密),不会在稀疏段算出虚高的偏差。 */
static float kart_playback_ol_cross_track(Point_2D cur, const kart_waypoint_t *wp, uint16 k)
{
    float tx, ty, len;

    /* k 已保证 >0(调用前判过见底),用 wp[k-1] 作倒车前进方向的下一点。 */
    tx = wp[k - 1].x - wp[k].x;
    ty = wp[k - 1].y - wp[k].y;
    len = sqrtf(tx * tx + ty * ty);

    /* 相邻两点几乎重合(静止段重复采样):切向无意义,判 0 偏差不纠,
     * 交给下一拍换到有效点。给 1mm 门限而不是判 0,防除零。 */
    if(len < 0.001f) return 0.0f;

    return (tx * (cur.y - wp[k].y) - ty * (cur.x - wp[k].x)) / len;
}

/* 位置闭环倒车一拍(菜单 S3 OLMode=1)。与方案 0 的差别只有两处:
 * 索引改最近点搜索、打角多一个横向 P 项。速度/软限幅/停机走同一套。
 * 推导与调参顺序见 kart_playback.h 的方案 1 注释块。 */
static void kart_playback_poll_closedloop(void)
{
    const int16 *kart_steer = kart_record_get_steer();
    const kart_waypoint_t *wp = kart_record_get_waypoints();
    kart_odom_snapshot_t kart_odom;
    Point_2D cur, start;
    uint16 k;
    const float *kart_dist = kart_record_get_dist();  /* 各点到起点的路径里程(m) */
    float delta, head_err, e_lat, corr_h, corr_e, sign, kh, ke, lead;

    kart_odom_get_snapshot(&kart_odom);
    cur = kart_playback_project_to_record(&kart_odom);

    /* 索引:最近点反向搜索,不依赖 dist_sum(打滑不再污染索引)。 */
    k = kart_playback_ol_find_nearest(cur, wp);
    playback_ol_index = k;

    /* 完成判据:进度见底,或已回到录制起点附近。
     * 前者是主判据(路径走完了),后者兜住"起点附近点密、索引降不到 0"的情况。 */
    start.x = wp[0].x;
    start.y = wp[0].y;
    /* 【2026-08-22 新增第三条:提前量 S3 Lead】上面两条都挂在 cur 上,而 cur 是 odom
     * 的 x/y 积分转到录制坐标系来的 —— 跟随 65m + 倒车 75m 之后累积误差是米级。
     * 实车现象:车已经压在发车线上了,程序还以为差 1.5m,于是继续往后倒,过线约
     * 1.5m 才刹(刹车本身不滑,实车确认过,纯粹是喊停喊晚了)。
     * 第三条改用录制里程表 dist[k]:该表只由 dist_sum 积分而来(kart_record.c:
     * 128/183),不含航向,实测 1% 内;且它沿路径度量 —— 蛇行不会像编码器总里程
     * 那样把它撑大,车横向偏多少也不影响它。于是"还剩 lead 米就喊停"等于把停车
     * 点整体往前挪 lead 米,正好抵掉那段滞后。挪多少现场量出来填多少。
     * 三条是 OR,谁先满足都算完成;本条写在最前只是因为它才是主判据。
     * 总里程不足 lead + MIN_MARGIN 时本条不启用,防短路径一发车就判完成。 */
    lead = kart_params_get(PARAM_S3_LEAD);
    if((playback_ol_total > lead + PLAYBACK_OL_LEAD_MIN_MARGIN
        && kart_dist[k] <= lead)
       || k == 0 || get_distance(cur, start) < PLAYBACK_OL_FIN_DIST)
    {
        kart_playback_complete();
        return;
    }

    /* ---- 前馈:该点录制时的真实物理打角。已验证有效,保留为基准 ---- */
    delta = (float)kart_steer[k];

    sign = PLAYBACK_OL_HEAD_SIGN;
    ke   = kart_params_get(PARAM_S3_OL_KE);
    /* Kh 不再单独调,由 Ke 反解(推导与系数来源见 kart_playback.h 的 PLAYBACK_OL_KH_FROM_KE)。
     * 用 |Ke|:Ke 的符号只决定横向纠偏方向,航向增益永远是正的。
     * Ke==0 → 退回读菜单 S3 OL Kh,与老固件逐位一致;菜单里那一行此时仍然是亮的。 */
    if(ke > PLAYBACK_OL_KE_EPS || ke < -PLAYBACK_OL_KE_EPS)
    {
        kh = PLAYBACK_OL_KH_FROM_KE * sqrtf((ke < 0.0f) ? -ke : ke);
    }
    else
    {
        kh = kart_params_get(PARAM_S3_OL_KH);
    }

    /* ---- 反馈项①:航向 P(与方案 0 同构,只是 Kp 改从菜单取)---- */
    {
        float ref_yaw = kart_playback_wrap180(play_origin_yaw + wp[k].yaw);
        head_err = kart_playback_wrap180(ref_yaw - kart_imu_get_yaw());

        if(head_err > PLAYBACK_OL_HEAD_DB || head_err < -PLAYBACK_OL_HEAD_DB)
        {
            corr_h = sign * kh * head_err;
            if(corr_h >  PLAYBACK_OL_CORR_MAX) corr_h =  PLAYBACK_OL_CORR_MAX;
            if(corr_h < -PLAYBACK_OL_CORR_MAX) corr_h = -PLAYBACK_OL_CORR_MAX;
        }
        else corr_h = 0.0f;

        playback_target_yaw = ref_yaw;
        kart_steer_set_target_yaw(ref_yaw);
    }

    /* ---- 反馈项②:横向位置 P(本方案的新增项)---- */
    e_lat = kart_playback_ol_cross_track(cur, wp, k);
    if(e_lat > PLAYBACK_OL_ELAT_DB || e_lat < -PLAYBACK_OL_ELAT_DB)
    {
        corr_e = sign * ke * e_lat;
        if(corr_e >  PLAYBACK_OL_ELAT_MAX) corr_e =  PLAYBACK_OL_ELAT_MAX;
        if(corr_e < -PLAYBACK_OL_ELAT_MAX) corr_e = -PLAYBACK_OL_ELAT_MAX;
    }
    else corr_e = 0.0f;

    /* 两项各自限幅后再合并:防"一项占满预算把另一项挤掉"。
     * 合并后仍过软限幅 —— 那是机械限位的最后一道,任何情况下不能越。 */
    delta += corr_h + corr_e;
    if(delta > KART_STEER_DELTA_LIMIT_L) delta = KART_STEER_DELTA_LIMIT_L;
    if(delta < KART_STEER_DELTA_LIMIT_R) delta = KART_STEER_DELTA_LIMIT_R;
    kart_steer_set_target_delta(delta);

    kart_control_set_target(kart_params_get(PARAM_S3_OL_SPD));

    /* 诊断:CH20=航向误差(度) CH21=横向偏差(m) CH22=索引 k CH23=最终打角。
     * 判读:e_lat 应被压向 0;若持续单向增大就是 Ke 符号反了,菜单取负。 */
    playback_cur_x = head_err;
    playback_cur_y = e_lat;
    playback_aim_x = (float)k;
    playback_aim_y = delta;
}

/* 倒车方案 0(默认,已实车验证):里程查表索引 + 航向 P 纠偏。
 * 一拍:倒退里程 d → 原路弧长 s=total-d → 查 dist_buf 找该弧长对应点 k →
 * 喂 steer_buf[k] 给转角内环 + 固定负速。剩余里程 < FINISH 或查到起点则停车。
 * 【是"位置开环"不是"全开环"】航向有闭环(下方 HEAD_EN 段),横向位置没有:
 * 车整体平移半米、航向依然正确时,本函数看到的误差是 0,不会去纠 —— 这正是实测
 * 终点横向偏 0.5~1m 的成因。好处是没有位置反馈就没有阿克曼倒车正反馈发散,
 * 误差只线性累积、有界,所以它能稳定完赛,作为出厂默认保留。
 * deadman 已在 kart_playback_poll() 入口拦截,此处不重复判。 */
static void kart_playback_poll_openloop(void)
{
    const int16 *kart_steer = kart_record_get_steer();
    const float *dist  = kart_record_get_dist();
    float d, s, delta;
    uint16 k;
#if PLAYBACK_OL_HEAD_EN
    float ref_yaw  = 0.0f;
    float head_err = 0.0f;
    float corr     = 0.0f;
#endif

    /* 倒退里程(kart_odom.dist_sum 为标量,倒车中持续增大)。 */
    d = kart_odom_get_dist() - playback_ol_dist0;
    if(d < 0.0f) d = 0.0f;

    /* 映射回原路弧长:从终点往起点递减。 */
    s = playback_ol_total - d;

    /* 返回发车区判定:剩余弧长足够小,停车。 */
    if(s <= KART_PLAYBACK_OL_FINISH)
    {
        kart_playback_complete();
        return;
    }

    /* 从当前游标向起点方向查表:找到首个 dist[k] <= s 的点(游标单调递减)。 */
    k = playback_ol_index;
    while(k > 0 && dist[k] > s) k--;
    playback_ol_index = k;

    /* 到起点也停(游标见底,防越界与卡死)。 */
    if(k == 0)
    {
        kart_playback_complete();
        return;
    }

    /* 打角基准:该弧长处录制时的真实物理打角。 */
    delta = (float)kart_steer[k];

#if PLAYBACK_OL_HEAD_EN
    /* ===== 航向 P 纠偏 =====
     * 参考航向 = 录制起点航向 + 该点录制时的相对航向 → 转回世界系,与 IMU 同系。
     * 误差过死区后才纠:内环本身有约 63 计数死区,小误差纠了也不动,只会让打角抖。
     * SIGN 取 -1 是倒车阿克曼的负反馈方向(与方案B倒车段一致)。
     * 同系前提:origin_yaw/wp[].yaw 都来自 kart_odom.yaw = KART_ODOM_YAW_SIGN*imu_yaw,
     * 当前 SIGN=+1 故可直接与 kart_imu_get_yaw() 相减(与方案B同写法)。
     * 若哪天把 KART_ODOM_YAW_SIGN 翻成 -1,这里和方案B都要改用 kart_odom_get_yaw()。 */
    {
        const kart_waypoint_t *wp = kart_record_get_waypoints();
        ref_yaw  = kart_playback_wrap180(play_origin_yaw + wp[k].yaw);
        head_err = kart_playback_wrap180(ref_yaw - kart_imu_get_yaw());

        if(head_err > PLAYBACK_OL_HEAD_DB || head_err < -PLAYBACK_OL_HEAD_DB)
        {
            /* 增益改读菜单 S3 OL Kh(两套方案共用):它的出厂默认就是
             * PLAYBACK_OL_HEAD_KP,经 kart_params 元表灌入,故默认行为与改动前
             * 完全一致 —— 只是现在不用重编译就能在现场加减。 */
            corr = PLAYBACK_OL_HEAD_SIGN * kart_params_get(PARAM_S3_OL_KH) * head_err;
            if(corr >  PLAYBACK_OL_CORR_MAX) corr =  PLAYBACK_OL_CORR_MAX;
            if(corr < -PLAYBACK_OL_CORR_MAX) corr = -PLAYBACK_OL_CORR_MAX;
            delta += corr;
        }
        else
        {
            corr = 0.0f;
        }

        /* 参考航向进 CH13,与 CH9(实测 yaw)直接对比。外环没开,这个字段只是显示用。 */
        playback_target_yaw = ref_yaw;
        kart_steer_set_target_yaw(ref_yaw);
    }
#endif

    /* 软限幅防顶机械硬限位,再喂转角内环。 */
    if(delta > KART_STEER_DELTA_LIMIT_L) delta = KART_STEER_DELTA_LIMIT_L;
    if(delta < KART_STEER_DELTA_LIMIT_R) delta = KART_STEER_DELTA_LIMIT_R;
    kart_steer_set_target_delta(delta);

    /* 固定负速倒车(菜单 S3 OLSpd 可调,不改这里的宏)。 */
    kart_control_set_target(kart_params_get(PARAM_S3_OL_SPD));

    /* 诊断快照:开环无投影坐标,借 cur/aim 四通道。
     * 关纠偏:CH20=倒退里程 d、CH21=剩余弧长 s、CH22=索引 k、CH23=最终打角;
     * 开纠偏:CH20 改记航向误差、CH21 改记纠偏量(里程 d 可从 CH19 减起点得到),
     *         这样一屏就能看清"误差有没有被压住"和"纠偏有没有顶到钳位"。 */
#if PLAYBACK_OL_HEAD_EN
    playback_cur_x = head_err;
    playback_cur_y = corr;
#else
    playback_cur_x = d;
    playback_cur_y = s;
#endif
    playback_aim_x = (float)k;
    playback_aim_y = delta;
}
