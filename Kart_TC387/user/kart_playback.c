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
static uint16 playback_index = 0;
static float  playback_target_yaw = 0.0f;

/* 方案B:正向复现里当前分段模式(带死区滞回)。0=前进段(Pure Pursuit),
 * 1=倒车段(开环回放录制打角)。死区内维持上一拍值,防边界抖动翻转使能位。 */
static uint8  playback_seg_reverse = 0;

/* ===== 科目四开环反向复现状态(不走 Pure Pursuit/航向外环)===== */
static uint8  playback_openloop_mode = 0;   /* =1 时 poll 走开环倒车分支 */
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

static float kart_playback_wrap180(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle <= -180.0f) angle += 360.0f;
    return angle;
}

static void kart_playback_poll_openloop(void);      /* 科目四开环倒车一拍(下方实现) */

/* 该点录制速度(脉冲/5ms):左右取平均。只用于判前进/倒车段,不决定快慢。 */
static float kart_playback_rec_v(const kart_waypoint_t *wp, uint16 i)
{
    return 0.5f * (wp[i].v_left + wp[i].v_right);
}

/* 第 i 段弦长(m),i..i+1。不用 kart_record 的 dist_buf:那是 RAM only,
 * 从 Flash 载入路径时是上一次录制的残留,只有 x/y 是可信的。 */
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
    float scale = kart_params_get(KART_PARAM_PB_SCALE);
    float vmax  = kart_params_get(KART_PARAM_PB_VMAX)  * KART_PLAYBACK_V_TO_MS;   /* m/s */
    float vmin  = kart_params_get(KART_PARAM_PB_VMIN)  * KART_PLAYBACK_V_TO_MS;   /* m/s */
    float alat  = kart_params_get(KART_PARAM_PB_ALAT);                            /* m/s² */
    uint16 i;

    playback_prof_n = 0;
    if(n < 2 || n > KART_RECORD_MAX_WAYPOINTS) return;
    /* 总开关(菜单 PB Prof):关 → 不生成,poll 退回取录制速度的旧行为。 */
    if(kart_params_get(KART_PARAM_PB_PROF) < 0.5f) return;
    if(alat < 0.1f) alat = 0.1f;

    /* ---- ① 曲率限速 ---- */
    for(i = 0; i < n; i++)
    {
        float rec = kart_playback_rec_v(wp, i);
        uint16 lo, hi, k;
        float arc = 0.0f, dyaw, kappa, v;

        if(rec < -KART_PLAYBACK_REV_SPEED_EPS)
        {
            /* 倒车段:提速只会放大开环打角回放的里程漂移,原速照抄。 */
            playback_prof[i] = rec * KART_PLAYBACK_V_TO_MS;
            continue;
        }

        lo = (i > KART_PLAYBACK_KAPPA_WIN) ? (uint16)(i - KART_PLAYBACK_KAPPA_WIN) : 0u;
        hi = (uint16)(i + KART_PLAYBACK_KAPPA_WIN);
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

    /* ---- ③ 反向传播(提前刹车)---- */
    for(i = n - 1; i > 0; i--)
    {
        float vn = playback_prof[i];
        float vc = playback_prof[i - 1];
        float ds, lim;

        /* 换向点两侧不互传:倒车段的负速与前进段的正速不在同一条刹车链上。 */
        if(vn < 0.0f || vc < 0.0f) continue;

        ds  = kart_playback_seg_len(wp, (uint16)(i - 1));
        lim = sqrtf(vn * vn + 2.0f * KART_PLAYBACK_ABRAKE * ds);
        if(vc > lim) playback_prof[i - 1] = lim;
    }

    /* ---- 换算回速度环量纲 ---- */
    for(i = 0; i < n; i++) playback_prof[i] *= KART_PLAYBACK_MS_TO_V;

    playback_prof_n = n;
}

void kart_playback_init(void)
{
    playback_running = 0;
    playback_openloop_mode = 0;
    playback_index = 0;
    playback_target_yaw = 0.0f;
    playback_prof_n = 0;
}

uint8 kart_playback_start(void)
{
    kart_odom_snapshot_t odom;
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

    /* 起点位姿一次性取整帧,x/y/yaw 必须同帧,否则复现坐标系原点就是歪的。 */
    kart_odom_get_snapshot(&odom);
    play_origin_x = odom.x;
    play_origin_y = odom.y;
    play_origin_yaw = odom.yaw;

    /* 起点数值退化修复:发车时 cur≈(0,0) 恰是录制第0点,find_nearest 会把
     * index 钉死在 0,速度取起点静止段≈0 → 永远卡原点起不了步。故跳过起点
     * 静止段,从第3点起步给非零初速;不足4点则退回0(短路径由完成判定收尾)。 */
    playback_index = (n > 3) ? 3 : 0;
    playback_running = 1;

    /* 外环目标初始化为当前航向,消除发车瞬间用残留 target_yaw 乱打方向。 */
    playback_target_yaw = kart_imu_get_yaw();
    kart_steer_set_target_yaw(playback_target_yaw);

    kart_steer_set_head_enable(1);
    kart_control_set_enable(1);
    return 1;
}

void kart_playback_stop(void)
{
    playback_running = 0;
    playback_openloop_mode = 0;
    /* 剖面判废:下次 start 会重算(可能改了 Vmax/Alat),不让旧剖面漏用。 */
    playback_prof_n = 0;
    kart_control_set_enable(0);
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);     /* 开环倒车只开内环,停机一并关掉 */
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
    float ld = KART_PLAYBACK_LD_BASE + kart_params_get(KART_PARAM_PB_LDGAIN) * av;
    if(ld < KART_PLAYBACK_LD_MIN) ld = KART_PLAYBACK_LD_MIN;
    if(ld > KART_PLAYBACK_LD_MAX) ld = KART_PLAYBACK_LD_MAX;
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
    kart_odom_snapshot_t odom;

    if(!playback_running) return;

    /* 遥控器是科目一deadman：失联或低挡立即停止复现。 */
    if(!kart_remote_is_online() || kart_remote_get_sw3() == KART_REMOTE_SW3_L)
    {
        kart_playback_stop();
        return;
    }

    /* ===== 科目四开环反向复现分支 =====
     * 不做定位/最近点/坐标投影,只用累计里程把倒车走过的距离 d 映射回原路弧长
     * s=total-d,查录制打角数组 steer[k] 直接喂转角内环,速度给固定负速。
     * 无位置反馈闭环 → 无阿克曼倒车正反馈发散,只有里程漂移(线性,非指数)。 */
    if(playback_openloop_mode)
    {
        kart_playback_poll_openloop();
        return;
    }

    n = kart_record_get_count();
    wp = kart_record_get_waypoints();

    /* 一致快照:x/y 一次性取整帧,避免被 5ms 中断插到半路取到撕裂位置。 */
    kart_odom_get_snapshot(&odom);
    dx = odom.x - play_origin_x;
    dy = odom.y - play_origin_y;

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

    /* 完成判定按进度点:进度已到末点,或离末点足够近 → 停。
     * 不再只看到终点距离,脱线绕圈时进度推到末点也能正常结束。 */
    {
        Point_2D last;
        last.x = wp[n - 1].x;
        last.y = wp[n - 1].y;
        if(nearest >= n - 1 || get_distance(cur, last) < KART_PLAYBACK_FINISH_DIST)
        {
            kart_playback_stop();
            return;
        }
    }

    /* 速度取进度点(最近点),而非前视点:速度跟当前所在弧段,弯道不提前拉满。
     * 剖面已生成 → 用剖面速度(几何算出的,与录制快慢无关);
     * 未生成(点数异常)→ 退回旧行为取录制速度,保证任何情况下都能跑。 */
    rec_v = 0.5f * (wp[nearest].v_left + wp[nearest].v_right);
    target_v = (playback_prof_n == n) ? playback_prof[nearest] : rec_v;
    if(target_v > KART_PLAYBACK_SPEED_MAX) target_v = KART_PLAYBACK_SPEED_MAX;
    if(target_v < -KART_PLAYBACK_SPEED_MAX) target_v = -KART_PLAYBACK_SPEED_MAX;

    /* ===== 方案B:按进度点【录制】速度符号分段选转向来源(带死区滞回)=====
     * 录制含倒车段时,Pure Pursuit 航向环对车尾正反馈会打圈。故:
     *   前进段(v> +eps):开航向外环,走 Pure Pursuit;
     *   倒车段(v< -eps):关航向外环,只开转角内环,开环回放录制打角;
     *   死区内(|v|<=eps):维持上一拍模式,防边界反复翻转使能位。
     * 判据必须用 rec_v(录制速度)而非 target_v:剖面里前进段最后几点会被
     * 反向传播压到接近 0(终点收油),若拿它判段会误判成"死区/倒车"翻掉航向环。
     * steer_buf 与 record_buf 同索引,倒车段可直接取录制打角开环回放。 */
    if(rec_v >  KART_PLAYBACK_REV_SPEED_EPS)      playback_seg_reverse = 0;
    else if(rec_v < -KART_PLAYBACK_REV_SPEED_EPS) playback_seg_reverse = 1;

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
        const int16 *steer = kart_record_get_steer();
        float delta = (float)steer[nearest];
        float ref_yaw = kart_playback_wrap180(play_origin_yaw + wp[nearest].yaw);
        float head_err = kart_playback_wrap180(ref_yaw - kart_imu_get_yaw());
        float corr = KART_PLAYBACK_REV_HEAD_SIGN * KART_PLAYBACK_REV_HEAD_KP * head_err;

        if(corr >  KART_PLAYBACK_REV_CORR_MAX) corr =  KART_PLAYBACK_REV_CORR_MAX;
        if(corr < -KART_PLAYBACK_REV_CORR_MAX) corr = -KART_PLAYBACK_REV_CORR_MAX;
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

/* =========================== 科目四开环反向复现 =========================== */
/* 启动开环倒车:车头不掉转,直接挂倒挡按里程回放录制打角。
 * 前置:车已停在原路终点(停车区),odom 仍在录制世界系(未 reset)。
 * 基准:total=录制全程里程,dist0=当前 odom 累计里程(倒车起点)。
 * 倒车中 odom.dist_sum 是标量(只增不减),故倒退里程 d=odom-dist0 单调增,
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
     * 注意本函数原来不设 play_origin_yaw(开环不需要位姿),但航向 P 纠偏要把
     * wp[k].yaw(相对录制起点的增量)转回世界系比 IMU,故这里必须补上。
     * 取 kart_record_get_origin_yaw() 而不是当前 odom.yaw:后者是"倒车起点"的
     * 航向,不是"录制起点"的航向,两者差多少就等于整段参考航向偏多少。
     * 前提:本次倒车紧接同一次录制(科目四流程就是如此)。若路径是从 Flash
     * 载入的,steer/dist 数组和 origin_yaw 都是上一次录制的残留,本来就不能用。 */
    play_origin_yaw = kart_record_get_origin_yaw();

    playback_openloop_mode = 1;
    playback_running = 1;

    /* 只开内环(锁打角),不开航向外环。初始目标转角=末点录制打角。 */
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(1);
    {
        const int16 *steer = kart_record_get_steer();
        kart_steer_set_target_delta((float)steer[n - 1]);
    }

    kart_control_set_enable(1);
    kart_control_set_target(kart_params_get(KART_PARAM_S4_OL_SPD));
    return 1;
}

/* 开环倒车一拍:倒退里程 d → 原路弧长 s=total-d → 查 dist_buf 找该弧长对应点 k →
 * 喂 steer_buf[k] 给转角内环 + 固定负速。剩余里程 < FINISH 或查到起点则停车。
 * 无位置反馈闭环 → 无阿克曼倒车正反馈发散,只有里程漂移(线性,非指数)。
 * deadman 已在 kart_playback_poll() 入口拦截,此处不重复判。 */
static void kart_playback_poll_openloop(void)
{
    const int16 *steer = kart_record_get_steer();
    const float *dist  = kart_record_get_dist();
    float d, s, delta;
    uint16 k;
#if KART_PLAYBACK_OL_HEAD_EN
    float ref_yaw  = 0.0f;
    float head_err = 0.0f;
    float corr     = 0.0f;
#endif

    /* 倒退里程(odom.dist_sum 为标量,倒车中持续增大)。 */
    d = kart_odom_get_dist() - playback_ol_dist0;
    if(d < 0.0f) d = 0.0f;

    /* 映射回原路弧长:从终点往起点递减。 */
    s = playback_ol_total - d;

    /* 返回发车区判定:剩余弧长足够小,停车。 */
    if(s <= KART_PLAYBACK_OL_FINISH)
    {
        kart_playback_stop();
        return;
    }

    /* 从当前游标向起点方向查表:找到首个 dist[k] <= s 的点(游标单调递减)。 */
    k = playback_ol_index;
    while(k > 0 && dist[k] > s) k--;
    playback_ol_index = k;

    /* 到起点也停(游标见底,防越界与卡死)。 */
    if(k == 0)
    {
        kart_playback_stop();
        return;
    }

    /* 打角基准:该弧长处录制时的真实物理打角。 */
    delta = (float)steer[k];

#if KART_PLAYBACK_OL_HEAD_EN
    /* ===== 航向 P 纠偏 =====
     * 参考航向 = 录制起点航向 + 该点录制时的相对航向 → 转回世界系,与 IMU 同系。
     * 误差过死区后才纠:内环本身有约 63 计数死区,小误差纠了也不动,只会让打角抖。
     * SIGN 取 -1 是倒车阿克曼的负反馈方向(与方案B倒车段一致)。
     * 同系前提:origin_yaw/wp[].yaw 都来自 odom.yaw = KART_ODOM_YAW_SIGN*imu_yaw,
     * 当前 SIGN=+1 故可直接与 kart_imu_get_yaw() 相减(与方案B同写法)。
     * 若哪天把 KART_ODOM_YAW_SIGN 翻成 -1,这里和方案B都要改用 kart_odom_get_yaw()。 */
    {
        const kart_waypoint_t *wp = kart_record_get_waypoints();
        ref_yaw  = kart_playback_wrap180(play_origin_yaw + wp[k].yaw);
        head_err = kart_playback_wrap180(ref_yaw - kart_imu_get_yaw());

        if(head_err > KART_PLAYBACK_OL_HEAD_DB || head_err < -KART_PLAYBACK_OL_HEAD_DB)
        {
            corr = KART_PLAYBACK_OL_HEAD_SIGN * KART_PLAYBACK_OL_HEAD_KP * head_err;
            if(corr >  KART_PLAYBACK_OL_CORR_MAX) corr =  KART_PLAYBACK_OL_CORR_MAX;
            if(corr < -KART_PLAYBACK_OL_CORR_MAX) corr = -KART_PLAYBACK_OL_CORR_MAX;
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

    /* 固定负速倒车(菜单 S4 OLSpd 可调,不改这里的宏)。 */
    kart_control_set_target(kart_params_get(KART_PARAM_S4_OL_SPD));

    /* 诊断快照:开环无投影坐标,借 cur/aim 四通道。
     * 关纠偏:CH20=倒退里程 d、CH21=剩余弧长 s、CH22=索引 k、CH23=最终打角;
     * 开纠偏:CH20 改记航向误差、CH21 改记纠偏量(里程 d 可从 CH19 减起点得到),
     *         这样一屏就能看清"误差有没有被压住"和"纠偏有没有顶到钳位"。 */
#if KART_PLAYBACK_OL_HEAD_EN
    playback_cur_x = head_err;
    playback_cur_y = corr;
#else
    playback_cur_x = d;
    playback_cur_y = s;
#endif
    playback_aim_x = (float)k;
    playback_aim_y = delta;
}
