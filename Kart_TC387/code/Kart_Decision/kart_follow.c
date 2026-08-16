/*********************************************************************************************************************
 * 文件名称  kart_follow
 * 功能说明  跟随控制律实现。设计取舍与参数依据见 kart_follow.h，此处只写实现。
 ********************************************************************************************************************/
#include "kart_follow.h"
#include "kart_vtrack.h"
#include "kart_params.h"  /* 现在从 Tracker 取输入，不再直接用 Detector */
#include <math.h>

static kart_follow_out_t kart_follow_out;

/* 上一拍下发的速度(m/s)。斜坡升降速和丢失减速都要从当前速度出发，不能跳变。 */
static float kart_follow_last_v;

/* 帧沿标志。kart_follow 每 10ms 被调一次而视觉只有约 2.8Hz,不区分"新帧"和
 * "同一帧被重复读"就会让按帧设计的判据在单帧内数满(见 .h NEAR_STOP_FRAMES)。 */
static uint8 follow_new_frame;

/* 近距安全联锁状态。带回差的二值锁存：
 *   scale_r 连续 NEAR_STOP_TICKS 拍 >= NEAR_STOP_R  → 置 1（停）
 *   scale_r          <= NEAR_RESUME_R               → 清 0（走）
 * 用锁存而不是每拍重判，是为了让"停"这个决定不被单帧噪声撤销。 */
#if (FOLLOW_NEAR_STOP_ENABLE)
static uint8  kart_follow_near_latch;
static uint16 kart_follow_near_cnt;
#endif

#if (FOLLOW_LON_MODE == FOLLOW_LON_SCALE)
/* SCALE 方案的巡航意图速度（积分器）。
 *
 * 【为什么不直接拿 kart_follow_last_v 来加减】
 *   last_v 是"最终下发值"，里面已经掺进了大方位角减速和近距联锁。如果三档
 *   增减从 last_v 出发，就等于把这两个瞬时抑制项灌进积分器：拐一个大弯把
 *   last_v 压到 0.35，弯出来之后积分器就只能从 0.35 开始按 0.3/拍 往回爬，
 *   而尺度这时候可能早就回到 NORMAL 档了 —— 车会一直慢下去爬不回来。
 *   所以积分器必须独立，抑制项只作用在输出级。 */
static float follow_scale_v;
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 速度斜坡。朝 target 走一步，步长 FOLLOW_V_RAMP，不过冲。
 * 为什么不直接赋值：速度阶跃会让后轮瞬间打滑、车头一甩，而车头一甩方位角
 * 观测就跟着抖 —— 视觉是唯一的横向反馈源，不能让执行机构去污染它。
 *-----------------------------------------------------------------------------------------------------------------*/
static float kart_follow_v_ramp_to(float cur, float target)
{
    if(cur < target)
    {
        cur += FOLLOW_V_RAMP;
        if(cur > target)
        {
            cur = target;
        }
    }
    else if(cur > target)
    {
        cur -= FOLLOW_V_RAMP;
        if(cur < target)
        {
            cur = target;
        }
    }

    return cur;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 【2026-08-10 已删除：方位角 3 帧中值滤波已移入 kart_vtrack】
 *   原先在此处对 Detector 的原始 bearing 做 3 帧中值，现在 kart_vtrack 内部已做，
 *   这里直接用 kart_vtrack->bearing_rad（已滤波）。median3() 和 bearing_hist[] 不再需要。
 *-----------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------
 * 软限位钳制。与项目其它下发路径同口径：先加中位偏置，再钳软限位。
 * 左正右负，DELTA_LIMIT_L 为正、DELTA_LIMIT_R 为负。
 *-----------------------------------------------------------------------------------------------------------------*/
static float kart_follow_clamp_delta(float delta, uint8 *saturated)
{
    float out = delta + KART_STEER_CENTER_OFS;

    *saturated = 0;

    if(out > (float)KART_STEER_DELTA_LIMIT_L)
    {
        out = (float)KART_STEER_DELTA_LIMIT_L;
        *saturated = 1;
    }
    else if(out < (float)KART_STEER_DELTA_LIMIT_R)
    {
        out = (float)KART_STEER_DELTA_LIMIT_R;
        *saturated = 1;
    }

    return out;
}

void kart_follow_reset(void)
{
    kart_follow_out.state         = KART_FOLLOW_IDLE;
    kart_follow_out.target_v_ms   = 0.0f;
    kart_follow_out.target_v_pulse= 0.0f;
    kart_follow_out.target_delta  = KART_STEER_CENTER_OFS;
    kart_follow_out.scale_level   = 0;
    kart_follow_out.scale_r       = 1.0f;
    kart_follow_out.near_stop     = 0;
    kart_follow_out.bearing_deg   = 0.0f;
    kart_follow_out.bearing_filt  = 0.0f;
    kart_follow_out.kappa         = 0.0f;
    kart_follow_out.lost_ticks    = 0;
    kart_follow_out.saturated     = 0;
    kart_follow_out.confidence    = 0;

    kart_follow_last_v = 0.0f;
    follow_new_frame = 0;

#if (FOLLOW_NEAR_STOP_ENABLE)
    kart_follow_near_latch = 0;
    kart_follow_near_cnt   = 0;
#endif

#if (FOLLOW_LON_MODE == FOLLOW_LON_SCALE)
    /* 从 V_MIN 起步而不是从 0：0 起步的话第一拍尺度必然是 NORMAL 档
     * （还没动，尺度不变），积分器加不上去，车会在原地卡一会儿才动。 */
    follow_scale_v = FOLLOW_V_MIN_MS;
#endif
}

void follow_note_new_frame(void)
{
    follow_new_frame = 1;
}

const kart_follow_out_t *kart_follow_update(const kart_vtrack_result_t *kart_vtrack)
{
    kart_follow_out_t *o = &kart_follow_out;

    /* 入口一次性取走并清零:update 有多条 return 路径,在这里消费掉才不会
     * 因为某条早退路径漏清而让同一帧被当成两次新帧。 */
    uint8 is_new = follow_new_frame;
    follow_new_frame = 0;

    float beta;              /* 方位角(弧度) */
    float v;                 /* 目标速度(m/s) */
    float kappa;             /* 纯跟踪曲率(1/m) */
    float delta;             /* 目标转角(计数) */
    float abs_deg;
    float ratio;

    /*--------------------------------------------------------------
     * 目标无效：累计丢失帧数。未到阈值时保持上一拍指令（人摆臂遮挡
     * 板子只有 0.1-0.2s，一遮就松油门会顿）；到阈值后缓速停车。
     *------------------------------------------------------------*/
    if((kart_vtrack == NULL) || (!kart_vtrack->valid))
    {
        if(o->lost_ticks < 0xFFFFU)
        {
            o->lost_ticks++;
        }

        /* 置信度直接取 kart_vtrack 的（kart_vtrack 无效时 confidence 已经很低或为 0）*/
        o->confidence = (kart_vtrack != NULL) ? kart_vtrack->confidence : 0;

        if(o->lost_ticks >= FOLLOW_LOST_TICKS)
        {
            o->state = KART_FOLLOW_LOST;

            /* 从当前速度往下退，不做急停 */
            v = kart_follow_last_v - FOLLOW_LOST_DECEL;
            if(v < 0.0f)
            {
                v = 0.0f;
            }
            kart_follow_last_v = v;

#if (FOLLOW_LON_MODE == FOLLOW_LON_SCALE)
            /* 积分器跟着一起退。不退的话：丢目标 2 秒滑停到 0，重捕的瞬间
             * 积分器还停在丢之前的 1.8m/s，车会立刻朝一个刚确认了不到一帧的
             * 目标全速冲过去。重捕后应该从慢速重新建立信任。 */
            if(follow_scale_v > v)
            {
                follow_scale_v = v;
            }
#endif

            o->target_v_ms    = v;
            o->target_v_pulse = v * KART_PULSE_MS_TO_V;

            /* 丢目标时方向回中：继续保持上一拍打角会画圈跑偏，
             * 而回中至少是"沿当前朝向直着滑停"，可预测。 */
            o->target_delta = kart_follow_clamp_delta(0.0f, &o->saturated);
        }
        /* 未到阈值：state / target_* 全部保持上一拍不变 */

        return o;
    }

    /*--------------------------------------------------------------
     * 目标有效
     *------------------------------------------------------------*/
    o->lost_ticks = 0;

    /* 方位角：kart_vtrack 内部已做 3 帧中值滤波，直接用 */
    beta = kart_vtrack->bearing_rad;

    o->bearing_deg  = beta * 57.29578f;       /* 给调试页看 */
    o->bearing_filt = beta * 57.29578f;       /* kart_vtrack 已滤波，这里两个值相同 */

    /* 尺度等级：来自 kart_vtrack 的 scale_level，-1/0/1 三档 */
    o->scale_level = (int8)kart_vtrack->scale_level;
    o->scale_r     = kart_vtrack->scale_r;

    /* 置信度直接取 kart_vtrack 的综合置信度（已包含存活率、FB 误差、scale 剧变、距 Detector 帧数）*/
    o->confidence = kart_vtrack->confidence;

    /* --- 转角：纯跟踪 kappa = 2*sin(beta)/L_NOMINAL ---
     * L 现在是固定标称值，不再用实测距离。代价是近处转向偏软、远处偏硬，
     * 由方位角减速兜住（BEARING_SLOW_DEG）。 */
    kappa = (2.0f * sinf(beta)) / FOLLOW_L_NOMINAL_M;
    o->kappa = kappa;

    /* R * |delta| = 1480，kappa = 1/R → delta = 1480 * kappa。
     * 符号：beta > 0 表示目标在右侧，需要右打角。项目约定左正右负，
     * 故取负号。【实车第一次跑必须确认这个符号，翻了就是一打就反向】 */
    delta = -(KART_STEER_R_TIMES_DELTA * kappa);
    o->target_delta = kart_follow_clamp_delta(delta, &o->saturated);

    /* --- 纵向：两套方案编译期二选一，见 kart_follow.h FOLLOW_LON_MODE --- */
#if (FOLLOW_LON_MODE == FOLLOW_LON_SCALE)
    /* 方案 SCALE：尺度比三档增减，车自己维持距离。
     * 积分器独立于 last_v（原因见文件头 follow_scale_v 的注释）。
     * NORMAL 档不动，靠 ±10% 死区把噪声挡在外面。 */
    if(KART_VTRACK_SCALE_TOO_FAR == kart_vtrack->scale_level)
    {
        follow_scale_v += FOLLOW_SCALE_V_DELTA_MS;
    }
    else if(KART_VTRACK_SCALE_TOO_NEAR == kart_vtrack->scale_level)
    {
        follow_scale_v -= FOLLOW_SCALE_V_DELTA_MS;
    }

    /* 积分器限幅。必须在这里夹，不能只夹输出：否则尺度长时间停在
     * TOO_FAR 会让积分器涨到几十 m/s，等目标终于靠近时要减速几十拍才
     * 有反应（积分饱和/windup）。 */
    if(follow_scale_v > FOLLOW_V_MAX_MS)
    {
        follow_scale_v = FOLLOW_V_MAX_MS;
    }
#if (FOLLOW_ALLOW_REVERSE)
    if(follow_scale_v < -FOLLOW_V_MIN_MS)
    {
        follow_scale_v = -FOLLOW_V_MIN_MS;
    }
#else
    /* 不允许倒退：下限夹在 0，不夹在 V_MIN。
     * 夹在 V_MIN 的话人走到车跟前停下时车还会以 0.25m/s 顶上去。 */
    if(follow_scale_v < 0.0f)
    {
        follow_scale_v = 0.0f;
    }
#endif

    v = follow_scale_v;

    /* 死区抬升：0 < v < V_MIN 的区间电机是小占空比堵转，发热不出力。
     * 要么给够要么给 0，不留中间态。 */
    if((v > 0.0f) && (v < FOLLOW_V_MIN_MS))
    {
        v = FOLLOW_V_MIN_MS;
    }
#else
    /* 方案 FIXED：固定巡航速度，【不做任何基于视觉的调速】。
     * scale_level / scale_r 已经在上面存进 o 了，但只是上报给调试页看，
     * 不进入速度计算。距离由人自己走快走慢维持。
     * 理由见 kart_follow.h 文件头【纵向：两套方案并存，编译期切，靠实车定】。
     *
     * 唯一的例外是近距安全联锁 —— 那是刹车，不是调速。 */
    /* 2026-08-16 死宏改读参数表,现场可调(菜单 Flw Cruise)。
     * 不缓存副本:每拍读一次,菜单一改立刻生效,不用等 apply 推送。 */
    v = kart_params_get(PARAM_FLW_CRUZ);
#endif

#if (FOLLOW_NEAR_STOP_ENABLE)
    /* 近距安全联锁。带回差 + 连续计数的二值锁存，只输出"走/停"。
     * 注意这里判的是 scale_r 而不是 scale_level：level 的阈值(1.10)是给
     * 轻微靠近用的，噪声能碰到；联锁要用更高的硬阈值(1.35)。 */
    if(kart_follow_near_latch)
    {
        /* 已经停了：等尺度掉回恢复阈值以下才放行 */
        if(kart_vtrack->scale_r <= FOLLOW_NEAR_RESUME_R)
        {
            kart_follow_near_latch = 0;
            kart_follow_near_cnt   = 0;
        }
    }
    else
    {
        if(kart_vtrack->scale_r >= FOLLOW_NEAR_STOP_R)
        {
            /* 只在帧沿上累加。不加这道门,同一帧被重复读约 36 次,
             * 一个像素的量化噪声就能数满 —— 见 .h NEAR_STOP_FRAMES 的推导。
             * 判阈值放在 if 外面:计数已经够了就该锁存,不必等下一个帧沿。 */
            if(is_new)
            {
                kart_follow_near_cnt++;
            }
            if(kart_follow_near_cnt >= FOLLOW_NEAR_STOP_FRAMES)
            {
                kart_follow_near_latch = 1;
            }
        }
        else
        {
            kart_follow_near_cnt = 0;       /* 没连上就重新数 */
        }
    }

    o->near_stop = kart_follow_near_latch;

    if(kart_follow_near_latch)
    {
        v = 0.0f;                            /* 人太近，停 */
    }
#else
    o->near_stop = 0;
#endif

    /* --- 大方位角减速 ---
     * 满舵半径 1.39m，近处大角度几何上转不过来，减速换转向余量。 */
    abs_deg = (o->bearing_deg < 0.0f) ? -o->bearing_deg : o->bearing_deg;
    if(abs_deg > FOLLOW_BEARING_SLOW_DEG)
    {
        if(abs_deg >= FOLLOW_BEARING_MAX_DEG)
        {
            ratio = FOLLOW_SLOW_MIN_RATIO;
        }
        else
        {
            /* 在 SLOW_DEG 到 MAX_DEG 之间线性插值到 SLOW_MIN_RATIO */
            ratio = 1.0f - ((1.0f - FOLLOW_SLOW_MIN_RATIO)
                            * (abs_deg - FOLLOW_BEARING_SLOW_DEG)
                            / (FOLLOW_BEARING_MAX_DEG - FOLLOW_BEARING_SLOW_DEG));
        }
        v *= ratio;
    }

    /* --- 斜坡到目标速度 ---
     * 两套方案在这里汇合，抑制项（近距联锁、大方位角减速）都只作用在输出级，
     * 不回写进 SCALE 的积分器。都不允许阶跃，统一走斜坡。
     *
     * FIXED 下 v 只有三种可能：V_CRUISE、0（联锁）、V_CRUISE*ratio（方位角减速）；
     * 方位角最狠只压到 V_CRUISE × SLOW_MIN_RATIO = 1.10 × 0.40 = 0.44 m/s，
     * 高于 V_MIN(0.25)，无堵转问题。【改巡航速度时必须重算这一乘】
     * FIXED 分支没有死区抬升（那段在 #if SCALE 里），乘完跌破 V_MIN 就真堵转。
     * SCALE 下 v 已在上面按 V_MAX / V_MIN 夹过，方位角减速可能把它压到 V_MIN
     * 以下 —— 这是有意的，弯道上宁可慢也不要冲出去，且是暂态不会持续。 */
    if(v < 0.0f)
    {
        v = 0.0f;      /* 恒不倒退：倒车时摄像头看不见人，属于开环 */
    }

    kart_follow_last_v = kart_follow_v_ramp_to(kart_follow_last_v, v);

    o->target_v_ms    = kart_follow_last_v;
    o->target_v_pulse = kart_follow_last_v * KART_PULSE_MS_TO_V;
    o->state          = (kart_follow_last_v > 0.0f) ? KART_FOLLOW_TRACKING : KART_FOLLOW_HOLD;

    return o;
}

const kart_follow_out_t *kart_follow_get(void)
{
    return &kart_follow_out;
}
