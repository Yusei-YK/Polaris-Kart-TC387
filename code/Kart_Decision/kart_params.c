#include "kart_params.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"
#include "kart_playback.h"
#include "kart_remote.h"
#include "kart_mission.h"
#include "kart_power.h"
#include "kart_follow.h"
#include "zf_driver_flash.h"

/* 参数表元数据。def 一律引用各模块原有的宏,保证"出厂值"与代码里写的一致,
 * 改宏后 Load Default 即生效,不会出现两处默认值打架。 */
static const kart_param_meta_t param_meta[PARAM_MAX] =
{
    /* name          min      max      step    def                              dec */
    { "Ramp Step",   0.0f,   30.0f,   0.1f,  SPEED_RAMP_STEP_DEFAULT,      1 },
    /* 2026-07-28 赛前改为出厂开(原出厂 0)。理由:剖面关着时复现是"照抄录制速度",
     * 录得慢就跑得慢,提速这条路直接堵死。开着 + Vmax 仍留 30 是"开了但钳保守":
     * 剖面被钳在 2.21 m/s,行为与原来慢速录制接近,风险可控;要快现场推 Vmax。
     * 【已知局限】剖面只算了横向加速度,没算转向速率上限(1800 计数/s = 41°/s)。
     * 绕桩段(κ≈0.55/m)剖面给 2.70 m/s,而一次打满反向 1646 计数要 0.91s、
     * 3m 半周期只有 1.11s → 实际上限约 1.7 m/s,剖面乐观约 60%。
     * 现场兜底手段是压 PB Scale,不是改 Alat(Alat 只影响弯道不影响直道)。 */
    { "PB Prof",     0.0f,    1.0f,   1.0f,  1.0f,                              0 },
    { "PB Scale",    0.10f,   6.00f,  0.05f, 1.00f,                             2 },
    /* Vmax 出厂给 30(约 2.2 m/s),不是 SPEED_MAX(60):剖面第一次开起来时
     * 直道会直接顶到 Vmax,给满量程等于一上来就全油门。要快自己往上调。
     * 上限 95:整车最高时速 25km/h = 94 脉冲/5ms,给到量程顶即可,再大是空的。
     * 【现场必读】几何最大曲率(满舵 R=1.32m → κ=0.76/m)对应 sqrt(4.0/0.76)=
     * 2.29 m/s = 31 脉冲。Vmax ≤ 31 时剖面整段被压平成常数,弯道整形完全不起作用,
     * 只剩终点锚点刹车。想要真正的过弯减速必须把 Vmax 放到 40~50。 */
    { "PB Vmax",     5.0f,  150.0f,   1.0f,  30.0f,                             0 },
    { "PB Vmin",     0.0f,   95.0f,   0.5f,  0.0f,                              1 },
    { "PB Alat",     0.5f,   40.0f,   0.2f,  PLAYBACK_ALAT_DEFAULT,        1 },
    { "PB LdGain",   0.0f,    0.50f,  0.002f,KART_PLAYBACK_LD_GAIN,             3 },
    { "PB Clamp",    5.0f,  150.0f,   1.0f,  KART_PLAYBACK_SPEED_MAX,           0 },
    { "PB ABrake",   0.2f,   20.0f,   0.1f,  PLAYBACK_ABRAKE,              1 },
    { "PB LdMax",    0.30f,   6.00f,  0.05f, KART_PLAYBACK_LD_MAX,              2 },
    { "RC Vmax",     5.0f,  150.0f,   1.0f,  KART_REMOTE_MAX_SPEED,             0 },
    { "Head Kp",     0.0f,  400.0f,   1.0f,  KART_HEAD_KP_DEFAULT,              0 },
    /* 保留两个占位项，避免后续参数的 Flash 下标整体移动。运行时和菜单均不使用。 */
    { "Reserved 1", -95.0f,  -1.0f,   1.0f, -25.0f,                             0 },
    /* 【2026-08-22】原 "Reserved 2" 占位槽改作科目三倒车提前完成量。量程 0.05..5.00 /
     * 步长 0.05 / 小数位 2 一个字没动 —— 它本来就是米量纲,正好合用;槽位序号和
     * PARAM_MAX 也没动,所以 Flash 旧存档照样读得回来。出厂值从 1.40 改成 1.00
     * (= 实测过线 1.5m 减去可接受的 0.5m),来历见 kart_playback.h。 */
    { "S3 Lead",     0.05f,   5.00f,  0.05f,  PLAYBACK_OL_LEAD_DEFAULT,         2 },
    { "S3 OLSpd",   -95.0f,  -1.0f,   1.0f,  KART_PLAYBACK_OL_SPEED,            0 },
    /* 倒车方案:出厂 0 = 已实车验证能完赛的里程查表方案。1 = 位置闭环(待验证)。
     * step=1 → 一下按键就切换,不用连点。 */
    { "S3 OLMode",   0.0f,    1.0f,   1.0f,  0.0f,                              0 },
    /* 横向增益(计数/米):满舵约 1100 计数,给 400 意味着偏 1m 就出 36% 舵。
     * 允许负值:倒车横向反馈符号只有 ±1 两种可能,现场发现越纠越歪就取负,
     * 不必重新烧写。出厂 0 → 新方案首次打开时只有最近点索引在起作用,
     * 与老方案只差"索引怎么算"这一个变量,便于单独判断索引改动的效果。 */
    { "S3 OL Ke", -3000.0f,3000.0f,  20.0f,  0.0f,                              0 },
    /* 倒车航向增益(计数/度):原 OL_HEAD_KP 宏搬进菜单。日志已验证 20 够用
     * (误差穿零、修正量没碰 ±400 钳位),故默认保持 20 不变。 */
    { "S3 OL Kh",    0.0f,  400.0f,   2.0f,  PLAYBACK_OL_HEAD_KP,          0 },
    /* ---- 2026-07-28 第二批:限制项本身进菜单。默认值一律等于原宏,行为不变 ---- */
    /* 速度环积分限幅。出厂 3000 = 原 KART_SPEED_IMAX_DEFAULT。
     * 上限给满量程 10000:提速要吃掉那 27% 拿不到的 duty 就得能放到这么大。
     * 【现场纪律】3000 → 5000 → 7000 分档试,别一步顶满 —— 2x150W 有刷 + 6S,
     * duty 真跑到满量程时 H 桥和电机温度要盯着。 */
    { "Spd Imax",    0.0f,10000.0f, 250.0f,  KART_SPEED_IMAX_DEFAULT,           0 },
    /* 速度环 Kp。出厂 200 = 原 KART_SPEED_KP_DEFAULT。与 Imax 一起决定稳态误差:
     * 目标 60 实测 40.9,靠这两项才补得回来。 */
    { "Spd Kp",      0.0f, 1000.0f,  10.0f,  KART_SPEED_KP_DEFAULT,             0 },
    /* 转角内环输出限幅。出厂 6000 = 原 KART_STEER_OUTMAX_DEFAULT。 */
    { "Str OutMax",1000.0f,10000.0f, 250.0f, KART_STEER_OUTMAX_DEFAULT,         0 },
    /* 倒车段速度倍率。出厂 1.0 = 原样照抄录制速度,行为与改动前逐位相同。
     * 放大它治"录制→回放每代掉一档";放大过头会放大开环打角回放的里程漂移。 */
    { "PB RevScl",   0.20f,   5.00f,  0.05f, 1.00f,                             2 },
    /* 后轮 duty 每拍升幅上限。出厂 400 = 原 KART_SLEW_REAR_STEP。 */
    { "Slew Rear", 100.0f,10000.0f, 100.0f,  (float)KART_SLEW_REAR_STEP,        0 },
    /* 倒车段航向纠偏钳位(计数)。出厂 400 = 原 KART_PLAYBACK_REV_CORR_MAX。
     * 上限 1133 = 转向软限位:纠偏最多允许打到满舵,再大也被软限位截断,给了也没用。
     * 【生效范围只有一处】前进复现里夹着的倒车段(kart_playback.c:482)。
     * 科目三那段独立开环倒车【不看它】,两条路径都用死宏 OL_CORR_MAX=400
     * (kart_playback.c:690/794)—— 科四倒车拐不进去调本项没用,详见 kart_params.h。
     * 【什么时候加】科一录制里带倒车段、该段拐不到位/半径偏大 → 加;左右摆头 → 减。 */
    { "PB RevCorr",  0.0f, 1133.0f, 25.0f,  KART_PLAYBACK_REV_CORR_MAX,        0 },
    /* 跟随巡航速度(m/s)。出厂 1.10 = 原 FOLLOW_V_CRUISE_MS。
     * 下限 0.65:见 kart_params.h 的堵转区推导。上限 3.00（2026-08-17 从 1.80 提到 2.20，2026-08-23 再提到 3.00，FIXED 分支不受 V_MAX_MS 钳位，那个宏只管 SCALE 分支）。【为什么还要放】科目三倒车段现在照抄录制速度再乘 PB RevScl，录制有多快倒车就有多快，跟随段的速度上限于是同时决定了倒车段的上限。实测跟随段中位只有 1.21~1.55 而峰值已到 2.25~2.34，说明直道上确实顶到过旧上限，放开有意义；弯里由方位角减速自己压住，不靠这个上限保护。
     * 【别一次跳过 1.5】速度上去后方位角环滞后会放大,先确认不振荡。 */
    { "Flw Cruise",  0.65f,   3.00f,  0.05f, FOLLOW_V_CRUISE_MS,           2 },
};

static float param_val[PARAM_MAX];

/* 脏标记:有值被改过且还没落盘。只有它=1 时才擦写 DFlash。
 * 目的两条:①现场调完不用记得手动 Save,退出界面自动存,reset 重置 IMU 不丢值;
 *          ②DFlash 擦写次数有限(且一次擦写阻塞数 ms),没改就绝不写。 */
static uint8 param_dirty = 0;

static uint32 page_buf[EEPROM_PAGE_LENGTH];

static uint32 f2u(float f)  { flash_data_union u; u.float_type  = f; return u.uint32_type; }
static float  u2f(uint32 v) { flash_data_union u; u.uint32_type = v; return u.float_type;  }

/* ==================== 速度联动量自动解算 ====================
 * 【为什么要它】Spd Imax / Str OutMax 不是独立自由度,它们是"你设的速度"的函数。
 * 过去必须手动联调:改了速度不跟着改这两个,速度就上不去(环饱和)或走线画龙。
 * 现在只留速度当旋钮,这两个自己算,菜单里转灰。
 *
 * Imax:速度环稳态 duty = Kp*e + I,想让 e -> 0 就得让积分器供得上整份 duty。
 *   后轮实测拟合 v(m/s) = 0.00046*duty - 0.10,反解目标速度需要的 duty,留 1.3 倍余量。
 *   代入现场值(Flw 1.70 / OLSpd -33 即 2.43m/s):1.3*(2.43+0.10)/0.00046 = 7150,
 *   与本文件下方原注释里手试出来的"3000 -> 5000 -> 7000"终点吻合 —— 互为验证。
 *   【放大它为什么安全】Imax 是天花板不是油门:变大不会让车超过命令速度,只是拆掉
 *   那道拿不到 31% duty 的人为限制。电流/温度由速度目标决定,那个还在人手里。
 *   代价是瞬态积分能积更多,起步/出弯可能有点速度超调,有 Ramp Step 兜着。
 *   下限锁在出厂 3000:只许往上,绝不因为反解把车调得比现在慢。
 *
 * OutMax:转角内环输出限幅决定转向速率(出厂 6000 实测 1800 计数/s)。按本文件
 *   PB Prof 注释里的绕桩推导,6000 支撑到约 1.7m/s;速度再上去必须同比放大,
 *   否则转向跟不上就画龙。只跟【跟随速度】挂钩:倒车沿录制轨迹回溯,弯没那么急。
 *
 * 【生效范围】只在 S3 OLMode == 1 时反解。打回 0 一次性恢复全套手调行为(手调
 * Imax/OutMax + 手调 Kh + 里程查表倒车) —— 沿用既有逃生开关,不再另加开关行。
 * 【不碰存储】Spd Imax / Str OutMax 存在 DFlash 的值原样保留,只是运行时不读。 */
#define SPD_DERIVE_DUTY_PER_MS   (0.00046f)   /* v(m/s) = 本值*duty - OFFSET,后轮实测拟合 */
#define SPD_DERIVE_DUTY_OFFSET   (0.10f)
#define SPD_DERIVE_IMAX_MARGIN   (1.30f)      /* 余量:拟合偏小才有害,这就是为它留的 */
#define SPD_DERIVE_IMAX_FLOOR    (3000.0f)    /* = 出厂 Imax,反解只许往上 */
#define SPD_DERIVE_IMAX_CEIL     (10000.0f)   /* = duty 满量程 = 菜单量程上限 */
#define SPD_DERIVE_STR_BASE      (6000.0f)    /* 出厂 OutMax */
#define SPD_DERIVE_STR_BASE_V    (1.70f)      /* 该 OutMax 支撑的速度(绕桩推导) */
#define SPD_DERIVE_STR_CEIL      (10000.0f)

static uint8 param_spd_derive_on(void)
{
    return (uint8)(param_val[PARAM_S3_OL_MODE] > 0.5f);
}

static float param_derive_imax(void)
{
    float v_flw = param_val[PARAM_FLW_CRUZ];                        /* m/s */
    float v_rev = -param_val[PARAM_S3_OL_SPD] * PLAYBACK_V_TO_MS;   /* 脉冲/5ms(负) -> m/s(正) */
    float v_top = (v_flw > v_rev) ? v_flw : v_rev;
    float imax  = SPD_DERIVE_IMAX_MARGIN
                  * (v_top + SPD_DERIVE_DUTY_OFFSET) / SPD_DERIVE_DUTY_PER_MS;

    if(imax < SPD_DERIVE_IMAX_FLOOR) imax = SPD_DERIVE_IMAX_FLOOR;
    if(imax > SPD_DERIVE_IMAX_CEIL)  imax = SPD_DERIVE_IMAX_CEIL;
    return imax;
}

static float param_derive_outmax(void)
{
    float outmax = SPD_DERIVE_STR_BASE
                   * param_val[PARAM_FLW_CRUZ] / SPD_DERIVE_STR_BASE_V;

    if(outmax < SPD_DERIVE_STR_BASE) outmax = SPD_DERIVE_STR_BASE;
    if(outmax > SPD_DERIVE_STR_CEIL) outmax = SPD_DERIVE_STR_CEIL;
    return outmax;
}

/* 四个速度旋钮任意一个变了都走这里重算并下推,不依赖 param_apply 的调用顺序
 * (kart_params_init 里 param_apply_all 是按下标循环的,顺序不能当依赖)。 */
static void param_apply_derived_speed(void)
{
    if(param_spd_derive_on())
    {
        kart_control_set_speed_imax(param_derive_imax());
        kart_steer_set_angle_outmax(param_derive_outmax());
    }
    else
    {
        kart_control_set_speed_imax(param_val[PARAM_SPD_IMAX]);
        kart_steer_set_angle_outmax(param_val[PARAM_STR_OUTMAX]);
    }
}

/* 把参数推给对应模块。只有"模块内部存了副本"的参数需要 apply;
 * 其余(复现倍率/地板/Alat/倒库里程等)由使用方每次直接 kart_params_get 读,
 * 故 apply 里没有它们 —— 少一次同步就少一处不一致。 */
static void param_apply(uint8 id)
{
    switch(id)
    {
        case PARAM_RAMP:
            kart_control_set_ramp_step(param_val[id]);
            break;

        case PARAM_HEAD_KP:
            /* 只改 Kp,Ki/Kd 沿用当前值(外环 Ki/Kd 默认为 0,不放进菜单)。 */
            kart_steer_set_head_pid(param_val[id],
                                    KART_HEAD_KI_DEFAULT,
                                    KART_HEAD_KD_DEFAULT);
            break;

        /* 这五项任意一个变化都要重算 Imax/OutMax:前两个是被反解的量本身(手调模式
         * 下才直接生效),中间两个是反解的输入,最后一个是反解的总开关。 */
        case PARAM_SPD_IMAX:
        case PARAM_STR_OUTMAX:
        case PARAM_FLW_CRUZ:
        case PARAM_S3_OL_SPD:
        case PARAM_S3_OL_MODE:
            param_apply_derived_speed();
            break;

        case PARAM_SPD_KP:
            kart_control_set_speed_kp(param_val[id]);
            break;

        case PARAM_SLEW_REAR:
            power_set_slew_rear_step((int16)param_val[id]);
            break;

        default:
            break;      /* 其余参数由使用方直接读,无需推送 */
    }
}

static void param_apply_all(void)
{
    uint8 i;
    for(i = 0; i < PARAM_MAX; i++) param_apply(i);
}

void kart_params_load_default(void)
{
    uint8 i;
    for(i = 0; i < PARAM_MAX; i++) param_val[i] = param_meta[i].def;
    param_apply_all();
    param_dirty = 1;        /* 恢复默认也算改动:退出界面时落盘,否则下次上电又读回旧值 */
}

void kart_params_init(void)
{
    uint8 i;

    kart_params_load_default();
    param_dirty = 0;        /* 上电阶段不算改动:载默认/载 Flash 都不该触发回写 */

    flash_read_page(0, PARAMS_PAGE, page_buf, EEPROM_PAGE_LENGTH);

    /* count 必须与当前 PARAM_MAX 完全一致:加/删参数后旧数据整表判废,
     * 避免按错位下标读到别的参数的值(比宁可用默认更危险)。 */
    if(page_buf[0] == PARAMS_MAGIC && page_buf[1] == (uint32)PARAM_MAX)
    {
        for(i = 0; i < PARAM_MAX; i++)
        {
            float v = u2f(page_buf[2 + i]);
            /* 逐个钳位:Flash 位翻转或改过 min/max 时不让越界值进来。 */
            if(v < param_meta[i].min) v = param_meta[i].min;
            if(v > param_meta[i].max) v = param_meta[i].max;
            param_val[i] = v;
        }
        param_apply_all();
    }
}

float kart_params_get(uint8 id)
{
    if(id >= PARAM_MAX) return 0.0f;
    return param_val[id];
}

/* 见 kart_params.h。只有被反解的两项返回算出来的值,其余原样透传。 */
float kart_params_derived(uint8 id)
{
    if(id >= PARAM_MAX) return 0.0f;
    if(param_spd_derive_on())
    {
        if(id == PARAM_SPD_IMAX)   return param_derive_imax();
        if(id == PARAM_STR_OUTMAX) return param_derive_outmax();
    }
    return param_val[id];
}

void kart_params_set(uint8 id, float v)
{
    if(id >= PARAM_MAX) return;
    if(v < param_meta[id].min) v = param_meta[id].min;
    if(v > param_meta[id].max) v = param_meta[id].max;
    /* 值没变就不置脏:钳位到边界后反复按 UP 不会白擦一次 Flash。 */
    if(v != param_val[id]) param_dirty = 1;
    param_val[id] = v;
    param_apply(id);
}

void kart_params_step(uint8 id, int8 dir)
{
    kart_params_step_mul(id, dir, 1u);
}

/* 粗调:一次走 mul 个 step。菜单长按/快旋时传 10,把"0.5 调到 2.0"从点 15 下
 * 变成点 2 下。mul=0 当 1 处理,防调用点算出 0 导致按了没反应。 */
void kart_params_step_mul(uint8 id, int8 dir, uint8 mul)
{
    if(id >= PARAM_MAX) return;
    if(mul == 0u) mul = 1u;
    kart_params_set(id, param_val[id] + param_meta[id].step * (float)dir * (float)mul);
}

const kart_param_meta_t* kart_params_meta(uint8 id)
{
    if(id >= PARAM_MAX) return &param_meta[0];
    return &param_meta[id];
}

uint8 kart_params_save(void)
{
    uint16 k;
    uint8  i;

    for(k = 0; k < EEPROM_PAGE_LENGTH; k++) page_buf[k] = 0u;

    page_buf[0] = PARAMS_MAGIC;
    page_buf[1] = (uint32)PARAM_MAX;
    for(i = 0; i < PARAM_MAX; i++) page_buf[2 + i] = f2u(param_val[i]);

    /* 阻塞擦写一页(数 ms)。调用方保证车已停(菜单里只在静止界面提供该项)。 */
    flash_write_page(0, PARAMS_PAGE, page_buf, EEPROM_PAGE_LENGTH);
    param_dirty = 0;
    return 0;
}

uint8 kart_params_is_dirty(void)
{
    return param_dirty;
}

/* 有改动才落盘。返回 1=本次真写了 Flash(调用方可据此提示 "Saved"),0=无改动跳过。
 * 菜单在退出参数界面时调它,所以不存在"改完忘了存";没改则一个字节都不写。 */
uint8 kart_params_save_if_dirty(void)
{
    if(!param_dirty) return 0;
    kart_params_save();
    return 1;
}
