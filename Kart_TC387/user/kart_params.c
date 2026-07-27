#include "kart_params.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"
#include "kart_playback.h"
#include "kart_remote.h"
#include "kart_mission.h"
#include "zf_driver_flash.h"

/* 参数表元数据。def 一律引用各模块原有的宏,保证"出厂值"与代码里写的一致,
 * 改宏后 Load Default 即生效,不会出现两处默认值打架。 */
static const kart_param_meta_t param_meta[KART_PARAM_MAX] =
{
    /* name          min      max      step    def                              dec */
    { "Ramp Step",   0.0f,   10.0f,   0.1f,  KART_SPEED_RAMP_STEP_DEFAULT,      1 },
    /* 剖面出厂关:第一次烧进去时科目一/科目二复现行为与改动前逐位一致,
     * 先只验证斜坡治好了顿挫,再单独打开剖面提速。 */
    { "PB Prof",     0.0f,    1.0f,   1.0f,  0.0f,                              0 },
    { "PB Scale",    0.30f,   2.50f,  0.05f, 1.00f,                             2 },
    /* Vmax 出厂给 30(约 2.2 m/s),不是 SPEED_MAX(60):剖面第一次开起来时
     * 直道会直接顶到 Vmax,给满量程等于一上来就全油门。要快自己往上调。 */
    { "PB Vmax",     5.0f,   80.0f,   1.0f,  30.0f,                             0 },
    { "PB Vmin",     0.0f,   30.0f,   0.5f,  0.0f,                              1 },
    { "PB Alat",     1.0f,   12.0f,   0.2f,  KART_PLAYBACK_ALAT_DEFAULT,        1 },
    { "PB LdGain",   0.0f,    0.10f,  0.002f,KART_PLAYBACK_LD_GAIN,             3 },
    { "RC Vmax",     5.0f,   80.0f,   1.0f,  KART_REMOTE_MAX_SPEED,             0 },
    { "Head Kp",     1.0f,  120.0f,   1.0f,  KART_HEAD_KP_DEFAULT,              0 },
    { "S1 RevSpd",  -40.0f,  -5.0f,   1.0f,  KART_S1_REVERSE_SPEED,             0 },
    { "S1 RevStop",  0.50f,   2.50f,  0.05f, KART_S1_REVERSE_STOP_DIST,         2 },
    { "S4 OLSpd",   -40.0f,  -5.0f,   1.0f,  KART_PLAYBACK_OL_SPEED,            0 },
};

static float param_val[KART_PARAM_MAX];

/* 脏标记:有值被改过且还没落盘。只有它=1 时才擦写 DFlash。
 * 目的两条:①现场调完不用记得手动 Save,退出界面自动存,reset 重置 IMU 不丢值;
 *          ②DFlash 擦写次数有限(且一次擦写阻塞数 ms),没改就绝不写。 */
static uint8 param_dirty = 0;

static uint32 page_buf[EEPROM_PAGE_LENGTH];

static uint32 f2u(float f)  { flash_data_union u; u.float_type  = f; return u.uint32_type; }
static float  u2f(uint32 v) { flash_data_union u; u.uint32_type = v; return u.float_type;  }

/* 把参数推给对应模块。只有"模块内部存了副本"的参数需要 apply;
 * 其余(复现倍率/地板/Alat/倒库里程等)由使用方每次直接 kart_params_get 读,
 * 故 apply 里没有它们 —— 少一次同步就少一处不一致。 */
static void param_apply(uint8 id)
{
    switch(id)
    {
        case KART_PARAM_RAMP:
            kart_control_set_ramp_step(param_val[id]);
            break;

        case KART_PARAM_HEAD_KP:
            /* 只改 Kp,Ki/Kd 沿用当前值(外环 Ki/Kd 默认为 0,不放进菜单)。 */
            kart_steer_set_head_pid(param_val[id],
                                    KART_HEAD_KI_DEFAULT,
                                    KART_HEAD_KD_DEFAULT);
            break;

        default:
            break;      /* 其余参数由使用方直接读,无需推送 */
    }
}

static void param_apply_all(void)
{
    uint8 i;
    for(i = 0; i < KART_PARAM_MAX; i++) param_apply(i);
}

void kart_params_load_default(void)
{
    uint8 i;
    for(i = 0; i < KART_PARAM_MAX; i++) param_val[i] = param_meta[i].def;
    param_apply_all();
    param_dirty = 1;        /* 恢复默认也算改动:退出界面时落盘,否则下次上电又读回旧值 */
}

void kart_params_init(void)
{
    uint8 i;

    kart_params_load_default();
    param_dirty = 0;        /* 上电阶段不算改动:载默认/载 Flash 都不该触发回写 */

    flash_read_page(0, KART_PARAMS_PAGE, page_buf, EEPROM_PAGE_LENGTH);

    /* count 必须与当前 KART_PARAM_MAX 完全一致:加/删参数后旧数据整表判废,
     * 避免按错位下标读到别的参数的值(比宁可用默认更危险)。 */
    if(page_buf[0] == KART_PARAMS_MAGIC && page_buf[1] == (uint32)KART_PARAM_MAX)
    {
        for(i = 0; i < KART_PARAM_MAX; i++)
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
    if(id >= KART_PARAM_MAX) return 0.0f;
    return param_val[id];
}

void kart_params_set(uint8 id, float v)
{
    if(id >= KART_PARAM_MAX) return;
    if(v < param_meta[id].min) v = param_meta[id].min;
    if(v > param_meta[id].max) v = param_meta[id].max;
    /* 值没变就不置脏:钳位到边界后反复按 UP 不会白擦一次 Flash。 */
    if(v != param_val[id]) param_dirty = 1;
    param_val[id] = v;
    param_apply(id);
}

void kart_params_step(uint8 id, int8 dir)
{
    if(id >= KART_PARAM_MAX) return;
    kart_params_set(id, param_val[id] + param_meta[id].step * (float)dir);
}

const kart_param_meta_t* kart_params_meta(uint8 id)
{
    if(id >= KART_PARAM_MAX) return &param_meta[0];
    return &param_meta[id];
}

uint8 kart_params_save(void)
{
    uint16 k;
    uint8  i;

    for(k = 0; k < EEPROM_PAGE_LENGTH; k++) page_buf[k] = 0u;

    page_buf[0] = KART_PARAMS_MAGIC;
    page_buf[1] = (uint32)KART_PARAM_MAX;
    for(i = 0; i < KART_PARAM_MAX; i++) page_buf[2 + i] = f2u(param_val[i]);

    /* 阻塞擦写一页(数 ms)。调用方保证车已停(菜单里只在静止界面提供该项)。 */
    flash_write_page(0, KART_PARAMS_PAGE, page_buf, EEPROM_PAGE_LENGTH);
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
