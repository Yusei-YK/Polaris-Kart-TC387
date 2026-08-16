/*********************************************************************************************************************
 * 文件名称  kart_vision
 * 功能说明  引导板色块检测实现。设计说明与标定方法见 kart_vision.h，此处只写实现细节。
 *
 * 2026-08-12 重写：色相扇区判别 + 种子 run 逐行传播连通域。
 * 2026-08-13 改动：先建掩膜再做十字腐蚀，和逐飞官方那套对齐；run 长度门退化成保险。
 * 为什么必须换掉老的色度比和全图 min/max 外接框，见 .h 文件头两大段，这里不重复。
 ********************************************************************************************************************/
#include "kart_vision.h"
#include "kart_preprocess.h"
#include <math.h>

/* 掩膜 19200B + 两条行备份 320B 全部放 cpu0 DSRAM，与 kart_preprocess.c 同处理。 */
#pragma section all "cpu0_dsram"

/*-------------------------------------------------------------------------------------------------------------------
 * 内部状态：只有一份结果快照。
 * 不做双缓冲 —— 本函数是同步调用，调用方拿到返回值才继续，不存在读写竞争。
 *-----------------------------------------------------------------------------------------------------------------*/
static kart_vision_result_t kart_vision_res;

/* 目标色掩膜。每像素 1 字节（0/1），不做位压缩：
 * 位压缩省到 2400B，但腐蚀要跨 32 位字边界取左右邻居，移位拼接容易写错，
 * 而这里省下的 17KB 并不解决任何实际问题。
 *
 * 【为什么加了掩膜反而更快】老实现里颜色判定会对同一个像素反复求值：
 * next_run 判完首像素还要在内层 while 里再判一次，第一趟找种子和第二趟传播又各扫一遍。
 * 色相里有一次整数除法，是本模块最贵的单步。现在每像素只求一次值，
 * 之后的腐蚀、找种子、传播全在掩膜上跑，只有字节比较。 */
static uint8 vis_mask[VISION_MASK_H][VISION_MASK_W];

/* 腐蚀用的行备份。腐蚀要就地做（省一份 19KB），而就地写会毁掉下一行需要的
 * "上一行原值"，所以留两条：prev 是上一行原值，cur 是本行原值。 */
static uint8 vis_row_prev[VISION_MASK_W];
static uint8 vis_row_cur[VISION_MASK_W];

/* 【掩膜语义换了：从三态标签改成存色相偏差本身】
 * 老实现每像素存 0/1/2(背景/候选/种子)，等于在建掩膜时就把 dev 判成了"过/不过"，
 * dev 这个连续量算完立刻丢掉。逆光时全图都不过 → 无种子 → 整帧丢，没有第二条路。
 *
 * 现在每像素存 dev 本身(0~126，252=无色相)，就是逐飞那套的第 2 步"目标色距离灰度图"。
 * 阈值判断推迟到用的时候，同一张图可以用不同的门跑两遍 —— 这才让"绝对门失守后
 * 改用全图最小 dev 起种"这件事不需要重扫原图(重扫 19200 px 带整数除法太贵)。
 *
 * bit7 复用成"已并入团块"标记：dev 真值最大 126(小于 0x80)，哨兵 252 会被夹到 127，
 * 所以 bit7 恒为 0，可以安全借用，不用第二份 19KB 数组。
 * 取 dev 一律用 VIS_DEV()，判是否已并入用 VIS_IS_TAKEN()，别直接读裸字节。 */
#define VIS_DEV_NONE                   (127U)  /* 哨兵：亮度/饱和度门挡掉，色相未参与 */
#define VIS_TAKEN_BIT                  (0x80U)
#define VIS_DEV(v)                     ((uint8)((v) & 0x7FU))
#define VIS_IS_TAKEN(v)                (((v) & VIS_TAKEN_BIT) != 0U)

/* 色相偏差的哨兵值：表示这个像素连色相都没算（暗部 / 灰白 / 三分量相等）。
 * 取色相圆周长 252，而真实偏差最大只有半圈 126，所以哨兵永远不会和真值撞上。
 * 判据是 dev < HUE_TOL，哨兵必然不通过，不需要额外的标志位。 */
#define VISION_HUE_NONE            (VISION_HUE_FULL)

/*-------------------------------------------------------------------------------------------------------------------
 * 色相偏差（整数版，全文件唯一一处颜色判断）
 *
 *   mx = max(R,G,B)   mn = min(R,G,B)   d = mx - mn
 *   mx==R → hue =   0 + 42*(G-B)/d      （负数 +252 折回）
 *   mx==G → hue =  84 + 42*(B-R)/d
 *   mx==B → hue = 168 + 42*(R-G)/d
 *   dev   = 色相圆上 hue 到 HUE_CENTER 的最短距离（0-126）
 *
 * 明度 mx 只出现在饱和度那道门的分母里，不进色相计算 —— 这是过曝免疫的全部来源：
 * 过曝时三个分量一起抬、d 一起缩，但 (G-B)/d 这个比值不变。
 *
 * 两道前置门都写成乘法形式，避免在每像素上做除法：
 *   亮度门   sum < MIN_SUM
 *   饱和度门 d/mx < MIN_SAT_PCT/100  →  d*100 < mx*MIN_SAT_PCT
 * 只有过了这两道门的像素才做那一次除法。
 *
 * 全部是 5 位量（0-31）的有符号整数运算，无浮点。
 * G-B / B-R / R-G 都可能为负，必须用 int16，不能图省事写 uint8。
 *
 * 出参可传 NULL。分量和 sum 在【任何返回路径上都先填好】—— 调试页要靠它们区分
 * "太暗"和"不够鲜艳"，提前返回时不填就等于把最需要的那一档信息弄丢了。
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 kart_vision_hue_deviation(uint16 pix, int16 *out_r5, int16 *out_g5,
                                       int16 *out_b5, int16 *out_sum)
{
    int16 r5  = (int16)VISION_R5(pix);
    int16 g5  = (int16)VISION_G5(pix);
    int16 b5  = (int16)VISION_B5(pix);
    int16 sum = (int16)(r5 + g5 + b5);
    int16 mx;
    int16 mn;
    int16 d;
    int16 hue;
    int16 dev;

    if(out_r5  != NULL) { *out_r5  = r5;  }
    if(out_g5  != NULL) { *out_g5  = g5;  }
    if(out_b5  != NULL) { *out_b5  = b5;  }
    if(out_sum != NULL) { *out_sum = sum; }

    /* 亮度门：暗处 d 只有个位数，(G-B)/d 的量化噪声能把色相甩出几十格 */
    if(sum < VISION_MIN_SUM)
    {
        return VISION_HUE_NONE;
    }

    mx = (r5 > g5) ? r5 : g5;
    if(b5 > mx) { mx = b5; }
    mn = (r5 < g5) ? r5 : g5;
    if(b5 < mn) { mn = b5; }
    d  = (int16)(mx - mn);

    /* 灰白：三分量相等时色相无定义，不是"某个颜色"，直接弃 */
    if(d <= 0)
    {
        return VISION_HUE_NONE;
    }

    /* 饱和度门。mx > 0 有保证（d > 0 蕴含 mx > 0），不会出现除零意义上的病态 */
    if(((int32)d * 100) < ((int32)mx * (int32)VISION_MIN_SAT_PCT))
    {
        return VISION_HUE_NONE;
    }

    if(mx == r5)
    {
        hue = (int16)((VISION_HUE_SECTOR * (g5 - b5)) / d);
    }
    else if(mx == g5)
    {
        hue = (int16)((VISION_HUE_SECTOR * 2) + ((VISION_HUE_SECTOR * (b5 - r5)) / d));
    }
    else
    {
        hue = (int16)((VISION_HUE_SECTOR * 4) + ((VISION_HUE_SECTOR * (r5 - g5)) / d));
    }

    if(hue < 0)
    {
        hue = (int16)(hue + VISION_HUE_FULL);
    }

    /* 圆上最短距离：先取绝对差，超过半圈就从另一边绕。 */
    dev = (int16)(hue - VISION_HUE_CENTER);
    if(dev < 0)
    {
        dev = (int16)(-dev);
    }
    if(dev > (VISION_HUE_FULL / 2))
    {
        dev = (int16)(VISION_HUE_FULL - dev);
    }

    return dev;
}

/* 掩膜单元 = 色相偏差本身，夹到 [0,127]。127 表示这个像素连色相都没算出来。
 * 不在这里做任何阈值判断 —— 那是回退逻辑要反复用不同门去问的事。 */
static uint8 vision_dev_cell(uint16 pix)
{
    int16 dev = kart_vision_hue_deviation(pix, NULL, NULL, NULL, NULL);

    if(dev < 0)   { return VIS_DEV_NONE; }
    if(dev > 126) { return VIS_DEV_NONE; }
    return (uint8)dev;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 建掩膜：0=背景，1=宽松候选，2=高置信种子
 *-----------------------------------------------------------------------------------------------------------------*/
/* 掩膜扫描顺带统计的三个计数,由 build_mask 写、process 读。纯诊断。 */
static int32 vis_stat_total;
static int32 vis_stat_dark;
static int32 vis_stat_over;
static int32 vis_stat_hue;

/* 全图最小色相偏差，由 build_mask 写。相对种子回退和 min_dev 诊断都读它。 */
static uint8 vis_min_dev;

/* 【纯诊断】vis_min_dev 那个像素的 d(=max-min)，和全图 sum 累加。
 * 跟着 vis_min_dev 一起更新，所以它答的是"最像目标色的那个像素还剩几级灰阶"，
 * 不是全图最大 d —— 后者会被背景高对比边缘(窗框、天空交界)占据，答的不是板子。 */
static uint8 vis_min_dev_d;
static int32 vis_sum_acc;

/* 宽松重建(L2)用的亮度/饱和度门。=0 走正常门。
 * 用 static 而不是加参数：hue_deviation 是被 probe_pixel(调试页) 共用的，
 * 给它加参数会连带改调试页的调用点，而调试页必须始终显示【正常门】下的判定 ——
 * 否则屏上看着过、跑车时判不过，比没有这个页面更坏。 */
static uint8 vis_loose_gate;

/* 宽松门版本的色相偏差。判据必须和 hue_deviation 逐字一致，只有两道前置门的
 * 阈值不同 —— 所以这里不能"顺手简化"，改 hue_deviation 时这里必须同步改。 */
static uint8 vision_dev_cell_loose(uint16 pix)
{
    int16 r5  = (int16)VISION_R5(pix);
    int16 g5  = (int16)VISION_G5(pix);
    int16 b5  = (int16)VISION_B5(pix);
    int16 sum = (int16)(r5 + g5 + b5);
    int16 mx;
    int16 mn;
    int16 d;
    int16 hue;
    int16 dev;

    if(sum < VISION_ADAPT_MIN_SUM) { return VIS_DEV_NONE; }

    mx = (r5 > g5) ? r5 : g5;
    if(b5 > mx) { mx = b5; }
    mn = (r5 < g5) ? r5 : g5;
    if(b5 < mn) { mn = b5; }
    d  = (int16)(mx - mn);

    if(d <= 0) { return VIS_DEV_NONE; }
    if(((int32)d * 100) < ((int32)mx * (int32)VISION_ADAPT_MIN_SAT_PCT))
    {
        return VIS_DEV_NONE;
    }

    if(mx == r5)
    {
        hue = (int16)((VISION_HUE_SECTOR * (g5 - b5)) / d);
    }
    else if(mx == g5)
    {
        hue = (int16)((VISION_HUE_SECTOR * 2) + ((VISION_HUE_SECTOR * (b5 - r5)) / d));
    }
    else
    {
        hue = (int16)((VISION_HUE_SECTOR * 4) + ((VISION_HUE_SECTOR * (r5 - g5)) / d));
    }

    if(hue < 0) { hue = (int16)(hue + VISION_HUE_FULL); }

    dev = (int16)(hue - VISION_HUE_CENTER);
    if(dev < 0) { dev = (int16)(-dev); }
    if(dev > (VISION_HUE_FULL / 2)) { dev = (int16)(VISION_HUE_FULL - dev); }
    if(dev > 126) { return VIS_DEV_NONE; }

    return (uint8)dev;
}

static void kart_vision_build_mask(const uint16 *img, int16 w, int16 h, int16 y_start)
{
    int16 y;
    int16 x;

    vis_stat_total = 0;
    vis_stat_dark  = 0;
    vis_stat_over  = 0;
    vis_stat_hue   = 0;
    vis_min_dev    = VIS_DEV_NONE;
    vis_min_dev_d  = 255U;
    vis_sum_acc    = 0;

    /* 【不能 memset 0】掩膜现在存 dev，0 表示"完美命中目标色"。
     * 清零等于把 ROI 以上整片刷成最强命中，种子会直接选在那里。填哨兵才是"空"。 */
    for(y = 0; y < y_start; y++)
    {
        memset(vis_mask[y], VIS_DEV_NONE, (size_t)w);
    }

    for(y = y_start; y < h; y++)
    {
        const uint16 *row  = &img[(int32)y * (int32)w];
        uint8        *mrow = vis_mask[y];

        for(x = 0; x < w; x++)
        {
            int16 r5 = (int16)VISION_R5(row[x]);
            int16 g5 = (int16)VISION_G5(row[x]);
            int16 b5 = (int16)VISION_B5(row[x]);
            int16 mx = (r5 > g5) ? r5 : g5;
            int16 mn = (r5 < g5) ? r5 : g5;
            uint8 dev;

            if(b5 > mx) { mx = b5; }
            if(b5 < mn) { mn = b5; }

            dev = (vis_loose_gate != 0U) ? vision_dev_cell_loose(row[x])
                                         : vision_dev_cell(row[x]);
            mrow[x] = dev;

            /* d 只在 min_dev 被刷新时才记：这两个量必须来自同一个像素，
             * 分开各取极值就变成两个无关的数，回答不了"这个色相可不可信"。 */
            if(dev < vis_min_dev)
            {
                vis_min_dev   = dev;
                vis_min_dev_d = (uint8)(mx - mn);
            }

            vis_sum_acc += (int32)(r5 + g5 + b5);

            vis_stat_total++;
            if((r5 + g5 + b5) < VISION_MIN_SUM)
            {
                vis_stat_dark++;
            }
            else if((r5 >= VISION_OVER_LEVEL) && (g5 >= VISION_OVER_LEVEL)
                    && (b5 >= VISION_OVER_LEVEL))
            {
                vis_stat_over++;
            }

            /* hue_pct 的语义保持不变：仍然是"进了候选门的像素占比"。
             * 这一路是现场判"色相窗口对不对"的唯一依据，语义不能跟着掩膜一起换。 */
            if(dev < VISION_HUE_TOL)
            {
                vis_stat_hue++;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 十字腐蚀一遍（就地）
 *
 * 结构元是"中心 + 上下左右"五点：只有自己和四个邻居都不是背景才留下。
 * 保留中心原来的 1/2 状态，因此腐蚀后仍能区分候选和种子。
 * 这是逐飞那套流程里的「腐蚀迭代」，比原来只看横向的 run 长度门强 ——
 * 一遍腐蚀就能杀掉任何一个方向上窄于 3px 的结构，包括纵向的毛刺和
 * 一像素宽的斜反光线，而 run 长度门对纵向的东西完全无能为力。
 *
 * 【图像边界当背景】最外一圈缺邻居，一律腐蚀掉。这里只用腐蚀结果算稳定中心，
 * 外接框和测距保留生长后的原始值，不受缩水影响。
 *
 * 【为什么敢就地做】只需要"上一行原值"，用 vis_row_prev 存着即可，
 * 不需要第二份完整掩膜。代价是每行两次 160 字节的行拷贝，可以忽略。
 *-----------------------------------------------------------------------------------------------------------------*/
static void kart_vision_erode_once(int16 w, int16 h, int16 y_start, uint8 tol)
{
    int16 y;
    int16 x;

    for(y = y_start; y < h; y++)
    {
        uint8       *mrow  = vis_mask[y];
        const uint8 *below = (y < (int16)(h - 1)) ? vis_mask[y + 1] : NULL;

        for(x = 0; x < w; x++)
        {
            vis_row_cur[x] = mrow[x];
        }

        for(x = 0; x < w; x++)
        {
            uint8 keep = vis_row_cur[x];

            if(VIS_DEV(keep) < tol)
            {
                /* 边界圈直接掉。放在最前面，后面就不用再判下标越界了。 */
                if((x == 0) || (x >= (int16)(w - 1)) || (y == y_start) || (y >= (int16)(h - 1)))
                {
                    keep = VIS_DEV_NONE;
                }
                else if((VIS_DEV(vis_row_cur[x - 1]) >= tol) || (VIS_DEV(vis_row_cur[x + 1]) >= tol))
                {
                    keep = VIS_DEV_NONE;
                }
                else if(VIS_DEV(vis_row_prev[x]) >= tol)
                {
                    keep = VIS_DEV_NONE;
                }
                else if(VIS_DEV(below[x]) >= tol)
                {
                    keep = VIS_DEV_NONE;
                }
            }
            else
            {
                keep = VIS_DEV_NONE;
            }

            mrow[x] = keep;
        }

        /* 存【原值】给下一行用，不能存刚写回去的腐蚀结果 —— 存错了就变成
         * 逐行累积腐蚀，一遍下来上面的行被啃掉的比下面多，团块整体上移。 */
        for(x = 0; x < w; x++)
        {
            vis_row_prev[x] = vis_row_cur[x];
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 行内取一条 run（在掩膜上）
 *
 * 从 x_from 开始向右找第一条长度 >= MIN_RUN 的连续段，返回是否找到。
 * 长度不足的段【直接跳过继续往右找】，不是返回失败。
 *
 * 【MIN_RUN 现在只是保险】杀噪点的活已经交给腐蚀了：一遍十字腐蚀就能吃掉
 * 长度 1-2 的横向段（两端像素必然缺一个横向邻居）。所以 MIN_RUN 取 1 时
 * 行为和取 3 基本一致，留着它是为了 ERODE_ITER 被改成 0 时还有一层兜底。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 kart_vision_next_run(const uint8 *mrow, int16 w, int16 x_from,
                                  int16 *run_lo, int16 *run_hi, uint8 tol)
{
    int16 x = x_from;

    while(x < w)
    {
        if(VIS_DEV(mrow[x]) < tol)
        {
            int16 lo = x;

            while((x < w) && (VIS_DEV(mrow[x]) < tol))
            {
                x++;
            }

            if((int16)(x - lo) >= VISION_MIN_RUN)
            {
                *run_lo = lo;
                *run_hi = (int16)(x - 1);
                return 1;
            }
            /* 太短，丢掉，从 x 继续往右找 */
        }
        else
        {
            x++;
        }
    }

    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 合并一行：把与 [band_lo-GAP, band_hi+GAP] 相交的 run 全部并入统计
 *
 * 返回本行并入的像素数（0 = 本行没有任何 run 与当前带相交，算一个空行）。
 * 同时把 band 更新为本行并入部分的实际范围 —— 带区间【跟着团块走】，
 * 这样斜置的板子能一行一行跟过去；如果固定用种子行的区间，斜到一定角度就断开。
 *
 * 只并"相交的"是关键：同一行里另一块独立的同色物体在横向上
 * 与当前带不重叠，就不会被并进来。老实现没有这一步，所以任何位置的同色像素都进框。
 *-----------------------------------------------------------------------------------------------------------------*/
/* Estimate vertical support at the midpoint of a seed run. */
static int16 kart_vision_seed_vertical_span(int16 cx, int16 y0, int16 h, uint8 tol)
{
    int16 top = y0;
    int16 bottom = y0;
    int16 skip = 0;
    int16 y;

    for(y = (int16)(y0 - 1); y >= 0; y--)
    {
        if(VIS_DEV(vis_mask[y][cx]) < tol)
        {
            top = y;
            skip = 0;
        }
        else
        {
            skip++;
            if(skip > VISION_MAX_SKIP)
            {
                break;
            }
        }
    }

    skip = 0;
    for(y = (int16)(y0 + 1); y < h; y++)
    {
        if(VIS_DEV(vis_mask[y][cx]) < tol)
        {
            bottom = y;
            skip = 0;
        }
        else
        {
            skip++;
            if(skip > VISION_MAX_SKIP)
            {
                break;
            }
        }
    }

    return (int16)(bottom - top + 1);
}

static int16 kart_vision_merge_row(uint8 *mrow, int16 w, int16 y,
                                   int16 *band_lo, int16 *band_hi,
                                   int32 *count, int32 *sum_x, int32 *sum_y,
                                   int16 *min_x, int16 *max_x, int16 *min_y, int16 *max_y,
                                   uint8 tol)
{
    int16 lim_lo = (int16)(*band_lo - VISION_ROW_GAP);
    int16 lim_hi = (int16)(*band_hi + VISION_ROW_GAP);
    int16 new_lo = 0;
    int16 new_hi = 0;
    int16 got    = 0;
    int16 taken  = 0;
    int16 x      = 0;
    int16 run_lo;
    int16 run_hi;

    while(kart_vision_next_run(mrow, w, x, &run_lo, &run_hi, tol))
    {
        x = (int16)(run_hi + 1);

        /* 区间相交测试。run 完全在带左侧就继续往右找，完全在右侧就可以收工了 ——
         * run 是按 x 递增取出来的，右侧之后不可能再有相交的。 */
        if(run_hi < lim_lo)
        {
            continue;
        }
        if(run_lo > lim_hi)
        {
            break;
        }

        /* 整条 run 并入，不做裁剪：run 是连通的一段，裁掉超出带的部分会把
         * 板子的斜边一层层削窄，宽度虚小、测距虚远。 */
        {
            int16 len = (int16)(run_hi - run_lo + 1);
            int16 i;

            for(i = run_lo; i <= run_hi; i++)
            {
                *sum_x += i;
                mrow[i] |= VIS_TAKEN_BIT;   /* 只打标记，dev 真值保留 */
            }
            *sum_y += (int32)y * (int32)len;
            *count += len;
            taken  = (int16)(taken + len);

            if(run_lo < *min_x) { *min_x = run_lo; }
            if(run_hi > *max_x) { *max_x = run_hi; }
            if(y      < *min_y) { *min_y = y;      }
            if(y      > *max_y) { *max_y = y;      }

            if(got == 0)
            {
                new_lo = run_lo;
                new_hi = run_hi;
                got    = 1;
            }
            else
            {
                if(run_lo < new_lo) { new_lo = run_lo; }
                if(run_hi > new_hi) { new_hi = run_hi; }
            }
        }
    }

    if(got != 0)
    {
        *band_lo = new_lo;
        *band_hi = new_hi;
    }

    return taken;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 一次完整的"找种子 + 区域生长"尝试
 *
 * 【为什么抽成函数】自适应回退要用不同的容差把同一张 dev 图跑两三遍。
 * 不抽出来就得把这七十行复制三份，改一处漏两处是必然的。
 *
 * seed_tol 决定"谁有资格起团"，grow_tol 决定"什么算团块的一部分"。
 * 回退只放宽 seed_tol，【grow_tol 一律保持 HUE_TOL 不动】——
 * 放宽生长门会把紧贴板子的地面并进框里(木地板 dev=16 就贴在 HUE_TOL 边上)，
 * 那正是 08-16 那次 width=160 大框、测距 0.45m、近距保护锁存、车不走的成因。
 * 起种门放宽只改变"从哪儿开始长"，不改变"能长到哪儿"，这是两者安全性的分界。
 *
 * 返回 1 = 长出了非空团块。面积门不在这里判，因为回退要先拿到团块再决定是否再试一级。
 *-----------------------------------------------------------------------------------------------------------------*/
typedef struct
{
    int32 count;
    int32 sum_x;
    int32 sum_y;
    int16 min_x;
    int16 max_x;
    int16 min_y;
    int16 max_y;
}vis_blob_t;

static uint8 vision_try_detect(int16 w, int16 h, int16 y_start,
                                    uint8 seed_tol, uint8 grow_tol, vis_blob_t *b)
{
    int16 seed_y        = -1;
    int16 seed_lo       = 0;
    int16 seed_hi       = 0;
    int16 seed_score    = 0;
    int32 seed_area_est = 0;
    int16 band_lo;
    int16 band_hi;
    int16 skip;
    int16 y;
    int16 x;

    b->count = 0;
    b->sum_x = 0;
    b->sum_y = 0;
    b->min_x = 0;
    b->max_x = 0;
    b->min_y = 0;
    b->max_y = 0;

    /* 抹掉上一次尝试留下的并入标记。回退是拿不同容差重跑【同一张】dev 图，
     * 标记不清就会把上一轮的团块也算进这一轮。只清 bit7，dev 真值不动。 */
    for(y = y_start; y < h; y++)
    {
        uint8 *mrow = vis_mask[y];
        for(x = 0; x < w; x++)
        {
            mrow[x] = VIS_DEV(mrow[x]);
        }
    }

    /* ---- 找种子：只有含 dev < seed_tol 像素的 run 才有资格起团 ---- */
    for(y = y_start; y < h; y++)
    {
        const uint8 *mrow = vis_mask[y];
        int16 run_lo;
        int16 run_hi;

        x = 0;
        while(kart_vision_next_run(mrow, w, x, &run_lo, &run_hi, grow_tol))
        {
            int16 len = (int16)(run_hi - run_lo + 1);
            int16 i;
            uint8 has_seed = 0;

            for(i = run_lo; i <= run_hi; i++)
            {
                if(VIS_DEV(mrow[i]) < seed_tol)
                {
                    has_seed = 1;
                    break;
                }
            }

            if(has_seed != 0)
            {
                int16 mid      = (int16)((run_lo + run_hi) / 2);
                int16 span     = kart_vision_seed_vertical_span(mid, y, h, grow_tol);
                int16 score    = (len < span) ? len : span;
                int32 area_est = (int32)len * (int32)span;

                /* 厚的二维物体优先于又长又薄的一条 —— 后者通常是地面或反光带。 */
                if((score > seed_score) ||
                   ((score == seed_score) && (area_est > seed_area_est)))
                {
                    seed_score    = score;
                    seed_area_est = area_est;
                    seed_y        = y;
                    seed_lo       = run_lo;
                    seed_hi       = run_hi;
                }
            }

            x = (int16)(run_hi + 1);
        }
    }

    if(seed_y < 0)
    {
        return 0;
    }

    /* ---- 从种子行向上下传播 ---- */
    b->min_x = seed_lo;
    b->max_x = seed_hi;
    b->min_y = seed_y;
    b->max_y = seed_y;

    band_lo = seed_lo;
    band_hi = seed_hi;
    (void)kart_vision_merge_row(vis_mask[seed_y], w, seed_y, &band_lo, &band_hi,
                                &b->count, &b->sum_x, &b->sum_y,
                                &b->min_x, &b->max_x, &b->min_y, &b->max_y, grow_tol);

    /* 向下。band 每行跟着团块走，所以斜置的板子也能一路跟下去。 */
    band_lo = seed_lo;
    band_hi = seed_hi;
    skip    = 0;
    for(y = (int16)(seed_y + 1); y < h; y++)
    {
        if(kart_vision_merge_row(vis_mask[y], w, y, &band_lo, &band_hi,
                                 &b->count, &b->sum_x, &b->sum_y,
                                 &b->min_x, &b->max_x, &b->min_y, &b->max_y, grow_tol) > 0)
        {
            skip = 0;
        }
        else
        {
            skip++;
            if(skip > VISION_MAX_SKIP) { break; }
        }
    }

    /* 向上。band 必须从种子行重新起算，不能接着上面那轮的值 —— 那是团块下缘的区间，
     * 拿它去匹配上缘会在板子斜置时直接匹配不上，只长出下半截。 */
    band_lo = seed_lo;
    band_hi = seed_hi;
    skip    = 0;
    for(y = (int16)(seed_y - 1); y >= y_start; y--)
    {
        if(kart_vision_merge_row(vis_mask[y], w, y, &band_lo, &band_hi,
                                 &b->count, &b->sum_x, &b->sum_y,
                                 &b->min_x, &b->max_x, &b->min_y, &b->max_y, grow_tol) > 0)
        {
            skip = 0;
        }
        else
        {
            skip++;
            if(skip > VISION_MAX_SKIP) { break; }
        }
    }

    return (uint8)((b->count > 0) ? 1 : 0);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 连续性门的跨帧状态
 *
 * 【只在锁还热的时候生效】cont_hold 每帧减一，成功一帧就重置回 HOLD_TICKS。
 * 连续失败 HOLD_TICKS 帧后归零、门自动放开。不这么做，第一次起锁和真正丢失后的
 * 重捕获会被自己拦死：那时没有可信的"上一帧位置"，任何位置都算跳变。
 *-----------------------------------------------------------------------------------------------------------------*/
static int16  vis_cont_cx   = 0;
static int16  vis_cont_cy   = 0;
static int16  vis_cont_w    = 0;
static uint8  vis_cont_hold = 0;
static uint16 vis_cont_rej  = 0;

/*-------------------------------------------------------------------------------------------------------------------
 * 处理一帧
 *
 * 四趟：
 *   第零趟 建三态掩膜（每像素求一次色相），不做前置腐蚀。
 *   第一趟 全图逐行取候选 run，只在含高置信像素的 run 里选最长一条作种子。
 *          弱命中不能独立起团，只能在第二趟被强种子扩张并入。
 *   第二趟 从种子行出发，先并种子行本身，再向下、向上逐行传播（"十字扩张"），
 *          连续空行超过 MAX_SKIP 就停，不扫完整幅图。
 *   第三趟 只保留选中区域，腐蚀其核心来算中心；原始框和面积用于测距与有效性门。
 *
 * 原图只在建掩膜时扫【一遍】，之后腐蚀/找种子/传播全在掩膜上跑。
 * 老实现反着来：没有掩膜，于是同一个像素的色相被反复求值（next_run 判首像素
 * 再判一次、两趟各扫一遍），而色相里有一次整数除法，是本模块最贵的单步。
 * 所以加了掩膜之后总耗时是降的，代价是 19.5KB DSRAM。
 * 原图缓冲同时是 DMA 的写入目标（靠 frame_ready/frame_release 互斥保护），
 * 只扫一遍也顺带缩短了持帧时间。
 *-----------------------------------------------------------------------------------------------------------------*/
const kart_vision_result_t *kart_vision_process(const uint16 *img, int16 w, int16 h)
{
    kart_vision_result_t *res = &kart_vision_res;

    int32 sum_x = 0;
    int32 sum_y = 0;
    int32 count = 0;
    int32 core_sum_x = 0;
    int32 core_sum_y = 0;
    int32 core_count = 0;
    int16 min_x = 0;
    int16 max_x = 0;
    int16 min_y = 0;
    int16 max_y = 0;
    int16 y_start = 0;  /* 黄板绑在上半身：必须扫描完整画面，禁止残留 ROI 裁掉目标 */
    vis_blob_t blob;
    uint8 min_dev_norm;
    int16 y;
    int16 bw;
    int16 bh;
    int16 aspect_x10;
    int32 fill_pct;
    float lens_theta;
    float lens_theta_abs;
    float lens_range_scale;

    /* 先整体清零：任何一条提前返回的路径都不会留下上一帧的残值 */
    res->valid       = 0;
    res->bearing_rad = 0.0f;
    res->dist_m      = 0.0f;
    res->cx_px       = 0;
    res->cy_px       = 0;
    res->width_px    = 0;
    res->height_px   = 0;
    res->area_px     = 0;
    res->reject      = VISION_REJ_AREA;
    res->confidence  = 0;
    res->box_x0      = 0;
    res->box_y0      = 0;
    res->dark_pct    = 0;
    res->over_pct    = 0;
    res->hue_pct     = 0;
    res->adapt_level = 0;
    res->min_dev     = VIS_DEV_NONE;
    res->min_dev_d   = 255U;
    res->mean_sum    = 0;
    res->cont_reject = vis_cont_rej;   /* 累计量，不清零，否则看不出差分 */

    if((img == NULL) || (w <= 0) || (h <= 0))
    {
        return res;
    }

    /* 掩膜是定长静态数组，图比它大就没法处理。返回"没看见"而不是截一部分算 ——
     * 截一部分会给出一个看着正常、实际上只覆盖左上角的结果，那比明确失败危险。 */
    if((w > VISION_MASK_W) || (h > VISION_MASK_H))
    {
        res->reject = VISION_REJ_AREA;
        return res;
    }

    if((y_start < 0) || (y_start >= h)) y_start = 0;

#if VISION_CONT_ENABLE
    /* 每帧先减一：任何一条失败路径都能让锁慢慢凉掉，不用在十几个 return 前各写一遍。
     * 成功帧在函数末尾会把它重置回 HOLD_TICKS。 */
    if(vis_cont_hold > 0U) { vis_cont_hold--; }
#endif

    /* ---------- 第零趟：建色相偏差图 ---------- */
    kart_vision_build_mask(img, w, h, y_start);

    /* 统计量在这里就落进 res:后面每一道几何门都会提前 return,放到函数末尾
     * 就只有成功帧才有统计 —— 而恰恰是失败帧才需要看这三个数。 */
    if(vis_stat_total > 0)
    {
        res->dark_pct = (uint8)((vis_stat_dark * 100) / vis_stat_total);
        res->over_pct = (uint8)((vis_stat_over * 100) / vis_stat_total);
        res->hue_pct  = (uint8)((vis_stat_hue  * 100) / vis_stat_total);
        res->mean_sum = (uint8)(vis_sum_acc / vis_stat_total);
    }

    /* 【必须在这里存下来】L2 回退会拿宽松门重建掩膜、把 vis_min_dev 覆盖掉，
     * 而现场要看的是"正常门下全图最像目标色的那个像素有多像" —— 只有这个数
     * 能回答"HUE_CENTER 是不是偏了"。宽松门下的最小值回答不了那个问题。 */
    min_dev_norm   = vis_min_dev;
    res->min_dev_d = vis_min_dev_d;

    /* ---------- 第一/二趟：找种子 + 生长。失败则两级回退 ---------- */
    /* L0 正常门。绝大多数帧到这里就结束了，回退分支一行也不执行。 */
    (void)vision_try_detect(w, h, y_start,
                                 VISION_HUE_SEED_TOL, VISION_HUE_TOL, &blob);

#if VISION_ADAPT_ENABLE
    /* 【回退的触发条件是"没长出够大的团块"，不只是"没找到种子"】
     * 逆光下更常见的形态不是一个种子都没有，而是只剩几个零星强命中像素、
     * 长出来的团块小于 MIN_AREA。那种帧和完全没种子一样是丢目标，一样该回退。
     *
     * 【一条必须知道的不变式】try_detect 进门就会清掉掩膜上的并入标记，
     * 所以一旦启动回退，上一级留下的标记就没了。这里靠"只在 count 更大时才接受
     * 新结果"来保证安全：被保留的 blob 如果不是最后一次 try_detect 的产物，
     * 它的 count 必然仍小于 MIN_AREA，下面第一道门就 return，
     * 永远走不到需要读并入标记的第三趟。 */
    if(blob.count < VISION_MIN_AREA)
    {
        vis_blob_t alt;
        uint8      st;

        /* ---- L1 相对种子：不重扫原图，直接在已有 dev 图上把种子门放到 min_dev+1 ----
         * 这就是"绝对判决 → 相对判决"的那一步。逆光下板子的 dev 从 1 抬到 11，
         * 绝对门 10 把它挡死，但它依然是全图最像目标色的那一块。 */
        st = (uint8)((vis_min_dev < 126U) ? (vis_min_dev + 1U) : 127U);
        if(st > (uint8)VISION_ADAPT_SEED_MAX) { st = (uint8)VISION_ADAPT_SEED_MAX; }

        /* st <= SEED_TOL 说明 L0 本来就有合格的种子像素，失败原因不在种子门
         * （比如 run 太短），拿一个不更宽的门重跑一遍是纯浪费。 */
        if(st > (uint8)VISION_HUE_SEED_TOL)
        {
            if(vision_try_detect(w, h, y_start, st, VISION_HUE_TOL, &alt) != 0U)
            {
                if(alt.count > blob.count)
                {
                    blob = alt;
                    res->adapt_level = 1;
                }
            }
        }

        /* ---- L2 宽松重建：连 L1 都无果，说明像素根本没进 dev 图 ----
         * 被亮度门 MIN_SUM=9 或饱和度门 MIN_SAT=25 拦在门外的像素，dev 是哨兵 127，
         * 种子门放多宽都救不回来 —— 必须重算一次色相。这一趟要重扫 19200 px
         * 带整数除法，是本模块最贵的单步，所以只在这条路上付。 */
        if(blob.count < VISION_MIN_AREA)
        {
            vis_loose_gate = 1;
            kart_vision_build_mask(img, w, h, y_start);
            vis_loose_gate = 0;   /* 立刻还原：后面任何一条 return 都不能把它留成 1 */

            st = (uint8)((vis_min_dev < 126U) ? (vis_min_dev + 1U) : 127U);
            if(st < (uint8)VISION_HUE_SEED_TOL) { st = (uint8)VISION_HUE_SEED_TOL; }
            if(st > (uint8)VISION_ADAPT_SEED_MAX) { st = (uint8)VISION_ADAPT_SEED_MAX; }

            if(vision_try_detect(w, h, y_start, st, VISION_HUE_TOL, &alt) != 0U)
            {
                if(alt.count > blob.count)
                {
                    blob = alt;
                    res->adapt_level = 2;
                }
            }

            if(blob.count < VISION_MIN_AREA)
            {
                res->adapt_level = 3;   /* 两级都没救回来 */
            }
        }
    }
#endif

    res->min_dev = min_dev_norm;

    count = blob.count;
    sum_x = blob.sum_x;
    sum_y = blob.sum_y;
    min_x = blob.min_x;
    max_x = blob.max_x;
    min_y = blob.min_y;
    max_y = blob.max_y;

    /* ---------- 四道门 ---------- */
    /* 面积不足：目标太远，或者种子只剩一小截。
     * 注意这里的 count 是【团块内】像素数，不是全图同色像素数（语义见 .h 的 area_px），
     * 所以换算法后这个数会明显变小，MIN_AREA 要重标。 */
    if(count < VISION_MIN_AREA)
    {
        res->area_px = (uint16)((count > 65535) ? 65535 : count);
        res->reject  = VISION_REJ_AREA;
        return res;
    }

    /* ---------- 第三趟：隔离选中区域，腐蚀核心只用于中心 ---------- */
    {
        int16 x;
        int16 it;

        /* 只留被选中团块的像素(bit7 有标记的)，其余全部置哨兵。
         * 保留 dev 真值、只清掉 bit7 —— 这样腐蚀继续按 HUE_TOL 判，
         * 不需要为"隔离后的掩膜"再造一套判据。 */
        for(y = y_start; y < h; y++)
        {
            uint8 *mrow = vis_mask[y];

            for(x = 0; x < w; x++)
            {
                mrow[x] = VIS_IS_TAKEN(mrow[x]) ? VIS_DEV(mrow[x]) : VIS_DEV_NONE;
            }
        }

        for(it = 0; it < VISION_ERODE_ITER; it++)
        {
            /* 【不能填 0】新语义下 0 = 完美命中。填 0 等于宣称上一行整行都是目标，
             * 第一行会被无条件保住，腐蚀在顶边失效。填哨兵才是"上面没有东西"。 */
            for(x = 0; x < w; x++)
            {
                vis_row_prev[x] = VIS_DEV_NONE;
            }
            kart_vision_erode_once(w, h, y_start, VISION_HUE_TOL);
        }

        for(y = y_start; y < h; y++)
        {
            for(x = 0; x < w; x++)
            {
                if(VIS_DEV(vis_mask[y][x]) < VISION_HUE_TOL)
                {
                    core_sum_x += x;
                    core_sum_y += y;
                    core_count++;
                }
            }
        }
    }

    bw = (int16)(max_x - min_x + 1);
    bh = (int16)(max_y - min_y + 1);

    /* 这几个量无论过不过门限都填上，方便现场看"差多少"而不是只看到一个失败标志 */
    res->cx_px     = (int16)((core_count > 0) ? (core_sum_x / core_count) : (sum_x / count));
    res->cy_px     = (int16)((core_count > 0) ? (core_sum_y / core_count) : (sum_y / count));
    res->width_px  = bw;
    res->height_px = bh;
    res->area_px   = (uint16)((count > 65535) ? 65535 : count);
    /* 画框、W/H、面积都来自腐蚀前的原始连通域。 */
    res->box_x0    = min_x;
    res->box_y0    = min_y;

    /* 太窄：宽度是测距的唯一依据，窄到几个像素时量化误差比测量值本身还大 */
    if(bw < VISION_MIN_WIDTH)
    {
        res->reject = VISION_REJ_WIDTH;
        return res;
    }

    /* 宽高比：竖板正对约 0.73。过扁或过宽的区域不用于跟随。 */
    aspect_x10 = (int16)(((int32)bw * 10) / (int32)bh);
    if((aspect_x10 < VISION_ASPECT_MIN_X10) || (aspect_x10 > VISION_ASPECT_MAX_X10))
    {
        res->reject = VISION_REJ_ASPECT;
        return res;
    }

    /* 填充率只挡斜置的细长反光带：外接框大、框内像素少。 */
    /* 用原始区域和原始框计算，分子分母同口径。 */
    fill_pct = ((int32)count * 100) /
               (((int32)max_x - (int32)min_x + 1) * ((int32)max_y - (int32)min_y + 1));
    if(fill_pct < VISION_MIN_FILL_PCT)
    {
        res->reject = VISION_REJ_FILL;
        return res;
    }

#if VISION_CONT_ENABLE
    /* ---------- 连续性门 ----------
     * 前面四道门问的都是"这一块像不像板子"，全是单帧几何，答不了
     * "这一块是不是【上一帧那块】板子"。.h 开头承认过：只看颜色无法区分两个同色物体，
     * 造图验证里也记着那条失效形态(板子 40px + 旁边第二块 60px → 选上第二块)。
     * 颜色维度已经没有信息可挖了，但时间维度有：帧间隔只有 25~27ms。 */
    if(vis_cont_hold > 0U)
    {
        int16 dx = (int16)(res->cx_px - vis_cont_cx);
        int16 dy = (int16)(res->cy_px - vis_cont_cy);
        uint8 jump = 0U;

        if(dx < 0) { dx = (int16)(-dx); }
        if(dy < 0) { dy = (int16)(-dy); }

        if((dx > VISION_CONT_MAX_JUMP_PX) || (dy > VISION_CONT_MAX_JUMP_PX))
        {
            jump = 1U;
        }

        /* 尺度跳变：一帧内宽度翻倍或减半。25ms 里距离不可能变一半，只能是换了物体。
         * 写成乘法而不是除法，既避开除零也避开整数截断。 */
        if((bw > (int16)(vis_cont_w * VISION_CONT_SCALE_NUM)) ||
           ((int16)(bw * VISION_CONT_SCALE_NUM) < vis_cont_w))
        {
            jump = 1U;
        }

        if(jump != 0U)
        {
            /* 【拒绝时不要清 hold】hold 在函数开头已经减过一次，会自己凉下去。
             * 在这里清零等于"被拦一次就彻底放开"，这道门就形同不存在 ——
             * 干扰物只要连续出现两帧就能抢走跟随目标。
             * reject 取值不能新增(kart_menu.c 和标定页按四个值分支)，借语义最近的 ASPECT，
             * 具体是哪一门拦的看 cont_reject 的差分。 */
            vis_cont_rej++;
            res->cont_reject = vis_cont_rej;
            res->reject      = VISION_REJ_ASPECT;
            return res;
        }
    }
#endif

    /* 130度等距鱼眼：r=f*theta，所以方位角直接等于像素半径/f。
     * 横向离轴时，竖向局部比例会被压缩；theta/sin(theta) 恢复斜距。
     * 只校正最终两个标量，不重采样160x120图像。 */
    lens_theta       = ((float)res->cx_px - ((float)w - 1.0f) * 0.5f) / VISION_FPX;
    lens_theta_abs   = (lens_theta < 0.0f) ? -lens_theta : lens_theta;
    lens_range_scale = 1.0f;

    if(lens_theta_abs > 0.01f)
    {
        lens_range_scale = lens_theta_abs / sinf(lens_theta_abs);
    }

    res->bearing_rad = lens_theta;
    res->dist_m      = ((VISION_BOARD_H_M * VISION_FPX) / (float)bh) * lens_range_scale;
    res->reject      = VISION_REJ_OK;
    res->valid       = 1;

#if VISION_CONT_ENABLE
    vis_cont_cx   = res->cx_px;
    vis_cont_cy   = res->cy_px;
    vis_cont_w    = bw;
    vis_cont_hold = VISION_CONT_HOLD_TICKS;
#endif

    /* 任一指标贴近门限，整体分数就低。当前只供VOFA和以后kart_vtrack融合。 */
    {
        int32 area_score   = 80 + ((count - VISION_MIN_AREA) * 175) / (VISION_MIN_AREA * 3);
        int32 width_score  = 80 + ((bw - VISION_MIN_WIDTH) * 175) / (VISION_MIN_WIDTH * 3);
        int32 fill_score   = 80 + ((fill_pct - VISION_MIN_FILL_PCT) * 175) / (70 - VISION_MIN_FILL_PCT);
        int32 aspect_mid   = (VISION_ASPECT_MIN_X10 + VISION_ASPECT_MAX_X10) / 2;
        int32 aspect_half  = (VISION_ASPECT_MAX_X10 - VISION_ASPECT_MIN_X10) / 2;
        int32 aspect_err   = (aspect_x10 > aspect_mid) ? (aspect_x10 - aspect_mid) : (aspect_mid - aspect_x10);
        int32 aspect_score = 255 - (aspect_err * 175) / aspect_half;
        int32 score = area_score;

        if(width_score < score) score = width_score;
        if(fill_score < score) score = fill_score;
        if(aspect_score < score) score = aspect_score;
        if(score < 80) score = 80;
        if(score > 255) score = 255;
        res->confidence = (uint8)score;
    }

    return res;
}

const kart_vision_result_t *kart_vision_get(void)
{
    return &kart_vision_res;
}

void kart_vision_reset(void)
{
    kart_vision_res.valid       = 0;
    kart_vision_res.bearing_rad = 0.0f;
    kart_vision_res.dist_m      = 0.0f;
    kart_vision_res.cx_px       = 0;
    kart_vision_res.cy_px       = 0;
    kart_vision_res.width_px    = 0;
    kart_vision_res.height_px   = 0;
    kart_vision_res.area_px     = 0;
    kart_vision_res.reject      = VISION_REJ_AREA;
    kart_vision_res.confidence  = 0;
    kart_vision_res.box_x0      = 0;
    kart_vision_res.box_y0      = 0;
    kart_vision_res.dark_pct    = 0;
    kart_vision_res.over_pct    = 0;
    kart_vision_res.hue_pct     = 0;
    kart_vision_res.adapt_level = 0;
    kart_vision_res.min_dev     = VIS_DEV_NONE;
    kart_vision_res.min_dev_d   = 255U;
    kart_vision_res.mean_sum    = 0;
    kart_vision_res.cont_reject = 0;

    /* 连续性状态必须一起清：reset 的语义是"忘掉之前看到的一切"。
     * 只清结果不清 hold，重新起跑的第一帧会拿上一轮的位置去卡新目标。 */
    vis_cont_cx   = 0;
    vis_cont_cy   = 0;
    vis_cont_w    = 0;
    vis_cont_hold = 0;
    vis_cont_rej  = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * ROI 色相统计（标定用，见 .h 的设计说明）
 *
 * 判据必须走 kart_vision_hue_deviation 的同一份实现，不能在这里照抄公式 ——
 * 抄一遍就是两份实现，改 BYTE_SWAP 或阈值时漏改一处，标定页量出来的中心
 * 和检测器实际用的窗口就不是一回事，那比没有这个功能更坏。
 * 但 deviation 只返回"到 CENTER 的距离"，标定需要的是绝对色相值，
 * 所以这里由 dev 反推：绝对 hue 落在 CENTER±dev 两侧，取与区域内多数一致的那一侧。
 * 反推会有一次二义性，用"先按 +dev 累计、再按 -dev 累计，取样本更集中的一组"解决。
 *
 * side 上限 31 -> 最多 961 个样本，插入排序 O(n^2) 最坏约 46 万次比较；
 * 本函数只在菜单标定页按键时调，不进跑车链路，这个代价可以接受。
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 vision_abs_hue(uint16 pix)
{
    int16 r5  = (int16)VISION_R5(pix);
    int16 g5  = (int16)VISION_G5(pix);
    int16 b5  = (int16)VISION_B5(pix);
    int16 sum = (int16)(r5 + g5 + b5);
    int16 mx;
    int16 mn;
    int16 d;
    int16 hue;

    if(sum < VISION_MIN_SUM) { return -1; }

    mx = (r5 > g5) ? r5 : g5;
    if(b5 > mx) { mx = b5; }
    mn = (r5 < g5) ? r5 : g5;
    if(b5 < mn) { mn = b5; }
    d  = (int16)(mx - mn);

    if(d <= 0) { return -2; }
    if(((int32)d * 100) < ((int32)mx * (int32)VISION_MIN_SAT_PCT)) { return -2; }

    if(mx == r5)
    {
        hue = (int16)((VISION_HUE_SECTOR * (g5 - b5)) / d);
    }
    else if(mx == g5)
    {
        hue = (int16)((VISION_HUE_SECTOR * 2) + ((VISION_HUE_SECTOR * (b5 - r5)) / d));
    }
    else
    {
        hue = (int16)((VISION_HUE_SECTOR * 4) + ((VISION_HUE_SECTOR * (r5 - g5)) / d));
    }

    if(hue < 0) { hue = (int16)(hue + VISION_HUE_FULL); }
    return hue;
}

void vision_roi_stat(const uint16 *img, int16 w, int16 h,
                          int16 cx, int16 cy, int16 side,
                          vision_roi_stat_t *out)
{
    static int16 hbuf[31 * 31];
    static int16 sbuf[31 * 31];
    static int16 mbuf[31 * 31];
    int16 half;
    int16 x;
    int16 y;
    int16 n     = 0;
    int32 total = 0;
    int32 dark  = 0;
    int32 lowsat= 0;
    int16 i;
    int16 j;

    if(out == NULL) { return; }

    out->med_hue = -1;  out->p10_hue = -1;  out->p90_hue = -1;
    out->ok_pct  = 0;   out->dark_pct = 0;   out->lowsat_pct = 0;
    out->med_sat_pct = 0; out->med_sum = 0;  out->n_total = 0;

    if((img == NULL) || (w <= 0) || (h <= 0)) { return; }

    if(side < 3)  { side = 3;  }
    if(side > 31) { side = 31; }
    if((side & 1) == 0) { side = (int16)(side - 1); }
    half = (int16)(side / 2);

    for(y = (int16)(cy - half); y <= (int16)(cy + half); y++)
    {
        if((y < 0) || (y >= h)) { continue; }
        for(x = (int16)(cx - half); x <= (int16)(cx + half); x++)
        {
            uint16 pix;
            int16  hue;

            if((x < 0) || (x >= w)) { continue; }

            pix = img[(int32)y * (int32)w + (int32)x];
            total++;
            hue = vision_abs_hue(pix);

            if(hue == -1)      { dark++;   continue; }
            else if(hue == -2) { lowsat++; continue; }

            {
                int16 r5 = (int16)VISION_R5(pix);
                int16 g5 = (int16)VISION_G5(pix);
                int16 b5 = (int16)VISION_B5(pix);
                int16 mx = (r5 > g5) ? r5 : g5;
                int16 mn = (r5 < g5) ? r5 : g5;
                int16 d;

                if(b5 > mx) { mx = b5; }
                if(b5 < mn) { mn = b5; }
                d = (int16)(mx - mn);

                hbuf[n] = hue;
                sbuf[n] = (int16)((mx > 0) ? (((int32)d * 100) / (int32)mx) : 0);
                mbuf[n] = (int16)(r5 + g5 + b5);
                n++;
            }
        }
    }

    out->n_total = (int16)total;
    if(total > 0)
    {
        out->dark_pct   = (uint8)((dark   * 100) / total);
        out->lowsat_pct = (uint8)((lowsat * 100) / total);
        out->ok_pct     = (uint8)(((int32)n * 100) / total);
    }

    /* 有效样本太少：宁可报 -1 也不给一个由三五个反光像素算出来的假中心。 */
    if(n < 5) { return; }

    /* 插入排序三条序列。色相绕零的情形（红色目标）这里不处理 ——
     * 已知边界：目标色相跨过 0/251 边界时中位数会算错，标定红色板要自己加 126 偏移。
     * 黄(42)、品红(210) 都远离边界，不受影响。 */
    for(i = 1; i < n; i++)
    {
        int16 kh = hbuf[i];
        int16 ks = sbuf[i];
        int16 km = mbuf[i];
        j = (int16)(i - 1);
        while((j >= 0) && (hbuf[j] > kh)) { hbuf[j + 1] = hbuf[j]; j--; }
        hbuf[j + 1] = kh;
        j = (int16)(i - 1);
        while((j >= 0) && (sbuf[j] > ks)) { sbuf[j + 1] = sbuf[j]; j--; }
        sbuf[j + 1] = ks;
        j = (int16)(i - 1);
        while((j >= 0) && (mbuf[j] > km)) { mbuf[j + 1] = mbuf[j]; j--; }
        mbuf[j + 1] = km;
    }

    out->med_hue     = hbuf[n / 2];
    out->p10_hue     = hbuf[(int16)(((int32)n * 10) / 100)];
    out->p90_hue     = hbuf[(int16)(((int32)n * 90) / 100)];
    out->med_sat_pct = (uint8)sbuf[n / 2];
    out->med_sum     = mbuf[n / 2];
}

/*-------------------------------------------------------------------------------------------------------------------
 * 单像素判别中间量（调试页用）
 *
 * 判据是 dev < HUE_TOL，两边都原样给出去：
 *   m_lhs = HUE_TOL（容差，恒定）   m_rhs = 本像素的色相偏差格数
 * 调试页那句 "M%6ld >%6ld" 不用改，显示出来就是"容差 6 > 偏差 3"。
 *
 * m_rhs 等于 252 是哨兵：亮度门或饱和度门把像素挡掉了，色相根本没参与判断
 * （灰白、暗部、三分量相等）。真实偏差最大只有 126，所以哨兵不会和真值混。
 *
 * 分量和 sum 由 kart_vision_hue_deviation 在所有返回路径上填好，
 * 这里不再自己提取一遍 —— 提取一遍就是两份实现，改 BYTE_SWAP 时漏改一处，
 * 调试页显示的 RGB 和检测器看到的就不是同一个像素。
 * 返回值走候选阈值；高置信种子只影响连通域从哪里开始，不改变准星读数。
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 kart_vision_probe_pixel(uint16 pix, int16 *r5, int16 *g5, int16 *b5,
                              int32 *m_lhs, int32 *m_rhs, int16 *sum)
{
    int16 dev = kart_vision_hue_deviation(pix, r5, g5, b5, sum);

    if(m_lhs != NULL) { *m_lhs = (int32)VISION_HUE_TOL; }
    if(m_rhs != NULL) { *m_rhs = (int32)dev; }

    return (uint8)(dev < VISION_HUE_TOL);
}

#pragma section all restore
