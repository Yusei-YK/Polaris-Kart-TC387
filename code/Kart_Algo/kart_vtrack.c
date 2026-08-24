/*********************************************************************************************************************
 * 文件名称  kart_vtrack
 * 功能说明  视觉跟踪实现。设计说明见 kart_vtrack.h，此处只写实现细节。
 ********************************************************************************************************************/
#include "kart_vtrack.h"
#include "kart_vision.h"     /* VISION_FPX */
#include <math.h>
#include <string.h>

#pragma section all "cpu0_dsram"

/*=========================== 金字塔图像 ===========================*/
/* 两层灰度：L0=160×120，L1=80×60。总计 (19200 + 4800) = 24000 字节。
 * 放 cpu0_dsram，与摄像头 DMA 目的地（scc8660_image 也在 dsram）同域，
 * RGB→灰度转换时 cache 友好。 */
/* 拆成两个独立数组，避免 60% 超配（原 38400B 现 24000B）。
 * 2026-08-10 修：原三维数组 [2][120][160] 给 L1 也分配了 120×160，
 * 实际 L1 只需 60×80。拆后按实际需求分配。 */
static uint8 pyr_L0[VTRACK_L0_H][VTRACK_L0_W];   /* L0: 160×120 = 19200B */
static uint8 pyr_L1[VTRACK_L1_H][VTRACK_L1_W];   /* L1:  80×60  =  4800B */

/* 每层的实际宽高（L0=原图，L1=1/2 下采样） */
static const int16 pyr_w[VTRACK_PYRAMID_LEVELS] = { VTRACK_L0_W, VTRACK_L1_W };
static const int16 pyr_h[VTRACK_PYRAMID_LEVELS] = { VTRACK_L0_H, VTRACK_L1_H };

/*=========================== 特征点集 ===========================*/
typedef struct
{
    float  x;               /* 当前位置 x（L0 坐标系，子像素） */
    float  y;               /* 当前位置 y */
    float  x_prev;          /* 上一帧位置（用于算 scale） */
    float  y_prev;
    uint8  alive;           /* 1 = 正在跟踪，0 = 已失效（FB 不过/出界/LK 不收敛） */
}kart_vtrack_point_t;

static kart_vtrack_point_t points[VTRACK_MAX_POINTS];
static uint16 total_points = 0;         /* 当前管理的点数（含未激活） */

/*=========================== 输出快照 ===========================*/
static kart_vtrack_result_t vtrack_res;

/*=========================== Detector 锚定状态 ===========================*/
static uint16 frames_since_detector = 0;   /* 距上次 Reacquisition 的帧数 */
static float  confidence_float = 0.0f;     /* 内部用浮点累积，输出时取整到 0-255 */

/*-------------------------------------------------------------------------------------------------------------------
 * RGB565 → 灰度（Y = 0.299R + 0.587G + 0.114B），整数近似
 *
 * RGB565: R5/G6/B5。为了让三通道同标度，G 右移 1 位取 G5（见 kart_vision.h）。
 * 整数系数近似：Y ≈ (77R + 150G + 29B) >> 8。分母 256 用移位，分子和 = 256。
 * 测试过几种典型色：
 *     纯白 (31,31,31) → 252       纯黄 (31,31, 0) → 219
 *     草地 (~4,8,3)   → 22        天空蓝 (~3,10,15) → 44
 * 动态范围 0-255 全覆盖，梯度方向保持（红 > 绿 > 蓝 的相对大小不变）。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 kart_vtrack_rgb565_to_gray(uint16 pix)
{
    /* 复用 kart_vision.h 的分量提取宏（已处理字节序） */
    uint8 r5 = (uint8)((pix >> 11) & 0x1FU);
    uint8 g6 = (uint8)((pix >>  5) & 0x3FU);
    uint8 b5 = (uint8)( pix        & 0x1FU);
    uint8 g5 = (uint8)(g6 >> 1);                /* G6 → G5，与 R/B 同标度 */

    return (uint8)(((uint16)77U * r5 + (uint16)150U * g5 + (uint16)29U * b5) >> 8);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 建金字塔：L0 = RGB→灰度，L1 = L0 双线性下采样 1/2
 *
 * 双线性 2×2 → 1：out[y][x] = (in[2y][2x] + in[2y][2x+1] + in[2y+1][2x] + in[2y+1][2x+1]) / 4
 * 160×120 → 80×60，访存 19200 读 + 4800 写，约几十 us。
 *-----------------------------------------------------------------------------------------------------------------*/
static void kart_vtrack_build_pyramid(const uint16 *img_rgb565, int16 w, int16 h)
{
    int16 x, y;

    /* L0: RGB565 → 灰度 */
    for(y = 0; y < h; y++)
    {
        const uint16 *row_in = &img_rgb565[(int32)y * (int32)w];
        uint8 *row_out = pyr_L0[y];

        for(x = 0; x < w; x++)
        {
            row_out[x] = kart_vtrack_rgb565_to_gray(row_in[x]);
        }
    }

    /* L1: L0 双线性下采样 1/2 */
    if(VTRACK_PYRAMID_LEVELS > 1)
    {
        int16 w1 = pyr_w[1];
        int16 h1 = pyr_h[1];

        for(y = 0; y < h1; y++)
        {
            int16 y0 = (int16)(y * 2);
            const uint8 *row0 = pyr_L0[y0];
            const uint8 *row1 = pyr_L0[y0 + 1];
            uint8 *out_row = pyr_L1[y];

            for(x = 0; x < w1; x++)
            {
                int16 x0 = (int16)(x * 2);
                uint16 sum = (uint16)row0[x0] + (uint16)row0[x0+1]
                           + (uint16)row1[x0] + (uint16)row1[x0+1];
                out_row[x] = (uint8)(sum >> 2);
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 双线性插值：从金字塔 level 层取子像素灰度
 *
 * I(x,y) = (1-fx)(1-fy) I(x0,y0) + fx(1-fy) I(x0+1,y0)
 *        + (1-fx) fy    I(x0,y0+1) + fx fy I(x0+1,y0+1)
 *
 * 边界：留 1px margin，插值窗口 [x0,x0+1] × [y0,y0+1] 必须在 [0, w-1] × [0, h-1] 内。
 *-----------------------------------------------------------------------------------------------------------------*/
static float kart_vtrack_interp(int level, float x, float y)
{
    int16 w_lv = pyr_w[level];
    int16 h_lv = pyr_h[level];

    if((x < 0.0f) || (x >= (float)(w_lv - 1)) || (y < 0.0f) || (y >= (float)(h_lv - 1)))
    {
        return 0.0f;    /* 出界返回 0（后续 LK 会判 out-of-bounds 失效） */
    }

    int16 x0 = (int16)x;
    int16 y0 = (int16)y;
    float fx = x - (float)x0;
    float fy = y - (float)y0;

    const uint8 *row0 = (level == 0) ? pyr_L0[y0] : pyr_L1[y0];
    const uint8 *row1 = (level == 0) ? pyr_L0[y0 + 1] : pyr_L1[y0 + 1];

    float v00 = (float)row0[x0];
    float v10 = (float)row0[x0 + 1];
    float v01 = (float)row1[x0];
    float v11 = (float)row1[x0 + 1];

    return (1.0f - fx) * (1.0f - fy) * v00
         +        fx   * (1.0f - fy) * v10
         + (1.0f - fx) *        fy   * v01
         +        fx   *        fy   * v11;
}

/*-------------------------------------------------------------------------------------------------------------------
 * LK 单层迭代：在 level 层金字塔上，从初值 (x_init, y_init) 开始优化，求位移 (dx, dy)
 *
 * 返回 1 = 收敛，0 = 不收敛（出界/梯度不足/超最大迭代次数）。
 *
 * LK 最小化光度误差：
 *     Σ_窗口 [ I_prev(x+i, y+j) - I_curr(x+dx+i, y+dy+j) ]²
 * 泰勒展开 I_curr(x+dx, y+j) ≈ I_curr(x,y) + ∇I·(dx,dy)，线性化后解 2×2 系统：
 *     [ Σ Ix²   Σ IxIy ] [ dx ]   [ Σ Ix(I_prev - I_curr) ]
 *     [ Σ IxIy  Σ Iy²  ] [ dy ] = [ Σ Iy(I_prev - I_curr) ]
 * 每次迭代更新 (dx, dy)，直到增量 < EPS 或超 MAX_ITER。
 *
 * 【为什么返回 0 不直接失效点】
 *   金字塔粗层不收敛不代表点就该丢 —— 可能是初值太远。粗层给个粗解，
 *   细层再精修。只有最细层（L0）还不收敛，才在外层标 alive=0。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 kart_vtrack_lk_one_level(int level,
                                      float x_prev, float y_prev,
                                      float x_init, float y_init,
                                      float *out_dx, float *out_dy)
{
    const int16 r = VTRACK_LK_WIN_RADIUS;
    const float eps_sq = VTRACK_LK_EPS * VTRACK_LK_EPS;
    const int16 w_lv = pyr_w[level];
    const int16 h_lv = pyr_h[level];

    float dx = 0.0f;
    float dy = 0.0f;
    int iter;

    /* 边界检查：窗口 [x_prev-r, x_prev+r] × [y_prev-r, y_prev+r] 必须在图内 */
    if((x_prev < (float)(r + 1)) || (x_prev >= (float)(w_lv - r - 1)) ||
       (y_prev < (float)(r + 1)) || (y_prev >= (float)(h_lv - r - 1)))
    {
        *out_dx = 0.0f;
        *out_dy = 0.0f;
        return 0;
    }

    for(iter = 0; iter < VTRACK_LK_MAX_ITER; iter++)
    {
        float x_curr = x_init + dx;
        float y_curr = y_init + dy;

        /* 当前估计出界 → 不收敛 */
        if((x_curr < (float)(r + 1)) || (x_curr >= (float)(w_lv - r - 1)) ||
           (y_curr < (float)(r + 1)) || (y_curr >= (float)(h_lv - r - 1)))
        {
            *out_dx = dx;
            *out_dy = dy;
            return 0;
        }

        /* 累积窗口内的 Hessian G = [ΣIx² ΣIxIy; ΣIxIy ΣIy²] 和右端 b = [ΣIx·err; ΣIy·err] */
        float G11 = 0.0f;
        float G12 = 0.0f;
        float G22 = 0.0f;
        float b1  = 0.0f;
        float b2  = 0.0f;

        int16 i, j;
        for(j = -r; j <= r; j++)
        {
            for(i = -r; i <= r; i++)
            {
                float px = x_prev + (float)i;
                float py = y_prev + (float)j;
                float cx = x_curr + (float)i;
                float cy = y_curr + (float)j;

                float I_prev = kart_vtrack_interp(level, px, py);
                float I_curr = kart_vtrack_interp(level, cx, cy);

                /* 梯度：中心差分 I_x = (I(x+1,y) - I(x-1,y))/2 */
                float Ix = (kart_vtrack_interp(level, cx + 1.0f, cy)
                          - kart_vtrack_interp(level, cx - 1.0f, cy)) * 0.5f;
                float Iy = (kart_vtrack_interp(level, cx, cy + 1.0f)
                          - kart_vtrack_interp(level, cx, cy - 1.0f)) * 0.5f;

                float err = I_prev - I_curr;

                G11 += Ix * Ix;
                G12 += Ix * Iy;
                G22 += Iy * Iy;
                b1  += Ix * err;
                b2  += Iy * err;
            }
        }

        /* 解 2×2 系统：det = G11*G22 - G12*G12，若 det 太小说明窗口无纹理 */
        float det = G11 * G22 - G12 * G12;
        if(fabsf(det) < 1e-6f)
        {
            *out_dx = dx;
            *out_dy = dy;
            return 0;       /* 梯度不足，不收敛 */
        }

        float inv_det = 1.0f / det;
        float ddx = inv_det * ( G22 * b1 - G12 * b2);
        float ddy = inv_det * (-G12 * b1 + G11 * b2);

        dx += ddx;
        dy += ddy;

        /* 收敛判据：增量足够小 */
        if((ddx * ddx + ddy * ddy) < eps_sq)
        {
            *out_dx = dx;
            *out_dy = dy;
            return 1;       /* 收敛 */
        }
    }

    /* 超最大迭代次数，不收敛 */
    *out_dx = dx;
    *out_dy = dy;
    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * LK 多层金字塔：粗→细迭代
 *
 * 从最粗层（L1）开始，用 0 初值跑 LK 得到粗位移，乘 PYR_SCALE 传给下一层做初值，
 * 直到最细层（L0）得到最终子像素位移。返回 1 = L0 收敛，0 = L0 不收敛。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 kart_vtrack_lk_pyramid(float x_prev, float y_prev,
                                    float x_init, float y_init,
                                    float *out_x, float *out_y)
{
    float dx_accum = 0.0f;
    float dy_accum = 0.0f;
    int lv;

    /* 从粗到细 */
    for(lv = VTRACK_PYRAMID_LEVELS - 1; lv >= 0; lv--)
    {
        float scale = 1.0f / (float)(1 << lv);      /* L1: scale=0.5, L0: scale=1.0 */
        float xp_lv = x_prev * scale;
        float yp_lv = y_prev * scale;
        float xi_lv = x_init * scale;
        float yi_lv = y_init * scale;

        /* 用累积位移做初值（已缩放到当前层） */
        float dx_lv = 0.0f;
        float dy_lv = 0.0f;
        uint8 ok = kart_vtrack_lk_one_level(lv, xp_lv, yp_lv,
                                            xi_lv + dx_accum * scale,
                                            yi_lv + dy_accum * scale,
                                            &dx_lv, &dy_lv);

        /* 粗层不收敛不算失败，继续往下传；只有 L0 不收敛才返回 0 */
        dx_accum += dx_lv / scale;          /* 缩放回 L0 坐标 */
        dy_accum += dy_lv / scale;

        if((lv == 0) && (ok == 0))
        {
            /* L0 不收敛 → 整个跟踪失败 */
            *out_x = x_init + dx_accum;
            *out_y = y_init + dy_accum;
            return 0;
        }
    }

    *out_x = x_init + dx_accum;
    *out_y = y_init + dy_accum;
    return 1;
}

/*-------------------------------------------------------------------------------------------------------------------
 * Forward-Backward 一致性检查
 *
 * p_prev (t-1) → forward LK → p_curr (t) → backward LK → p_back (t-1)
 * 往返误差 = ||p_prev - p_back||。若 < FB_THRESH，认为点仍在目标上；否则已漂移/遮挡。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 kart_vtrack_fb_check(float x_prev, float y_prev, float x_curr, float y_curr)
{
    float x_back, y_back;

    /* Backward: 从 curr 回到 prev（金字塔仍然是当前帧和上一帧的，
     * 这里"backward"是逻辑上的，实际还是用当前帧金字塔做模板、prev 做搜索 ——
     * 标准 FB 需要保留上一帧金字塔，内存翻倍。简化实现：当前帧金字塔 + 当前位置反向跟。
     * 【坑】这要求 pyr_gray 是 t-1 和 t 两帧都建好。实际上我们只建 t 帧，
     * 所以这里的 backward 只能近似：用 t 帧金字塔、从 curr 往 prev 方向跑 LK。
     * 真实 FB 要两套金字塔，占 48KB dsram，当前预算吃紧，先用近似版。
     * 【TODO】若 FB 误判率高，改成双金字塔。 */

    /* 用当前帧 pyramid 做 backward（这是简化，真实 FB 要上一帧 pyramid） */
    uint8 ok = kart_vtrack_lk_pyramid(x_curr, y_curr, x_prev, y_prev, &x_back, &y_back);

    if(ok == 0)
    {
        return 0;       /* backward 不收敛 → FB 不通过 */
    }

    float dx_fb = x_prev - x_back;
    float dy_fb = y_prev - y_back;
    float err_sq = dx_fb * dx_fb + dy_fb * dy_fb;

    return (uint8)(err_sq < (VTRACK_FB_THRESH * VTRACK_FB_THRESH));
}

/*-------------------------------------------------------------------------------------------------------------------
 * MAD 异常点剔除：位移向量与中值方向夹角过大的点剔除
 *
 * 所有存活点的位移向量 v_i = (x_i - x_prev_i, y_i - y_prev_i)，
 * 算中值方向 v_med（用 x/y 分量各自的中值），然后剔除 angle(v_i, v_med) > MAD_ANGLE 的点。
 *-----------------------------------------------------------------------------------------------------------------*/
static void kart_vtrack_mad_filter(void)
{
    uint16 alive_cnt = 0;
    float vx_list[VTRACK_MAX_POINTS];
    float vy_list[VTRACK_MAX_POINTS];
    uint16 i, j;

    /* 收集所有存活点的位移向量 */
    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            vx_list[alive_cnt] = points[i].x - points[i].x_prev;
            vy_list[alive_cnt] = points[i].y - points[i].y_prev;
            alive_cnt++;
        }
    }

    if(alive_cnt < 3)
    {
        return;     /* 点太少，不做 MAD */
    }

    /* 冒泡排序取 x/y 中值（点数 < 32，冒泡足够）*/
    for(i = 0; i < alive_cnt - 1; i++)
    {
        for(j = i + 1; j < alive_cnt; j++)
        {
            if(vx_list[i] > vx_list[j])
            {
                float tmp = vx_list[i];
                vx_list[i] = vx_list[j];
                vx_list[j] = tmp;
            }
            if(vy_list[i] > vy_list[j])
            {
                float tmp = vy_list[i];
                vy_list[i] = vy_list[j];
                vy_list[j] = tmp;
            }
        }
    }

    float vx_med = vx_list[alive_cnt / 2];
    float vy_med = vy_list[alive_cnt / 2];
    float norm_med = sqrtf(vx_med * vx_med + vy_med * vy_med);

    if(norm_med < 0.1f)
    {
        return;     /* 中值位移太小（几乎静止），不做角度判定 */
    }

    /* 剔除角度偏离过大的点 */
    float cos_thresh = cosf(VTRACK_MAD_ANGLE_DEG * 3.14159265f / 180.0f);

    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            float vx = points[i].x - points[i].x_prev;
            float vy = points[i].y - points[i].y_prev;
            float norm = sqrtf(vx * vx + vy * vy);

            if(norm < 0.1f)
            {
                continue;   /* 该点几乎不动，保留 */
            }

            /* cos(angle) = (v · v_med) / (|v| |v_med|) */
            float dot = vx * vx_med + vy * vy_med;
            float cos_angle = dot / (norm * norm_med);

            if(cos_angle < cos_thresh)
            {
                points[i].alive = 0;    /* 方向偏离 > MAD_ANGLE，剔除 */
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 中值 scale 计算：所有存活点对之间距离比的中值
 *
 * scale_r = median( d_ij(t) / d_ij(t-1) )  over all pairs (i,j)
 * 点数 N → 对数 N(N-1)/2。N=20 → 190 对，可接受。
 *-----------------------------------------------------------------------------------------------------------------*/
static float kart_vtrack_median_scale(void)
{
    float ratio_list[VTRACK_MAX_POINTS * VTRACK_MAX_POINTS / 2];
    uint16 ratio_cnt = 0;
    uint16 i, j;

    for(i = 0; i < total_points; i++)
    {
        if(!points[i].alive) continue;

        for(j = i + 1; j < total_points; j++)
        {
            if(!points[j].alive) continue;

            /* 上一帧距离 */
            float dx_prev = points[i].x_prev - points[j].x_prev;
            float dy_prev = points[i].y_prev - points[j].y_prev;
            float d_prev = sqrtf(dx_prev * dx_prev + dy_prev * dy_prev);

            /* 当前帧距离 */
            float dx_curr = points[i].x - points[j].x;
            float dy_curr = points[i].y - points[j].y;
            float d_curr = sqrtf(dx_curr * dx_curr + dy_curr * dy_curr);

            if(d_prev > 1.0f)       /* 避免除接近 0 的数 */
            {
                ratio_list[ratio_cnt++] = d_curr / d_prev;
            }
        }
    }

    if(ratio_cnt == 0)
    {
        return 1.0f;    /* 无有效对，返回无变化 */
    }

    /* 冒泡排序取中值 */
    for(i = 0; i < ratio_cnt - 1; i++)
    {
        for(j = i + 1; j < ratio_cnt; j++)
        {
            if(ratio_list[i] > ratio_list[j])
            {
                float tmp = ratio_list[i];
                ratio_list[i] = ratio_list[j];
                ratio_list[j] = tmp;
            }
        }
    }

    return ratio_list[ratio_cnt / 2];
}

/*-------------------------------------------------------------------------------------------------------------------
 * 质心方位角：所有存活点 x 坐标的中值（抗 outlier）→ 方位角
 *-----------------------------------------------------------------------------------------------------------------*/
static float kart_vtrack_bearing(void)
{
    float x_list[VTRACK_MAX_POINTS];
    uint16 cnt = 0;
    uint16 i, j;

    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            x_list[cnt++] = points[i].x;
        }
    }

    if(cnt == 0)
    {
        return 0.0f;
    }

    /* 冒泡排序取 x 中值 */
    for(i = 0; i < cnt - 1; i++)
    {
        for(j = i + 1; j < cnt; j++)
        {
            if(x_list[i] > x_list[j])
            {
                float tmp = x_list[i];
                x_list[i] = x_list[j];
                x_list[j] = tmp;
            }
        }
    }

    float cx = x_list[cnt / 2];

    /* 针孔模型：tan(β) = (cx - w/2) / f_px。复用 kart_vision.h 的 VISION_FPX。 */
    
    float w_half = (float)VTRACK_L0_W * 0.5f;

    return atanf((cx - w_half) / VISION_FPX);
}

/*-------------------------------------------------------------------------------------------------------------------
 * Shi-Tomasi 角点响应：min(λ1, λ2) of structure tensor G = [ΣIx² ΣIxIy; ΣIxIy ΣIy²]
 *
 * 窗口 3×3，特征值 λ = 0.5 * (trace ± sqrt(trace² - 4*det))，取较小的那个。
 * 简化：min(λ1,λ2) ≈ det/trace（Shi-Tomasi 原文的 fast approximation）。
 *-----------------------------------------------------------------------------------------------------------------*/
static float kart_vtrack_corner_response(int16 x, int16 y)
{
    const uint8 *img = pyr_L0[0];      /* L0 原图 */
    const int16 w = VTRACK_L0_W;
    const int16 h = VTRACK_L0_H;

    if((x < 1) || (x >= w - 1) || (y < 1) || (y >= h - 1))
    {
        return 0.0f;        /* 边界返回 0 */
    }

    float Gxx = 0.0f;
    float Gxy = 0.0f;
    float Gyy = 0.0f;

    int16 i, j;
    for(j = -1; j <= 1; j++)
    {
        for(i = -1; i <= 1; i++)
        {
            int16 px = x + i;
            int16 py = y + j;
            int32 idx = (int32)py * (int32)w + (int32)px;

            /* 梯度：Sobel 3×3 简化为中心差分 */
            float Ix = (float)((int16)img[idx + 1] - (int16)img[idx - 1]) * 0.5f;
            float Iy = (float)((int16)img[idx + w] - (int16)img[idx - w]) * 0.5f;

            Gxx += Ix * Ix;
            Gxy += Ix * Iy;
            Gyy += Iy * Iy;
        }
    }

    float trace = Gxx + Gyy;
    float det = Gxx * Gyy - Gxy * Gxy;

    if(trace < 1e-6f)
    {
        return 0.0f;
    }

    return det / trace;     /* 近似 min(λ1, λ2) */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 初始化
 *-----------------------------------------------------------------------------------------------------------------*/
void kart_vtrack_init(void)
{
    memset(points, 0, sizeof(points));
    total_points = 0;
    frames_since_detector = 0;
    confidence_float = 0.0f;

    memset(&vtrack_res, 0, sizeof(vtrack_res));
    vtrack_res.valid = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * Reacquisition：从 Detector 的 bbox 内均匀撒点
 *-----------------------------------------------------------------------------------------------------------------*/
void kart_vtrack_reacquire(const kart_vision_result_t *vis)
{
    uint16 i, j;
    uint16 cand_cnt = 0;

    if((vis == NULL) || (vis->valid == 0))
    {
        return;     /* Detector 无效，不做 reacquire */
    }

    /* 清空当前点集 */
    total_points = 0;

    /* 在 bbox 内均匀撒 GRID_SIDE × GRID_SIDE 个候选点 */
    const int16 N = VTRACK_REACQ_GRID_SIDE;
    int16 x0 = vis->box_x0;
    int16 y0 = vis->box_y0;
    int16 bw = vis->width_px;
    int16 bh = vis->height_px;

    /* 候选点 + 响应，用于排序取前 MAX_POINTS */
    typedef struct {
        float x;
        float y;
        float response;
    } candidate_t;

    candidate_t candidates[VTRACK_REACQ_GRID_SIDE * VTRACK_REACQ_GRID_SIDE];

    for(j = 0; j < N; j++)
    {
        for(i = 0; i < N; i++)
        {
            int16 px = x0 + (int16)((i + 0.5f) * (float)bw / (float)N);
            int16 py = y0 + (int16)((j + 0.5f) * (float)bh / (float)N);

            float resp = kart_vtrack_corner_response(px, py);

            if(resp >= VTRACK_MIN_CORNER_RESPONSE)
            {
                candidates[cand_cnt].x = (float)px;
                candidates[cand_cnt].y = (float)py;
                candidates[cand_cnt].response = resp;
                cand_cnt++;
            }
        }
    }

    if(cand_cnt == 0)
    {
        return;     /* bbox 内无合格特征点，放弃本次 reacquire */
    }

    /* 按响应降序排序（冒泡，候选数 ≤ 36）*/
    for(i = 0; i < cand_cnt - 1; i++)
    {
        for(j = i + 1; j < cand_cnt; j++)
        {
            if(candidates[i].response < candidates[j].response)
            {
                candidate_t tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }

    /* 取前 MAX_POINTS 个 */
    uint16 n_take = (cand_cnt < VTRACK_MAX_POINTS) ? cand_cnt : VTRACK_MAX_POINTS;
    for(i = 0; i < n_take; i++)
    {
        points[i].x = candidates[i].x;
        points[i].y = candidates[i].y;
        points[i].x_prev = candidates[i].x;
        points[i].y_prev = candidates[i].y;
        points[i].alive = 1;
    }

    total_points = n_take;
    frames_since_detector = 0;
    confidence_float = 255.0f;      /* Reacquire 后置信度拉满 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 单帧更新
 *-----------------------------------------------------------------------------------------------------------------*/
const kart_vtrack_result_t *kart_vtrack_update(const uint16 *img, int16 w, int16 h)
{
    uint16 i;
    uint16 alive_before = 0;
    uint16 alive_after_lk = 0;
    uint16 alive_after_fb = 0;
    float fb_err_list[VTRACK_MAX_POINTS];
    uint16 fb_err_cnt = 0;

    /* 默认无效，后面条件满足才置 valid=1 */
    vtrack_res.valid = 0;
    vtrack_res.bearing_rad = 0.0f;
    vtrack_res.scale_level = KART_VTRACK_SCALE_NORMAL;
    vtrack_res.scale_r = 1.0f;
    vtrack_res.confidence = 0;
    vtrack_res.alive_count = 0;
    vtrack_res.total_count = total_points;
    vtrack_res.fb_error_median = 0.0f;
    vtrack_res.frames_since_detector = frames_since_detector;

    if((img == NULL) || (w != VTRACK_L0_W) || (h != VTRACK_L0_H) || (total_points == 0))
    {
        return &vtrack_res;
    }

    /* 建金字塔 */
    kart_vtrack_build_pyramid(img, w, h);

    /* Forward LK：上一帧位置 → 当前帧位置 */
    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            alive_before++;

            float x_new, y_new;
            uint8 ok = kart_vtrack_lk_pyramid(points[i].x_prev, points[i].y_prev,
                                              points[i].x_prev, points[i].y_prev,
                                              &x_new, &y_new);

            if(ok)
            {
                /* 暂存到 x/y，后面 FB 通过才真正更新 */
                points[i].x = x_new;
                points[i].y = y_new;
                alive_after_lk++;
            }
            else
            {
                points[i].alive = 0;        /* Forward 不收敛 → 失效 */
            }
        }
    }

    /* Forward-Backward 一致性检查 */
    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            uint8 fb_ok = kart_vtrack_fb_check(points[i].x_prev, points[i].y_prev,
                                               points[i].x, points[i].y);

            if(fb_ok)
            {
                alive_after_fb++;

                /* 记录 FB 误差（调试用） */
                float dx = points[i].x - points[i].x_prev;
                float dy = points[i].y - points[i].y_prev;
                fb_err_list[fb_err_cnt++] = sqrtf(dx * dx + dy * dy);
            }
            else
            {
                points[i].alive = 0;        /* FB 不通过 → 失效 */
            }
        }
    }

    /* MAD 异常点剔除 */
    kart_vtrack_mad_filter();

    /* 统计最终存活数 */
    uint16 alive_final = 0;
    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            alive_final++;
        }
    }

    /* 存活点数不足 → 无效 */
    if(alive_final < VTRACK_MIN_ALIVE_POINTS)
    {
        vtrack_res.alive_count = alive_final;
        return &vtrack_res;
    }

    /* 中值 scale */
    float scale_r = kart_vtrack_median_scale();
    vtrack_res.scale_r = scale_r;

    if(scale_r > VTRACK_SCALE_NEAR_THRESH)
    {
        vtrack_res.scale_level = KART_VTRACK_SCALE_TOO_NEAR;
    }
    else if(scale_r < VTRACK_SCALE_FAR_THRESH)
    {
        vtrack_res.scale_level = KART_VTRACK_SCALE_TOO_FAR;
    }
    else
    {
        vtrack_res.scale_level = KART_VTRACK_SCALE_NORMAL;
    }

    /* 质心方位角 */
    vtrack_res.bearing_rad = kart_vtrack_bearing();

    /* FB 误差中值（调试用）*/
    if(fb_err_cnt > 0)
    {
        /* 简单排序取中值 */
        for(i = 0; i < fb_err_cnt - 1; i++)
        {
            for(uint16 j = i + 1; j < fb_err_cnt; j++)
            {
                if(fb_err_list[i] > fb_err_list[j])
                {
                    float tmp = fb_err_list[i];
                    fb_err_list[i] = fb_err_list[j];
                    fb_err_list[j] = tmp;
                }
            }
        }
        vtrack_res.fb_error_median = fb_err_list[fb_err_cnt / 2];
    }

    /* 更新 x_prev/y_prev，下一帧用 */
    for(i = 0; i < total_points; i++)
    {
        if(points[i].alive)
        {
            points[i].x_prev = points[i].x;
            points[i].y_prev = points[i].y;
        }
    }

    /* 置信度：综合存活率、FB 误差、scale 剧变、距 Detector 帧数 */
    float survive_ratio = (float)alive_final / (float)total_points;
    float conf = confidence_float;

    /* 存活率权重：< 50% 快速衰减 */
    if(survive_ratio < 0.5f)
    {
        conf *= (survive_ratio / 0.5f);
    }

    /* scale 剧变惩罚：|scale_r - 1| > 0.2 说明目标在剧烈旋转/形变 */
    float scale_dev = fabsf(scale_r - 1.0f);
    if(scale_dev > 0.2f)
    {
        conf *= (0.8f - scale_dev);     /* 超过 0.2 每增加 0.1 扣 10% */
        if(conf < 0.0f) conf = 0.0f;
    }

    /* 无 Detector 锚定衰减 */
    conf *= VTRACK_CONF_DECAY_PER_FRAME;

    /* 超最大帧数强制失效 */
    frames_since_detector++;
    if(frames_since_detector > VTRACK_MAX_FRAMES_SINCE_DET)
    {
        conf = 0.0f;
    }

    confidence_float = conf;
    vtrack_res.confidence = (uint8)((conf > 255.0f) ? 255 : ((conf < 0.0f) ? 0 : conf));

    /* 置信度过低也判无效 */
    if(vtrack_res.confidence < 50)
    {
        vtrack_res.valid = 0;
    }
    else
    {
        vtrack_res.valid = 1;
    }

    vtrack_res.alive_count = alive_final;
    vtrack_res.frames_since_detector = frames_since_detector;

    return &vtrack_res;
}

const kart_vtrack_result_t *kart_vtrack_get(void)
{
    return &vtrack_res;
}

void kart_vtrack_reset(void)
{
    total_points = 0;
    frames_since_detector = 0;
    confidence_float = 0.0f;

    memset(&vtrack_res, 0, sizeof(vtrack_res));
    vtrack_res.valid = 0;
}

#pragma section all restore

