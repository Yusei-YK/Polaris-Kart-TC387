#include "kart_traj_view.h"

#if TRAJ_VIEW_ENABLE
#include "zf_device_ips200.h"
#include "kart_record.h"
#include "kart_playback.h"

/* -------------------- 版面 --------------------
 * 240x320 竖屏。上面留三行文字(标题/统计/包围盒),底部留一行按键提示,
 * 中间整块给绘图区。数值与 kart_menu.c 的 UI_* 对齐,但这里刻意不去
 * kart_include kart_menu.h —— 那些宏是 kart_menu.c 的内部定义,拿不到,
 * 故本文件自带一份,改版面只改这里。 */
#define TV_COLS         (30)            /* 8x16 字体,240px 宽 = 30 字 */
#define TV_Y_TITLE      (0)
#define TV_Y_INFO1      (18)            /* 点数 / 总里程 */
#define TV_Y_INFO2      (36)            /* 包围盒尺寸 / 比例 */
#define TV_Y_HINT       (302)           /* 底部提示(最后一行,16px 高,318<320) */

#define TV_PLOT_X0      (4)             /* 绘图区左上 */
#define TV_PLOT_Y0      (56)
#define TV_PLOT_W       (232)
#define TV_PLOT_H       (240)           /* 56+240=296,离提示行还有 6px */

/* 日光可读白蓝配色，与 kart_menu.c 保持一致。 */
#define TV_BG           (0xFFFF)
#define TV_FG           (0x194D)
#define TV_BAR_BG       (0x03D9)
#define TV_BAR_FG       (0xFFFF)
#define TV_NUM          (0x0471)
#define TV_DIM          (0x7BEF)
#define TV_WARN         (0xC3E0)
#define TV_FWD          (0x03D9)        /* 前进段:主蓝 */
#define TV_REV          (0xD9E7)        /* 倒车段:红，保留方向警示语义 */
#define TV_SAMPLE       (0x012E)        /* 实际采样点:深蓝 */
#define TV_CURSOR       (0xFFE0)        /* 当前修正位置:亮黄 */
#define TV_GRID         (0xCE7F)        /* 边框:浅蓝 */

/* 最小包围盒边长(米)。路径几乎是原地打转时包围盒接近 0,
 * 直接拿去算比例会除出天文数字/inf。兜到 0.5m。 */
#define TV_SPAN_MIN     (0.5f)

static uint16 tv_cursor = 0;
static uint8  tv_editing = 0;
static uint8  tv_dirty = 0;

void kart_traj_view_set_cursor(uint16 index, uint8 editing, uint8 dirty)
{
    tv_cursor = index;
    tv_editing = editing ? 1U : 0U;
    tv_dirty = dirty ? 1U : 0U;
}

/* -------------------- 定宽文本 --------------------
 * 与 kart_menu.c 同一条规矩:补空格到 30 字,右侧残留一并盖掉。
 * 本页每次都是清屏后重画,理论上不补也行,但保持一致,将来改成页内刷新不踩坑。 */
static void tv_bar(uint16 y, const char *s, uint16 fg, uint16 bg)
{
    char line[TV_COLS + 1];
    uint8 i = 0;

    while(i < TV_COLS && s[i] != '\0') { line[i] = s[i]; i++; }
    while(i < TV_COLS)                 { line[i] = ' ';  i++; }
    line[TV_COLS] = '\0';

    ips200_set_color(fg, bg);
    ips200_show_string(0, y, line);
}

/* 绘图区边框。四条边用 draw_line 一次画完(横线每像素一次开窗,232 点约 3ms,
 * 只画一次可以接受)。有框才看得出"图画满了没有"。 */
static void tv_frame(void)
{
    uint16 x0 = TV_PLOT_X0, y0 = TV_PLOT_Y0;
    uint16 x1 = (uint16)(TV_PLOT_X0 + TV_PLOT_W - 1);
    uint16 y1 = (uint16)(TV_PLOT_Y0 + TV_PLOT_H - 1);

    ips200_draw_line(x0, y0, x1, y0, TV_GRID);
    ips200_draw_line(x0, y1, x1, y1, TV_GRID);
    ips200_draw_line(x0, y0, x0, y1, TV_GRID);
    ips200_draw_line(x1, y0, x1, y1, TV_GRID);
}

/* 实心小方块(标起点/终点)。半径 r,自带钳位:draw_point 里有 zf_assert,
 * 越界会直接把程序打进断言死循环,所以这里宁可少画几个像素。 */
static void tv_mark(uint16 cx, uint16 cy, uint8 r, uint16 color)
{
    int32 x, y;

    for(y = (int32)cy - r; y <= (int32)cy + r; y++)
    {
        if(y < 0 || y >= (int32)ips200_height_max) continue;
        for(x = (int32)cx - r; x <= (int32)cx + r; x++)
        {
            if(x < 0 || x >= (int32)ips200_width_max) continue;
            ips200_draw_point((uint16)x, (uint16)y, color);
        }
    }
}

/* -------------------- 数字转字符串(不用 %f)--------------------
 * 【为什么不直接 sprintf("%.2f")】两条:
 *   1) 长度不可控。里程计跑飞(编码器掉线、IMU 发疯)时 x/y 可能是 1e30 甚至
 *      inf,"%.1f" 会吐出三百多位,把栈上的 buf 冲掉 —— 栈溢出比屏上数字错
 *      难查得多。gcc 开 -Wformat-overflow=2 就直接报这条。
 *   2) TASKING 的浮点 printf 代码体积大、还要拖进 double 运算。
 * 改成先钳到 ±9999、再用整数拆出整数位和小数位,输出长度封顶 8 字("-9999.99"),
 * 长度可证、没有浮点格式化。
 * 钳过的值屏上显示成 9999,一眼能看出是数据坏了,不会误认成路径真有那么大。 */
#define TV_NUM_LEN      (12)        /* "-9999.99" 8 字 + 结尾,取整留余量 */

static void tv_fmt(float v, uint8 dec, char *out)
{
    int32 scale = 1, whole, frac;
    uint8 i, p = 0;
    uint8 neg = 0;

    if(!(v == v)) v = 0.0f;                 /* nan:和自己都不相等,得单独判 */
    if(v >  9999.0f) v =  9999.0f;          /* 顺带吃掉 +inf */
    if(v < -9999.0f) v = -9999.0f;

    if(v < 0.0f) { neg = 1; v = -v; }
    for(i = 0; i < dec; i++) scale *= 10;

    /* +0.5 做四舍五入。v 已钳在 9999 内,乘 100 最大 999950,int32 装得下。 */
    whole = (int32)v;
    frac  = (int32)((v - (float)whole) * (float)scale + 0.5f);
    if(frac >= scale) { frac -= scale; whole += 1; }     /* 0.999→进位 */

    if(neg) out[p++] = '-';

    /* 整数部分:最多 4 位,从高位往低位写,前导零跳过。 */
    {
        int32 div = 1000;
        uint8 started = 0;
        while(div > 0)
        {
            int32 d = (whole / div) % 10;
            if(d != 0 || started || div == 1) { out[p++] = (char)('0' + d); started = 1; }
            div /= 10;
        }
    }

    if(dec > 0)
    {
        out[p++] = '.';
        scale /= 10;
        while(scale > 0)
        {
            out[p++] = (char)('0' + (frac / scale) % 10);
            scale /= 10;
        }
    }

    out[p] = '\0';
}

/* -------------------- 米 → 像素 --------------------
 * 一次算好比例和偏移,后面每个点只做一次乘加。
 *   sx = (x - cx) * k + 屏心x        （x 向右,与屏一致,不取反）
 *   sy = 屏心y - (y - cy) * k        （y 向前,屏 y 朝下,故取负）
 * 等比例:x/y 用同一个 k,不然弯道会被拉成椭圆,看图判不出实际半径。 */
typedef struct
{
    float  k;           /* 像素/米 */
    float  cx, cy;      /* 包围盒中心(米) */
    uint16 ox, oy;      /* 绘图区中心(像素) */
} tv_map_t;

/* 把米坐标映射成屏坐标,并钳在绘图区内。
 * 【为什么一定要钳】ips200_draw_point/draw_line 内部是 zf_assert 检边界,
 * 越界不是"画不出来"而是断言卡死。等比例缩放理论上不会越界,但浮点取整
 * 差一个像素、或者将来有人改了 SPAN_MIN 就可能擦到边,这里兜住。
 * 返回 1=点原本就在框内,0=被钳过(调用方可据此决定要不要连线,避免
 * 被钳到边框上的假线段;当前实现照连,因为等比例下不会真的发生)。 */
static uint8 tv_project(const tv_map_t *m, float x, float y, uint16 *px, uint16 *py)
{
    int32 sx, sy;
    int32 xlo = TV_PLOT_X0 + 1,  xhi = TV_PLOT_X0 + TV_PLOT_W - 2;
    int32 ylo = TV_PLOT_Y0 + 1,  yhi = TV_PLOT_Y0 + TV_PLOT_H - 2;
    uint8 inside = 1;
    float fx = (x - m->cx) * m->k;
    float fy = (y - m->cy) * m->k;

    /* 先在浮点域挡掉 nan/超范围:float→int32 转换在值超出 int32 或为 nan 时是
     * 未定义行为(TriCore 上会给出随机值),不能指望后面的整数钳位兜住。 */
    if(!(fx == fx)) fx = 0.0f;
    if(!(fy == fy)) fy = 0.0f;
    if(fx >  1.0e6f) fx =  1.0e6f;
    if(fx < -1.0e6f) fx = -1.0e6f;
    if(fy >  1.0e6f) fy =  1.0e6f;
    if(fy < -1.0e6f) fy = -1.0e6f;

    sx = (int32)fx + (int32)m->ox;
    sy = (int32)m->oy - (int32)fy;

    if(sx < xlo) { sx = xlo; inside = 0; }
    if(sx > xhi) { sx = xhi; inside = 0; }
    if(sy < ylo) { sy = ylo; inside = 0; }
    if(sy > yhi) { sy = yhi; inside = 0; }

    *px = (uint16)sx;
    *py = (uint16)sy;
    return inside;
}

void kart_traj_view_draw(void)
{
    const kart_waypoint_t *wp = kart_record_get_waypoints();
    uint16 n = kart_record_get_count();
    /* buf 比一行宽:sprintf 先写满 buf,再由 tv_bar 截到 30 字。
     * 数字都经 tv_fmt 封顶 8 字(见那边说明),故最长输出可证在 48 内。 */
    char buf[64];
    char na[TV_NUM_LEN], nb[TV_NUM_LEN], nc[TV_NUM_LEN];
    float xmin, xmax, ymin, ymax, spanx, spany, kx, ky;
    tv_map_t map;
    uint16 i, px, py, ppx = 0, ppy = 0;

    ips200_set_color(TV_BAR_FG, TV_BAR_BG);
    tv_bar(TV_Y_TITLE, " S1 Path", TV_BAR_FG, TV_BAR_BG);

    if(n < 2 || wp == NULL)
    {
        /* 菜单进页前已把科目一 Flash 槽 0 读回 RAM。此时仍不足
         * 2 点，表示槽位为空、校验失败，或是不兼容的旧版存储格式。 */
        tv_bar(TV_Y_INFO1, " No path in Flash",        TV_WARN, TV_BG);
        tv_bar(TV_Y_INFO2, " Record and save Slot1",  TV_DIM,  TV_BG);
        tv_bar(TV_Y_HINT,  " KART_LEFT back",              TV_DIM,  TV_BG);
        return;
    }

    /* 包围盒。1500 点扫一遍纯 RAM 访问,几十 us,比后面画线便宜得多。 */
    xmin = xmax = wp[0].x;
    ymin = ymax = wp[0].y;
    for(i = 1; i < n; i++)
    {
        if(wp[i].x < xmin) xmin = wp[i].x;
        if(wp[i].x > xmax) xmax = wp[i].x;
        if(wp[i].y < ymin) ymin = wp[i].y;
        if(wp[i].y > ymax) ymax = wp[i].y;
    }

    spanx = xmax - xmin;
    spany = ymax - ymin;
    if(spanx < TV_SPAN_MIN) spanx = TV_SPAN_MIN;    /* 防除零 / 比例爆表 */
    if(spany < TV_SPAN_MIN) spany = TV_SPAN_MIN;

    /* 等比例取小的那个,保证长边正好铺满、短边居中。留 4px 边距不贴框。 */
    kx = (float)(TV_PLOT_W - 8) / spanx;
    ky = (float)(TV_PLOT_H - 8) / spany;
    map.k  = (kx < ky) ? kx : ky;
    map.cx = 0.5f * (xmin + xmax);
    map.cy = 0.5f * (ymin + ymax);
    map.ox = (uint16)(TV_PLOT_X0 + TV_PLOT_W / 2);
    map.oy = (uint16)(TV_PLOT_Y0 + TV_PLOT_H / 2);

    /* 数字全走 tv_fmt(见上方说明:不用 %f,长度可证)。n 是 uint16,但录制上限是
     * KART_RECORD_MAX_WAYPOINTS=1500,所以点数和光标序号各最多 4 位。
     * 按当前两个格式串算最宽:
     *   第一行 " %u pts P%u %s%s%s" → 点数+光标+EDIT+*+LOCK 全满时 26 字;
     *   第二行 " box %sx%sm  %sp/m" → 两个 9999.9 加一个 9999 时 28 字。
     * 都在 30 字之内,tv_bar 不会截掉有效信息。
     * 【下面这行算了总里程却没用】tv_fmt(...total_dist..., na) 之后的
     * sprintf 并不引用 na,na 到第二行又被包围盒宽度覆盖 —— 屏上看不到
     * 总里程。留着不删是因为要加回去只需在第一行插一个 %s。 */
    if(tv_cursor >= n) tv_cursor = (uint16)(n - 1U);
    tv_fmt(kart_record_get_total_dist(), 2, na);
    sprintf(buf, " %u pts P%u %s%s%s", (unsigned)n, (unsigned)tv_cursor,
            tv_editing ? "EDIT" : "VIEW", tv_dirty ? "*" : "",
            (0.5f * (wp[tv_cursor].v_left + wp[tv_cursor].v_right)
             < -KART_PLAYBACK_REV_SPEED_EPS) ? " LOCK" : "");
    tv_bar(TV_Y_INFO1, buf, TV_NUM, TV_BG);

    tv_fmt(xmax - xmin, 1, na);
    tv_fmt(ymax - ymin, 1, nb);
    tv_fmt(map.k,       0, nc);
    sprintf(buf, " box %sx%sm  %sp/m", na, nb, nc);
    tv_bar(TV_Y_INFO2, buf, TV_DIM, TV_BG);

    tv_frame();

    /* 逐段连线。段色按录制速度符号分前进/倒车 —— 判据与 kart_playback.c 的
     * 倒车段判定同源(rec_v 与 ±REV_SPEED_EPS 比),这样屏上红色段就等于
     * 复现时会走开环回放打角的那一段,对着看能直接判断倒库段录得对不对。
     * 用左右轮均值:单轮在原地转向时可能一正一负,均值才代表车体前后。 */
    for(i = 0; i < n; i++)
    {
        uint16 color;
        float  v;

        tv_project(&map, wp[i].x, wp[i].y, &px, &py);

        if(i > 0)
        {
            v = 0.5f * (wp[i].v_left + wp[i].v_right);
            color = (v < -KART_PLAYBACK_REV_SPEED_EPS) ? TV_REV : TV_FWD;
            ips200_draw_line(ppx, ppy, px, py, color);
        }

        ppx = px;
        ppy = py;
    }

    /* 把每个真实采样点再压一颗深蓝像素，既能看路径形状，也能直观看到
     * 直线稀疏、弯道密集的自适应采样是否正常。最多 1500 点，只在换页画一次。 */
    for(i = 0; i < n; i++)
    {
        tv_project(&map, wp[i].x, wp[i].y, &px, &py);
        ips200_draw_point(px, py, TV_SAMPLE);
    }

    /* 起点/终点方块最后画,盖在线和采样点上面,不会被路径压住。 */
    tv_project(&map, wp[0].x, wp[0].y, &px, &py);
    tv_mark(px, py, 3, TV_FWD);
    tv_project(&map, wp[n - 1].x, wp[n - 1].y, &px, &py);
    tv_mark(px, py, 3, TV_REV);

    /* 当前点最后画，黄色方块覆盖路径，调整后位置一眼可见。 */
    tv_project(&map, wp[tv_cursor].x, wp[tv_cursor].y, &px, &py);
    tv_mark(px, py, 2, TV_CURSOR);

    if(tv_editing)
        tv_bar(TV_Y_HINT, " arrows move  MID done", TV_DIM, TV_BG);
    else
        tv_bar(TV_Y_HINT, "UP/DN pick MID edit START save", TV_DIM, TV_BG);
}

#endif

