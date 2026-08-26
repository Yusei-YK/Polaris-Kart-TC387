#include "kart_flash.h"
#include "zf_driver_flash.h"
#include <string.h>

/*
 * 路径 Flash 存储层实现 —— 见 kart_flash.h 头注释。
 * ------------------------------------------------------------------
 * 逻辑存储流(以 uint32 为单元,顺序铺开在槽位的连续页里):
 *   word[0]      = MAGIC          (校验标识,v2='KPT2')
 *   word[1]      = count          (路径点数)
 *   word[2]      = origin_yaw     (录制起点世界航向,度)
 *   word[3]      = origin_x       (录制起点世界坐标,米)
 *   word[4]      = origin_y
 *   word[5..7]   = 保留(写 0)
 *   word[8..]    = 每点 7 个 word:x/y/yaw/v_left/v_right/kart_steer/dist
 *                  (float 按位存为 uint32;kart_steer 是 int16 打角,存时转 float)
 * 一槽位跨 S1/S2/S2R_PAGES_PER_SLOT 页(21/14/7),每页 EEPROM_PAGE_LENGTH(512) 个 word。
 * 存时只写用到的页(ceil(总字数/512)),读时以页头 count 为准,尾部残留不影响。
 * ------------------------------------------------------------------
 */

/* 根据槽位号计算Flash页起始地址
 * 三段式:槽0 科目一(21页) | 槽1~5 科目二去程(14页/槽) | 槽6~10 科目二返程(7页/槽)。
 * 三段的页宽不同,所以只能逐段算,不能用统一的 slot*PAGES 公式。 */
static uint32 slot_base_page(uint8 slot)
{
    if(slot == 0)
    {
        /* 槽0: 科目一,页0~20 */
        return KART_FLASH_BASE_PAGE;
    }
    else if(slot < FLASH_S2R_FIRST_SLOT)
    {
        /* 槽1~5: 科目二门洞去程,页21开始,每槽14页(到页90) */
        return KART_FLASH_BASE_PAGE + KART_FLASH_S1_PAGES_PER_SLOT + (slot - 1) * KART_FLASH_S2_PAGES_PER_SLOT;
    }
    else if(slot < KART_FLASH_SLOT_NUM)
    {
        /* 槽6~10: 科目二门洞返程,页91开始,每槽7页(到页125) */
        return FLASH_S2R_BASE_PAGE
             + (uint32)(slot - FLASH_S2R_FIRST_SLOT) * FLASH_S2R_PAGES_PER_SLOT;
    }
    /* 【已知缺口】越界槽号在这里落到页 0,而页 0 正是科目一路径的首页。
     * 配合 slot_capacity() 同样返回 0(→ save 把 count 截成 0),一次越界的 save 会往
     * 页 0 写下 magic + count=0,科目一路径当场判空,必须重录。
     * 现在不会发生:save/load/slot_count 三个入口都先查 slot >= KART_FLASH_SLOT_NUM
     * 再往下走,静态槽号也都来自菜单的固定范围。所以这是潜在坑,不是现行故障。
     * 要补就把返回值改成一个不可能的页号(如 0xFFFFFFFF)并在调用处判。 */
    return 0;
}

/* 根据槽位号获取该槽最大容量(点数)。
 * save 用它做截断、load/slot_count 用它判页头 count 是否可信 —— 返程槽
 * 容量小(510),这里漏加分支就会把 510 以上的残留 count 当成合法值读越页。 */
static uint16 slot_capacity(uint8 slot)
{
    if(slot == 0)
        return KART_FLASH_S1_SLOT_CAPACITY;
    else if(slot < FLASH_S2R_FIRST_SLOT)
        return KART_FLASH_S2_SLOT_CAPACITY;
    else if(slot < KART_FLASH_SLOT_NUM)
        return FLASH_S2R_SLOT_CAPACITY;
    return 0;
}

/* float <-> uint32 按位互转(存进 Flash 的是 32bit 原始位模式)。 */
static uint32 f2u(float f)  { flash_data_union u; u.float_type  = f; return u.uint32_type; }
static float  u2f(uint32 v) { flash_data_union u; u.uint32_type = v; return u.float_type;  }

/* 组装/解析用的单页缓冲(512 word)。放静态区,别占栈。 */
static uint32 page_buf[EEPROM_PAGE_LENGTH];

/* 存盘时的一次性上下文。stream_word 是被逐字调用的(每页 512 次),
 * 参数表拉长成 6 个反而更容易漏传,集中放这里由 save 入口填一次。 */
static const kart_waypoint_t  *sw_wp;
static const int16            *sw_steer;
static const float            *sw_dist;
static const kart_flash_meta_t*sw_meta;
static uint16                  sw_count;

/* 取逻辑流第 idx 个 word(前 FLASH_HDR_WORDS 个是页头,其余为点数据)。 */
static uint32 stream_word(uint32 idx)
{
    if(idx < FLASH_HDR_WORDS)
    {
        switch(idx)
        {
            case 0: return KART_FLASH_MAGIC;
            case 1: return (uint32)sw_count;
            case 2: return sw_meta ? f2u(sw_meta->origin_yaw) : 0u;
            case 3: return sw_meta ? f2u(sw_meta->origin_x)   : 0u;
            case 4: return sw_meta ? f2u(sw_meta->origin_y)   : 0u;
            default:return 0u;           /* 保留字 */
        }
    }

    uint32 di = idx - FLASH_HDR_WORDS;      /* 数据区偏移 */
    uint32 p  = di / FLASH_FIELDS_PER_PT;   /* 第几个点 */
    uint32 f  = di % FLASH_FIELDS_PER_PT;   /* 点内第几个字段 */
    if(p >= sw_count) return 0;

    switch(f)
    {
        case 0: return f2u(sw_wp[p].x);
        case 1: return f2u(sw_wp[p].y);
        case 2: return f2u(sw_wp[p].yaw);
        case 3: return f2u(sw_wp[p].v_left);
        case 4: return f2u(sw_wp[p].v_right);
        /* kart_steer 是 int16,统一按 float 存:全流一个字宽,读回不必分类型解析。
         * 存进来的是 kart_steer_abs_get_center_delta()(见 kart_record.c:182),
         * 也就是相对中位的计数差:软限位 ±1064、机械行程 ±1084(kart_calib.h:116/108)。
         * float 能精确表示到 2^24 的整数,整个 int16 值域都无精度损失,读回 (int16)
         * 强转拿到的就是原值。 */
        case 5: return sw_steer ? f2u((float)sw_steer[p]) : 0u;
        default:return sw_dist  ? f2u(sw_dist[p])         : 0u;
    }
}

uint8 kart_flash_save_path(uint8 slot, const kart_waypoint_t *wp,
                           const int16 *kart_steer, const float *dist,
                           const kart_flash_meta_t *meta, uint16 count)
{
    if(slot >= KART_FLASH_SLOT_NUM)               return 1;
    if(count == 0 || wp == NULL)                  return 2;

    uint16 max_cap = slot_capacity(slot);
    if(count > max_cap)
        count = max_cap;                          /* 超容量截断,保证不越页 */

    sw_wp = wp; sw_steer = kart_steer; sw_dist = dist; sw_meta = meta; sw_count = count;

    uint32 total_words = (uint32)FLASH_HDR_WORDS
                       + (uint32)count * FLASH_FIELDS_PER_PT;
    uint32 need_pages  = (total_words + EEPROM_PAGE_LENGTH - 1) / EEPROM_PAGE_LENGTH;
    uint32 base_page   = slot_base_page(slot);

    uint32 pi, k, widx;
    for(pi = 0; pi < need_pages; pi++)
    {
        uint32 base_word = pi * EEPROM_PAGE_LENGTH;
        for(k = 0; k < EEPROM_PAGE_LENGTH; k++)
        {
            widx = base_word + k;
            page_buf[k] = (widx < total_words) ? stream_word(widx) : 0u;
        }
        /* 阻塞擦写一页(内部自动擦除脏页)。只在停车静止时调。
         * 【注意】v2 一槽最多 21 页,整段 save 比 v1 长约 40%,更不能在行驶中调。 */
        flash_write_page(0, base_page + pi, page_buf, EEPROM_PAGE_LENGTH);
    }
    /* 【返回 0 的含义】只表示"参数合法、该写的页都走了一遍",不表示写进去了。
     * 逐飞的 flash_write_page() 返回 void(zf_driver_flash.h),擦写结果拿不到 ——
     * 库里带状态的是 flash_write_page_from_buffer(),但它一次只能写 buffer 那一页,
     * 换过去要重排整个组页流程。
     * 现场判据只能靠回读:存完在菜单里看槽位点数(kart_flash_slot_count),
     * 点数对得上才算存住了。 */
    return 0;
}

uint16 kart_flash_load_path(uint8 slot, kart_waypoint_t *wp_out,
                            int16 *steer_out, float *dist_out,
                            kart_flash_meta_t *meta_out, uint16 max_count)
{
    if(slot >= KART_FLASH_SLOT_NUM || wp_out == NULL) return 0;

    uint32 base_page = slot_base_page(slot);

    /* 先读首页,校验 magic + 取 count + 页头 meta。 */
    flash_read_page(0, base_page, page_buf, EEPROM_PAGE_LENGTH);
    if(page_buf[0] != KART_FLASH_MAGIC) return 0;      /* 无效/空槽位/v1 老布局 */

    uint32 count = page_buf[1];
    uint16 max_cap = slot_capacity(slot);
    if(count == 0 || count > max_cap) return 0;
    if(count > max_count) count = max_count;

    /* meta 必须在逐点循环【之前】取:循环会把 page_buf 换成后面的页。 */
    if(meta_out != NULL)
    {
        meta_out->origin_yaw = u2f(page_buf[2]);
        meta_out->origin_x   = u2f(page_buf[3]);
        meta_out->origin_y   = u2f(page_buf[4]);
    }

    /* 首页里已含页头 + 前若干点。逐点从逻辑流取,跨页时按需重读。 */
    uint32 cur_page = 0;                               /* 当前 page_buf 装的是槽位第几页 */
    uint32 p, f;
    float  fld[FLASH_FIELDS_PER_PT];
    for(p = 0; p < count; p++)
    {
        for(f = 0; f < FLASH_FIELDS_PER_PT; f++)
        {
            uint32 widx = (uint32)FLASH_HDR_WORDS
                        + p * FLASH_FIELDS_PER_PT + f;
            uint32 pg   = widx / EEPROM_PAGE_LENGTH;
            uint32 off  = widx % EEPROM_PAGE_LENGTH;
            if(pg != cur_page)
            {
                flash_read_page(0, base_page + pg, page_buf, EEPROM_PAGE_LENGTH);
                cur_page = pg;
            }
            fld[f] = u2f(page_buf[off]);
        }
        wp_out[p].x       = fld[0];
        wp_out[p].y       = fld[1];
        wp_out[p].yaw     = fld[2];
        wp_out[p].v_left  = fld[3];
        wp_out[p].v_right = fld[4];
        if(steer_out != NULL) steer_out[p] = (int16)fld[5];
        if(dist_out  != NULL) dist_out[p]  = fld[6];
    }
    return (uint16)count;
}

uint16 kart_flash_slot_count(uint8 slot)
{
    if(slot >= KART_FLASH_SLOT_NUM) return 0;

    uint32 base_page = slot_base_page(slot);
    flash_read_page(0, base_page, page_buf, EEPROM_PAGE_LENGTH);
    if(page_buf[0] != KART_FLASH_MAGIC) return 0;

    uint32 count = page_buf[1];
    uint16 max_cap = slot_capacity(slot);
    if(count == 0 || count > max_cap) return 0;
    return (uint16)count;
}
