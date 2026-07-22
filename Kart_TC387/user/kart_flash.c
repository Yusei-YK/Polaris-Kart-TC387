#include "kart_flash.h"
#include "zf_driver_flash.h"
#include <string.h>

/*
 * 路径 Flash 存储层实现 —— 见 kart_flash.h 头注释。
 * ------------------------------------------------------------------
 * 逻辑存储流(以 uint32 为单元,顺序铺开在槽位的连续页里):
 *   word[0]      = MAGIC          (校验标识)
 *   word[1]      = count          (路径点数)
 *   word[2..]    = 每点 5 个 word:x/y/yaw/v_left/v_right(float 按位存为 uint32)
 * 一槽位跨 KART_FLASH_PAGES_PER_SLOT 页,每页 EEPROM_PAGE_LENGTH(512) 个 word。
 * 存时只写用到的页(ceil(总字数/512)),读时以页头 count 为准,尾部残留不影响。
 * ------------------------------------------------------------------
 */

/* 根据槽位号计算Flash页起始地址 */
static uint32 slot_base_page(uint8 slot)
{
    if(slot == 0)
    {
        /* 槽0: 科目一,页0~14 */
        return KART_FLASH_BASE_PAGE;
    }
    else if(slot >= 1 && slot <= 5)
    {
        /* 槽1~5: 科目二门洞,页15开始,每槽10页 */
        return KART_FLASH_BASE_PAGE + KART_FLASH_S1_PAGES_PER_SLOT + (slot - 1) * KART_FLASH_S2_PAGES_PER_SLOT;
    }
    return 0;
}

/* 根据槽位号获取该槽最大容量 */
static uint16 slot_capacity(uint8 slot)
{
    if(slot == 0)
        return KART_FLASH_S1_SLOT_CAPACITY;
    else if(slot >= 1 && slot <= 5)
        return KART_FLASH_S2_SLOT_CAPACITY;
    return 0;
}

/* 根据槽位号获取该槽页数 */
static uint8 slot_pages(uint8 slot)
{
    if(slot == 0)
        return KART_FLASH_S1_PAGES_PER_SLOT;
    else if(slot >= 1 && slot <= 5)
        return KART_FLASH_S2_PAGES_PER_SLOT;
    return 0;
}

/* float <-> uint32 按位互转(存进 Flash 的是 32bit 原始位模式)。 */
static uint32 f2u(float f)  { flash_data_union u; u.float_type  = f; return u.uint32_type; }
static float  u2f(uint32 v) { flash_data_union u; u.uint32_type = v; return u.float_type;  }

/* 组装/解析用的单页缓冲(512 word)。放静态区,别占栈。 */
static uint32 page_buf[EEPROM_PAGE_LENGTH];

/* 取逻辑流第 idx 个 word(idx=0 magic,1 count,其余为点数据)。 */
static uint32 stream_word(uint32 idx, const kart_waypoint_t *wp, uint16 count)
{
    if(idx == 0) return KART_FLASH_MAGIC;
    if(idx == 1) return (uint32)count;

    uint32 di = idx - 2;                 /* 数据区偏移 */
    uint32 p  = di / 5;                  /* 第几个点 */
    uint32 f  = di % 5;                  /* 点内第几个字段 */
    if(p >= count) return 0;

    switch(f)
    {
        case 0: return f2u(wp[p].x);
        case 1: return f2u(wp[p].y);
        case 2: return f2u(wp[p].yaw);
        case 3: return f2u(wp[p].v_left);
        default:return f2u(wp[p].v_right);
    }
}

uint8 kart_flash_save_path(uint8 slot, const kart_waypoint_t *wp, uint16 count)
{
    if(slot >= KART_FLASH_SLOT_NUM)               return 1;
    if(count == 0 || wp == NULL)                  return 2;

    uint16 max_cap = slot_capacity(slot);
    if(count > max_cap)
        count = max_cap;                          /* 超容量截断,保证不越页 */

    uint32 total_words = 2u + (uint32)count * 5u; /* 头 2 + 点数据 */
    uint32 need_pages  = (total_words + EEPROM_PAGE_LENGTH - 1) / EEPROM_PAGE_LENGTH;
    uint32 base_page   = slot_base_page(slot);

    uint32 pi, k, widx;
    for(pi = 0; pi < need_pages; pi++)
    {
        uint32 base_word = pi * EEPROM_PAGE_LENGTH;
        for(k = 0; k < EEPROM_PAGE_LENGTH; k++)
        {
            widx = base_word + k;
            page_buf[k] = (widx < total_words) ? stream_word(widx, wp, count) : 0u;
        }
        /* 阻塞擦写一页(内部自动擦除脏页)。只在停车静止时调。 */
        flash_write_page(0, base_page + pi, page_buf, EEPROM_PAGE_LENGTH);
    }
    return 0;
}

uint16 kart_flash_load_path(uint8 slot, kart_waypoint_t *wp_out, uint16 max_count)
{
    if(slot >= KART_FLASH_SLOT_NUM || wp_out == NULL) return 0;

    uint32 base_page = slot_base_page(slot);

    /* 先读首页,校验 magic + 取 count。 */
    flash_read_page(0, base_page, page_buf, EEPROM_PAGE_LENGTH);
    if(page_buf[0] != KART_FLASH_MAGIC) return 0;      /* 无效/空槽位 */

    uint32 count = page_buf[1];
    uint16 max_cap = slot_capacity(slot);
    if(count == 0 || count > max_cap) return 0;
    if(count > max_count) count = max_count;

    /* 首页里已含头 2 word + 前若干点。逐点从逻辑流取,跨页时按需重读。 */
    uint32 cur_page = 0;                               /* 当前 page_buf 装的是槽位第几页 */
    uint32 p, f;
    float  fld[5];
    for(p = 0; p < count; p++)
    {
        for(f = 0; f < 5; f++)
        {
            uint32 widx = 2u + p * 5u + f;
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
