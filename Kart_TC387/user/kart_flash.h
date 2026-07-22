#ifndef KART_FLASH_H_
#define KART_FLASH_H_

#include "zf_common_headfile.h"
#include "kart_record.h"        /* kart_waypoint_t */

/*
 * 路径 Flash 存储层（DFlash 槽位持久化）
 * ------------------------------------------------------------------
 * 把 kart_record 的 RAM 路径点数组存进 TC387 片内 DFlash,断电保留;
 * 上电用 kart_flash_load_path() 读回给 kart_playback 复现。
 *
 * 【为什么要它】比赛现场无上位机,发车排队 + 天气原因需一次性把三科目
 *   路径都录好存 Flash,现场从菜单选槽位加载复现。
 *
 * 【槽位规划】
 *   槽位 0:      科目一三合一路径(1500点,15页,页0~14)
 *   槽位 1~5:    科目二门洞路径(每个1000点,10页/槽,页15~64)
 *   总计占用:    65页 < 128页,剩余63页
 *
 * 【容量】DFlash 128 页 × 2KB(512 uint32/页)。1 路径点=5 float=5 uint32。
 *   科目一槽: 15 页 = 7680 uint32,减 2 字头 = 7678/5 ≈ 1535 点,>1500 上限,够用。
 *   科目二槽: 10 页 = 5120 uint32,减 2 字头 = 5118/5 ≈ 1023 点,>1000 上限,够用。
 *
 * !!! 使用纪律 !!!
 *   flash_write_page 是阻塞擦写(每页逐字循环,耗时数 ms),
 *   只能在"停车静止"时调 save,严禁在行驶/中断里调,否则卡死主循环+烧写风险。
 *   load 是纯读(直接映射地址),快,可在准备阶段调。
 * ------------------------------------------------------------------
 */

/* -------------------- 槽位/页规划 -------------------- */
#define KART_FLASH_MAGIC            (0x4B505448u)   /* 'KPTH' 路径页标识,读回校验 */
#define KART_FLASH_BASE_PAGE        (0)             /* 路径存储起始页 */

/* 科目一槽位(槽0) */
#define KART_FLASH_S1_SLOT_NUM      (1)             /* 科目一槽位数 */
#define KART_FLASH_S1_PAGES_PER_SLOT (15)           /* 科目一每槽页数(15*512-2 = 7678 word ≈ 1535 点) */
#define KART_FLASH_S1_SLOT_CAPACITY  ((KART_FLASH_S1_PAGES_PER_SLOT * EEPROM_PAGE_LENGTH - 2) / 5)

/* 科目二槽位(槽1~5) */
#define KART_FLASH_S2_SLOT_NUM      (5)             /* 科目二槽位数(5个门洞) */
#define KART_FLASH_S2_PAGES_PER_SLOT (10)           /* 科目二每槽页数(10*512-2 = 5118 word ≈ 1023 点) */
#define KART_FLASH_S2_SLOT_CAPACITY  ((KART_FLASH_S2_PAGES_PER_SLOT * EEPROM_PAGE_LENGTH - 2) / 5)

/* 总槽位数 */
#define KART_FLASH_SLOT_NUM         (KART_FLASH_S1_SLOT_NUM + KART_FLASH_S2_SLOT_NUM)

/* 兼容旧代码:科目一使用槽0,按15页容量 */
#define KART_FLASH_PAGES_PER_SLOT   KART_FLASH_S1_PAGES_PER_SLOT
#define KART_FLASH_SLOT_CAPACITY    KART_FLASH_S1_SLOT_CAPACITY
#if (KART_FLASH_SLOT_CAPACITY >= KART_RECORD_MAX_WAYPOINTS)
    #define KART_FLASH_MAX_WAYPOINTS    (KART_RECORD_MAX_WAYPOINTS)
#else
    #define KART_FLASH_MAX_WAYPOINTS    (KART_FLASH_SLOT_CAPACITY)
#endif

/* -------------------- 对外接口 -------------------- */

/* 把 count 个路径点写入 slot 槽位(阻塞擦写,只能停车时调)。
 * 返回 0=成功,1=slot 越界,2=count 为 0/无效。 */
uint8  kart_flash_save_path(uint8 slot, const kart_waypoint_t *wp, uint16 count);

/* 从 slot 槽位读回路径点到 wp_out(最多 max_count 个)。
 * 返回实际读出点数;0 表示槽位空/magic 不符/slot 越界。 */
uint16 kart_flash_load_path(uint8 slot, kart_waypoint_t *wp_out, uint16 max_count);

/* 查询 slot 槽位是否存有有效路径(magic 匹配)。有=返回点数,无=0。
 * 只读页头 2 word,快;供菜单显示"槽位是否已录"用。 */
uint16 kart_flash_slot_count(uint8 slot);

#endif
