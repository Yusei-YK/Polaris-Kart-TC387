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
 * 【2026-07-28 存储格式升级 v2:每点 5 字 → 7 字】
 *   起因(实车):科目一从槽位载入路径后倒车入库半径明显比手动大、倒不进库。
 *   原因:v1 只存 x/y/yaw/v_left/v_right 五个字段,而倒车段的打角是开环回放
 *   steer_buf[] 的,dist_buf[] 同理 —— 这两个数组过去是 RAM only。载入后它们
 *   还是上一次录制的残留(冷启动即全 0),playback 倒车段拿到 delta=0,
 *   整段打角只剩航向纠偏那一项,而纠偏被 REV_CORR_MAX=400 计数钳住 →
 *   R = 1480/400 ≈ 3.7m,而手动满锁 R=1.32m。这就是"半径很大"的来源。
 *   故 v2 把 steer/dist 一起入 Flash,并在页头存录制起点位姿(倒车纠偏的参考系)。
 *   MAGIC 同步升版,老槽位数据判为无效(布局不兼容,必须重录)。
 *
 * 【槽位规划】v2 每点变宽,页数同步放大,保证点数上限不缩水
 *   槽位 0:      科目一三合一路径(1500点,21页,页0~20)
 *   槽位 1~5:    科目二门洞【去程】发车区→门洞→集结点(每个1000点,14页/槽,页21~90)
 *   槽位 6~10:   科目二门洞【返程】集结点→门洞→发车区(2026-07-29 加,7页/槽,页91~125)
 *   总计占用:    126页 ≤ 126(页127 是参数表),剩余 1 页(页126)
 *
 * 【容量】DFlash 128 页 × 2KB(512 uint32/页)。1 路径点=7 uint32。
 *   科目一槽: 21 页 = 10752 uint32,减 8 字头 = 10744/7 = 1534 点,>1500 上限,够用。
 *   科目二槽: 14 页 = 7168  uint32,减 8 字头 = 7160/7  = 1022 点,>1000 上限,够用。
 *   返程槽:   7  页 = 3584  uint32,减 8 字头 = 3576/7  = 510 点。
 *     够不够:录制阈值 0.05m/点(KART_RECORD_DIST_THRESH),510 点 ≈ 25m 直线路程,
 *     弯多时点更密、里程更短,但"集结点→门洞→发车区"实测量级 10~15m,余量 ×1.7。
 *     只给 7 页是因为 DFlash 只剩 37 页,5 个槽必须均分且不能碰参数页。
 *     【真录满了怎么办】save 会截断到 510 点(kart_flash.c 有 slot_capacity 钳位),
 *     后半段路径直接没了 —— 现场判据是菜单里显示的点数,接近 510 就要缩短返程录制。
 *
 * 【为什么加返程槽不用升 MAGIC / 不用重录】
 *   槽 0~5 的起始页(slot_base_page)和每点字宽(7 字)一个字节都没动,
 *   新槽只是往后面的空白页里新建。老槽里的数据布局不变,MAGIC 仍是 'KPT2'。
 *
 * !!! 使用纪律 !!!
 *   flash_write_page 是阻塞擦写(每页逐字循环,耗时数 ms),
 *   只能在"停车静止"时调 save,严禁在行驶/中断里调,否则卡死主循环+烧写风险。
 *   load 是纯读(直接映射地址),快,可在准备阶段调。
 * ------------------------------------------------------------------
 */

/* -------------------- 槽位/页规划 -------------------- */
/* 'KPT2':v2 布局标识。【故意与 v1 的 'KPTH' 不同】—— 老槽位里是 5 字/点的流,
 * 按 7 字/点解出来是整段错位的垃圾数据,车会照着乱打方向。宁可判为空槽逼你重录。 */
#define KART_FLASH_MAGIC            (0x4B505432u)   /* 'KPT2' 路径页标识,读回校验 */
#define KART_FLASH_BASE_PAGE        (0)             /* 路径存储起始页 */

/* 页头字数。前 5 个用了:magic/count/origin_yaw/origin_x/origin_y,
 * 后 3 个留白 —— 下次要加全局量(如录制时的 PB 参数快照)不必再动布局/换 MAGIC。 */
#define KART_FLASH_HDR_WORDS        (8)
/* 每点字数:x/y/yaw/v_left/v_right/steer/dist。 */
#define KART_FLASH_FIELDS_PER_PT    (7)

/* 科目一槽位(槽0) */
#define KART_FLASH_S1_SLOT_NUM      (1)             /* 科目一槽位数 */
#define KART_FLASH_S1_PAGES_PER_SLOT (21)           /* 科目一每槽页数(21*512-8 = 10744 word = 1534 点) */
#define KART_FLASH_S1_SLOT_CAPACITY  ((KART_FLASH_S1_PAGES_PER_SLOT * EEPROM_PAGE_LENGTH - KART_FLASH_HDR_WORDS) / KART_FLASH_FIELDS_PER_PT)

/* 科目二去程槽位(槽1~5) */
#define KART_FLASH_S2_SLOT_NUM      (5)             /* 科目二去程槽位数(5个门洞) */
#define KART_FLASH_S2_PAGES_PER_SLOT (14)           /* 科目二每槽页数(14*512-8 = 7160 word = 1022 点) */
#define KART_FLASH_S2_SLOT_CAPACITY  ((KART_FLASH_S2_PAGES_PER_SLOT * EEPROM_PAGE_LENGTH - KART_FLASH_HDR_WORDS) / KART_FLASH_FIELDS_PER_PT)

/* 科目二返程槽位(槽6~10,2026-07-29 加,语音返回用)
 * 每槽只 7 页 = 510 点:DFlash 去掉槽0~5 的 91 页、参数页 127,只剩 36 页可分,
 * 5 个槽均分最多 7 页/槽(35 页,落 91~125)。返程只跑"集结点→门洞→发车区"
 * 一段(10~15m),0.05m/点约 200~300 点,余量 ×1.7,不必与去程等宽。 */
#define KART_FLASH_S2R_SLOT_NUM      (5)            /* 返程槽位数(与去程一一对应) */
#define KART_FLASH_S2R_PAGES_PER_SLOT (7)           /* 返程每槽页数(7*512-8 = 3576 word = 510 点) */
#define KART_FLASH_S2R_SLOT_CAPACITY  ((KART_FLASH_S2R_PAGES_PER_SLOT * EEPROM_PAGE_LENGTH - KART_FLASH_HDR_WORDS) / KART_FLASH_FIELDS_PER_PT)
/* 返程首槽(槽6)的起始页,= 槽0~5 用完之后的第一页 = 91。 */
#define KART_FLASH_S2R_BASE_PAGE     (KART_FLASH_BASE_PAGE + KART_FLASH_S1_PAGES_PER_SLOT + \
                                      KART_FLASH_S2_SLOT_NUM * KART_FLASH_S2_PAGES_PER_SLOT)
/* 槽号 → 返程槽的起始槽号(槽6)。slot >= 这个值就是返程槽。 */
#define KART_FLASH_S2R_FIRST_SLOT    (KART_FLASH_S1_SLOT_NUM + KART_FLASH_S2_SLOT_NUM)

/* 页数放大后必须复核不越界:最后一页要落在参数页(kart_params.h 的 127)之前,
 * 否则存路径会把整张参数表擦掉。
 * 0 + 21 + 5*14 + 5*7 = 126 页 → 用到页 0~125,页 126 空余,页 127 是参数表。 */
#if ((KART_FLASH_S2R_BASE_PAGE + \
      KART_FLASH_S2R_SLOT_NUM * KART_FLASH_S2R_PAGES_PER_SLOT) > 127)
    #error "kart_flash: path slots overlap the param page (127)"
#endif

/* 总槽位数(1 科目一 + 5 去程 + 5 返程 = 11) */
#define KART_FLASH_SLOT_NUM         (KART_FLASH_S1_SLOT_NUM + KART_FLASH_S2_SLOT_NUM + \
                                     KART_FLASH_S2R_SLOT_NUM)

/* 兼容旧代码:科目一使用槽0,按 S1 槽页数/容量(v2 = 21 页) */
#define KART_FLASH_PAGES_PER_SLOT   KART_FLASH_S1_PAGES_PER_SLOT
#define KART_FLASH_SLOT_CAPACITY    KART_FLASH_S1_SLOT_CAPACITY
#if (KART_FLASH_SLOT_CAPACITY >= KART_RECORD_MAX_WAYPOINTS)
    #define KART_FLASH_MAX_WAYPOINTS    (KART_RECORD_MAX_WAYPOINTS)
#else
    #define KART_FLASH_MAX_WAYPOINTS    (KART_FLASH_SLOT_CAPACITY)
#endif

/* -------------------- 对外接口 -------------------- */

/* 一次录制的"非路径点"全局量。存在页头,与点数组一起进出 Flash。
 * 为什么必须一起存:倒车段的航向纠偏参考系 = origin_yaw(录制起点世界航向),
 * 位置闭环还要 origin_x/y 做平移基准。只存点不存这三个,载入后参考系是
 * 上一次录制(或冷启动的 0),整段参考航向偏多少,车就照着偏多少。 */
typedef struct
{
    float origin_yaw;       /* 录制起点航向(度,世界系) */
    float origin_x;         /* 录制起点世界坐标 X(米) */
    float origin_y;         /* 录制起点世界坐标 Y(米) */
} kart_flash_meta_t;

/* 把 count 个路径点 + 并行的 steer/dist 数组 + 页头 meta 写入 slot 槽位
 * (阻塞擦写,只能停车时调)。
 * steer/dist/meta 允许传 NULL:缺的按 0 存,读回来就是 0(老行为)。
 * 返回 0=成功,1=slot 越界,2=count 为 0/wp 无效。 */
uint8  kart_flash_save_path(uint8 slot, const kart_waypoint_t *wp,
                            const int16 *steer, const float *dist,
                            const kart_flash_meta_t *meta, uint16 count);

/* 从 slot 槽位读回路径点到 wp_out(最多 max_count 个)。
 * steer_out/dist_out/meta_out 可传 NULL(不需要就不取)。
 * 返回实际读出点数;0 表示槽位空/magic 不符/slot 越界。 */
uint16 kart_flash_load_path(uint8 slot, kart_waypoint_t *wp_out,
                            int16 *steer_out, float *dist_out,
                            kart_flash_meta_t *meta_out, uint16 max_count);

/* 查询 slot 槽位是否存有有效路径(magic 匹配)。有=返回点数,无=0。
 * 只读页头 2 word,快;供菜单显示"槽位是否已录"用。 */
uint16 kart_flash_slot_count(uint8 slot);

#endif
