/*********************************************************************************************************************
 * 文件名称  kart_person_link
 * 所属分层  Kart_App（与 kart_remote / kart_voice 同层：占一个 UART 收帧并解释成物理量）
 *           为何不放 Kart_Driver：本模块要往外输出 kart_vtrack_result_t（Kart_Algo 的类型），
 *           而 Kart_Driver 与 Kart_Algo 是同一层的平级目录，Kart_Driver 引 Kart_Algo 类型是横向依赖。
 *           kart_remote 已经立了这个先例：它也只是“一个 UART + 一个帧格式”，同样在 Kart_App。
 * 功能说明  TC4D7 人体跟踪遥测帧接收 + 转成 kart_vtrack_result_t 送给 kart_follow
 *
 * 【为什么有这个模块】
 *   387 跑不了人体检测模型（模型代码里的 __vccm / vloadN_typed_vec_q8 / vNint_t
 *   是 ARC PPU 向量内建，TriCore 编译器直接报错；即使手改成标量，单帧
 *   二十多 M MAC 在 300MHz 上也是百毫秒级，把 5ms 控制环吃死）。
 *   所以视觉放在 TC4D7（带 PPU），387 只拿结果做运动控制。
 *   4D7 那侧发送代码已完成，帧格式见 4D7 工程的 PERSON_TELEMETRY_PROTOCOL.md。
 *   本文件只实现接收端，不往 4D7 发任何东西（单向链路）。
 *
 * 【帧格式（25 字节定长，小端）】
 *   off len 字段
 *    0   1  HEAD0 = 0xA5
 *    1   1  HEAD1 = 0x5A
 *    2   1  VERSION = 0x01
 *    3   1  TYPE = 0x01
 *    4   1  PAYLOAD_LEN = 18
 *    5   2  SEQUENCE        uint16，自然回绕
 *    7   1  FLAGS           bit0 VALID / bit1 TRACKING / bit2 FILTER_READY
 *                           bit3 FAR  / bit4 MID      / bit5 NEAR
 *    8   1  ZONE            int8  -3..+3，负=左
 *    9   2  STEERING_ERROR  int16 -1000..+1000，负=往左打
 *   11   2  CENTER_X        uint16 0..1000（归一化×1000）
 *   13   2  CENTER_Y        uint16 0..1000
 *   15   2  WIDTH           uint16 0..1000
 *   17   2  HEIGHT          uint16 0..1000
 *   19   2  CONFIDENCE      uint16 0..1000
 *   21   2  RUN_TIME        uint16 单位 0.1ms
 *   23   2  CRC16           CRC-16/CCITT-FALSE，覆盖 off 2..22（共 21 字节），低字节先发
 *
 * 【端口选择：为何不新开一个 UART】
 *   387 上没有真正空闲的引脚：
 *     UART_0  P14.0/P14.1 —— board_pins.h 已记档 P14.x 是 boot 相关脚，占了不好下载，
 *                            且 2026-07-26 实测过 TLD7002 在这里收发不通，只当 VOFA 备用。
 *     UART_3  P15.6/P15.7 —— SBUS 遥控，唯一的人工接管安全绳，不能动。
 *   所以复用。两个选项（PERSON_LINK_PORT，定义在 board_pins.h）：
 *     LIGHT = UART_1  P11.12(TX)/P11.10(RX)  灯板 TLD7002 飞线位
 *             视觉阶段把灯板拔下来插 4D7，跟随调完再插回灯板。
 *             两者不同时使用，且本宏一开就把点阵屏整条链路（tld7002_init /
 *             tld7002_set_duty / 1ms CCU61_CH0 扇描 / uart1_rx_isr 里的 tld7002_callback）
 *             全部静默，不存在两个主抢 ASCLIN1 波特率的情况。
 *     VOFA  = UART_10 P13.0(TX)/P13.1(RX)   无线模块排针位
 *             比赛不让接无线模块 → 这个坐子比赛时本来就是空的。
 *             但调试时 VOFA 日志也在这个口，两者真冲突 → 选了这个口就必须同时
 *             把 LOG_ON_UART0 改 1（日志改走 USB-TTL 直插 P14.0/P14.1），
 *             否则 board_pins.h 会 #error 拦下来。
 *
 * 【为何用 RX 中断而不像 kart_voice 那样轮询】
 *   uart_query_byte() 读的是 ASCLIN 硬件 RX FIFO（IfxAsclin_getRxFifoFillLevel），
 *   深度有限。本链路是连续流：115200 下 25 字节帧只需 ~2.2ms 发完，
 *   若放到 10ms 任务里轮询，一整帧突发进来时 FIFO 已溢出丢字节。
 *   语音模块能轮询是因为它的帧稀疏（人说一句才来一帧），本模块不是。
 *   所以抬 kart_remote 那套：中断逐字节喰（kart_person_link_rx_callback），
 *   主循环只负责计时与失联判定（kart_person_link_poll）。
 *   中断里绝不能用 uart_read_byte()：它在 zf_driver_uart.c 里是
 *   while(getRxFifoFillLevel()==0) 阻塞自旋。已确认进中断时 FIFO 非空，
 *   故这里用它是安全的（和 kart_remote 一致），但轮询路径上一律用 uart_query_byte。
 *
 * 【并发】
 *   收帧缓冲与状态机只在中断上下文访问。最新帧由中断写、主循环读，
 *   读取用 interrupt_global_disable 成组快照，避免读到半更新的帧。
 *   idle_ms 由主循环累加、中断清零（单次写，天然安全）。
 *
 * 修改记录
 * date            author        note
 * 2026-08-12      kart          首版：4D7 → 387 人体跟踪遥测链路接收端
 ********************************************************************************************************************/
#ifndef KART_PERSON_LINK_H_
#define KART_PERSON_LINK_H_
#include "zf_common_headfile.h"
#include "board_pins.h"
#include "kart_calc.h"        /* degree_to_rad；并把 rad_to_degree 带给包本头的诊断页 */
#include "kart_vtrack.h"    /* kart_vtrack_result_t：本模块的对外输出类型 */

/*=========================== 总开关与端口 ===========================*/
/* PERSON_LINK_ENABLE / PERSON_LINK_PORT / BOARD_PERSON_LINK_* 全在
 * board_pins.h（已在上面 kart_include）。不在本文件重复定义也不给 #ifndef 默认值：
 * 两处都能定义的宏，早晚会出现两处不同步而又都能编过的情况。
 * 端口选型的完整理由也写在 board_pins.h 那一段。 */

/*=========================== 帧常量 ===========================*/
#define PLINK_FRAME_LEN            (25U)   /* 定长，含 2 字节帧头与 2 字节 CRC */
#define PLINK_HEAD0                (0xA5U)
#define PLINK_HEAD1                (0x5AU)
#define PLINK_VERSION              (0x01U)
#define PLINK_TYPE_PERSON          (0x01U)
#define PLINK_PAYLOAD_LEN          (18U)
#define PLINK_CRC_COVER_BEGIN      (2U)    /* CRC 覆盖 off 2..22 */
#define PLINK_CRC_COVER_LEN        (21U)

/* FLAGS 位定义（与 4D7 协议文档一致，bit6/7 保留不读）*/
#define PLINK_FLAG_VALID           (0x01U)
#define PLINK_FLAG_TRACKING        (0x02U)
#define PLINK_FLAG_FILTER_READY    (0x04U)
#define PLINK_FLAG_FAR             (0x08U)
#define PLINK_FLAG_MID             (0x10U)
#define PLINK_FLAG_NEAR            (0x20U)

/*=========================== 波特率 ===========================*/
/* 4D7 那侧写死 115200 8N1（UART4 与 UART11 同帧同序号），改不得。 */
#define PERSON_LINK_BAUD           (115200U)

/*=========================== 失联与重同步 ===========================*/
/* 连续多久没收到 VALID 帧就当目标丢失（ms）。
 * 协议文档接收端规则 6 明写：>200ms 无 VALID 帧 → 目标丢失，停车或减速。
 * 这是安全红线，不要往上调：4D7 推理约 30ms/帧，200ms 已经容得下 6 帧丢包。 */
#define PLINK_LOST_TIMEOUT_MS      (200U)

/*=========================== 视场映射 ===========================*/
/* STEERING_ERROR 归一化到 ±1 后乘本值得方位角（度）。
 * 4D7 那侧 raw_error = (cx - 0.5) * 2，即 ±1 对应画面左/右边缘，
 * 所以本值 = 水平视场半角。
 * 【未测】先填 30°（常见开发板镜头 60° HFOV 的一半）。实车必须标：
 *   把人放在车前方已知角度位置，读 VOFA ch43(bearing_deg)，
 *   比值不对就改这一个数。标偏了只影响转向增益，不会发散（比例环）。
 *   4D7 那侧还带了 0.08 死区 + 0.30 EMA，因此回传值已经是滤过的。 */
#define PLINK_HALF_FOV_DEG         (30.0f)

/* FLAGS 距离档 → kart_vtrack scale_r 的等效值。
 * kart_follow 消费两个东西：scale_level（三档）与 scale_r（连续比值，
 * 还兼任近距安全联锁的判据，NEAR_STOP_R=1.35 / NEAR_RESUME_R=1.15）。
 * 4D7 只给了三档离散量，没有连续尺度比，所以这里合成：
 *   FAR → 0.85（< FAR_THRESH 0.90，判 TOO_FAR）
 *   MID → 1.00（死区内，NORMAL）
 *   NEAR→ 1.10（判 TOO_NEAR，但【必须低于 NEAR_RESUME_R 1.15】）
 *         2026-08-12 从 1.20 降到 1.10：原值 1.20 > NEAR_RESUME_R 1.15，
 *         4D7 一报 NEAR 就永久锁停出不来（scale_r 恒在 1.20，达不到
 *         解锁线 1.15 以下）。现在梅子自洽：
 *         FAR 0.85 < MID 1.00 < NEAR 1.10 < RESUME 1.15 < STOP_R 1.35 < SCALE_R_STOP 1.40
 * 为何 NEAR 不直接给 1.40 去触联锁：NEAR 档在 4D7 那侧的阈值是
 * height > 0.75，人在画面里占得高就算 NEAR，这距离未必真到要刹车的程度，
 * 而且三档量跳变，一进 NEAR 就锁停会频繁点头。距离安全靠
 * PLINK_NEAR_HEIGHT_STOP 单独判（下面），那个用的是连续量 height。 */
#define PLINK_SCALE_R_FAR          (0.85f)
#define PLINK_SCALE_R_MID          (1.00f)
#define PLINK_SCALE_R_NEAR         (1.10f)

/* 真正的近距刹车阈值：归一化 HEIGHT 超此值就把 scale_r 推到
 * NEAR_STOP_R 以上，让 kart_follow 的联锁锁停。
 * 0.85：4D7 的 NEAR 档阈值是 0.75，再紧一档作为刹车线。
 * 【未测】实车要量：人站在目标跟车距离 1.5m 处读 ch41(height)，
 * 把本值设在它与“人贴到车前”读数之间。 */
#define PLINK_NEAR_HEIGHT_STOP     (0.85f)

/* 进了刹车区后的回差：height 降到本值以下才把 scale_r 放回正常，
 * 避免在阈值上抗动。kart_follow 自己也有一层回差（RESUME_R 1.15），
 * 两层叠加不冲突：本层管“要不要抬到刹车值”，那层管“什么时候释锁”。 */
#define PLINK_NEAR_HEIGHT_RESUME   (0.78f)

/* 刹车命中时往上抬的 scale_r 值。
 * 【耦合点，改一个必须看另一个】它必须严格大于 kart_follow.h 的
 * FOLLOW_NEAR_STOP_R（当前 1.35），否则 kart_follow 的近距联锁永远不会触发，
 * 本模块的 height 刹车判定就成了空转。取 1.40 留 0.05 余量。
 * 为何不直接引用 FOLLOW_NEAR_STOP_R：kart_follow 在 Kart_Decision，本模块在 Kart_App，
 * Kart_App 引 Kart_Decision 是反向依赖（kart_include.h 定的 Kart_Decision → Kart_App 单向）。
 * 宁可写两个数字加一段注释，也不能把依赖方向绕反。 */
/* 2026-08-17 1.72 -> 2.19: 跟随 NEAR_STOP_R 从 1.67 抬到 2.14(停车距离 0.70m)。
 * 仍保持高 0.05 的余量。不抬这个数，本模块的 height 刹车就再也推不动了。 */
#define PLINK_SCALE_R_STOP         (2.19f)

/*=========================== 输出类型 ===========================*/
/* 链路原始帧（已过 CRC 与序号新鲜度检查，字段已反序列化但未归一化）。
 * 给诊断页/VOFA 看原始值用；控制链路请用 kart_person_link_vtrack()。 */
typedef struct
{
    uint16 sequence;            /* 发送侧帧序号，自然回绕 */
    uint8  flags;               /* PLINK_FLAG_* 位集合 */
    int8   zone;                /* -3..+3，负=左 */
    int16  steering_error;      /* -1000..+1000，负=往左打 */
    uint16 center_x;            /* 0..1000 */
    uint16 center_y;            /* 0..1000 */
    uint16 width;               /* 0..1000 */
    uint16 height;              /* 0..1000 */
    uint16 confidence;          /* 0..1000 */
    uint16 run_time;            /* 4D7 推理耗时，单位 0.1ms */
}kart_person_frame_t;

/* 链路诊断计数。全部只增不减，回绕也无害（看增量不看绝对值）。
 * 这组数字是现场定位链路问题的唯一依据，已接到 VOFA CH39-44：
 *   frame_ok 不涨              → 线没接对/波特率不对/4D7 没在发（先看 byte_count）
 *   byte_count 涨但 frame_ok 不涨 → 帧头没对上或 CRC 全错（看 crc_err/hdr_err）
 *   crc_err 与 frame_ok 同量级    → 电平/干扰/共地问题，不是协议问题
 *   seq_stale 大量              → 两个口（UART4/UART11）都接上了，帧收了两遍 */
typedef struct
{
    uint32 byte_count;          /* 中断收到的字节总数 */
    uint32 frame_ok;            /* 头+类型+CRC+序号全过的帧数 */
    uint32 crc_err;             /* CRC 不符丢弃数 */
    uint32 hdr_err;             /* VERSION/TYPE/PAYLOAD_LEN 不符丢弃数 */
    uint32 seq_stale;           /* 序号不新丢弃数（重包/乱序）*/
    uint32 resync;              /* RX 环形缓冲溢出后放弃半帧的次数 */
    uint32 lost_events;         /* 从在线转失联的次数（上升沿计数）*/
}kart_person_link_stat_t;

/*=========================== 对外接口 ===========================*/

/* 初始化。内部做 uart_init(115200) + uart_rx_interrupt(1)。
 * 【调用位置】放在 kart_menu_init()（含开机动画）之后、正式主循环之前：
 *   避免 4D7 连续串口中断拖慢软件 SPI 动画；此时摄像头配置、点阵端口仲裁和
 *   debug UART 初始化均已完成，后初始化者明确取得所选 UART 的所有权。 */
void kart_person_link_init          (void);

/* 科目生命周期门控：进入科目三传 1，退出传 0。待机/普通菜单时 UART RX 中断关闭。 */
void kart_person_link_set_rx_enabled(uint8 enabled);

/* 10ms 周期调用。限量排空 RX、拼帧校验，再刷新失联状态与 kart_vtrack；不阻塞。 */
void kart_person_link_poll          (uint16 period_ms);

/* UART RX 中断回调。一次只读一个字节放入环形缓冲，不在中断内解析或算 CRC。 */
void kart_person_link_rx_callback   (void);

/* 目标是否在线：最近 PLINK_LOST_TIMEOUT_MS 内收到过带 VALID 位的合法帧。
 * 返回 0 时上层必须停车或减速（协议规则 6）。 */
uint8 kart_person_link_is_online    (void);

/* 取最新一帧原始值快照。dst 不可为 NULL。返回 0 表示从上电到现在没收到过合法帧。 */
uint8 kart_person_link_get_frame    (kart_person_frame_t *dst);

/* 取诊断计数快照。dst 不可为 NULL。 */
void kart_person_link_get_stat      (kart_person_link_stat_t *dst);

/* 【控制链路入口】把最新帧折成 kart_vtrack_result_t，直接送给 kart_follow_update()。
 * 返回的指针永不为 NULL（指向模块内静态体）。失联/无效时 valid=0，
 * kart_follow 自己会走丢失分支减速，不需要上层再判一遍。
 * 【为何返回 kart_vtrack 而不是 kart_vision】kart_follow_update() 的入参 2026-08-10 已改成
 * const kart_vtrack_result_t *，两个结构体内存布局不兼容（kart_vision 第三个字段是
 * float dist_m，kart_vtrack 是 enum scale_level），传错了不是编译错而是跟随行为鬼异。 */
const kart_vtrack_result_t *kart_person_link_vtrack (void);

#endif
