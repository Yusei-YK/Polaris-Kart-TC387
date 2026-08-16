#ifndef KART_MULTICORE_H_
#define KART_MULTICORE_H_
#include "zf_common_headfile.h"
#include "kart_vision.h"

/*
 * Phase-1 kart_multicore compatibility scheduler.
 *
 * The caller keeps the original execution order and waits for the target
 * core to finish.  This moves ownership and hot data to the intended core
 * without changing kart_menu/kart_remote/kart_control behaviour.  Set to 0 for an immediate
 * local-execution fallback during hardware bring-up.
 */
#define KART_MULTICORE_COMPAT_ENABLE    (0)

/* ---------------- core3 异步视觉 ----------------
 * 与上面的 COMPAT 通道【完全独立】,不受 COMPAT_ENABLE 和 runtime_enabled 管。
 *
 * 为什么必须另开一套:COMPAT 的 kart_mc_request() 发完请求就自旋等 done_seq,
 * 那是同步语义。视觉一帧要 360ms,同步等只是把 360ms 从 CPU0 搬到 CPU0 等 core3,
 * 控制环照样饿死,一点没救。所以这套是投递即返回,结果下次再取。
 *
 * 【实测依据】1.csv 跟随段:出新帧的行平均丢 34 拍,没新帧的行只丢 2.4 拍,
 * 停顿严格绑在"这一拍跑了 kart_vision_process"上。有效 tick 率 26Hz(标称 200),
 * 转向内环按 5ms 标定却 38ms 才执行一次 → 1.12Hz 自激,就是乱打左右方向。
 *
 * core3 是空闲的:COMPAT_ENABLE=0 时 core3_service() 从来没执行过任何命令,
 * 点阵屏实际由 CPU0 的 1ms PIT(cc61_pit_ch0_isr)扫,不在 core3 上。
 * 灯板和视觉不会同时跑,这是选 core3 的前提。 */
#define MC_VISION_ENABLE           (1)

void  kart_multicore_init(void);
void  kart_multicore_mark_online(uint8 core_id);
void  kart_multicore_enable_runtime(void);
uint8 kart_multicore_is_runtime_enabled(void);

/* Drop-in wrappers used at the original CPU0 call sites. */
void kart_multicore_imu_update(void);
void kart_multicore_odom_update(void);
void kart_multicore_record_poll(void);
void kart_multicore_dot_show_string(const char *str);
void kart_multicore_dot_scan(void);

/* ---------------- core3 异步视觉:CPU0 侧接口 ----------------
 * 三个都是非阻塞,任何一个都不会自旋等 core3。 */

/* 投递一帧给 core3。img 必须指向在 core3 读完前不会被覆写的缓冲。
 * 返回 1 = 已投递(core3 之前是空闲的);0 = core3 还在算上一帧,本次不投。
 * 返回 0 时调用方【不要】等,直接沿用上一帧结果。 */
uint8 multicore_vision_submit(const uint16 *img, int16 w, int16 h);

/* core3 是否已算完一帧新结果(未被 fetch 过)。 */
uint8 multicore_vision_ready(void);

/* 取最近一次算完的结果快照。ready==0 时返回的是上一次的旧值,
 * 所以调用方必须靠 ready 自己维护年龄,不能拿它当"新鲜"的证据。
 * 返回的是 CPU0 侧私有副本,core3 不会在你读的时候改它。 */
const kart_vision_result_t *multicore_vision_get(void);

/* 诊断:core3 算完的帧数、上一帧耗时(us)、历史最大耗时(us)、投递被拒次数。 */
uint32 multicore_vision_frames(void);
uint32 multicore_vision_last_us(void);
uint32 multicore_vision_max_us(void);
uint32 multicore_vision_reject(void);

/* ---------------- core2 屏幕绘制代理 ----------------
 * 【为什么要它】IPS200 走软件 SPI(zf_device_ips200.h:72 IPS200_USE_SOFT_SPI=1),
 * 每个 bit 都是 CPU 手动翻 GPIO:1 次 MOSI 写 + 2 次 gpio_toggle_level,
 * 而 toggle 是外设寄存器读-改-写。实测一拍菜单刷新要推 61.8 万 bit,
 * 每 bit 约 574ns → 355ms。这一下把 71 个 5ms 控制拍全挤掉了。
 * 屏幕是纯 CPU-bound 的 GPIO 翻脚,不争任何外设,正好甩给空闲核。
 *
 * 【为什么不把 kart_menu_poll 整个搬过去】menu_draw_* 要读十几个模块的状态,
 * 而 kart_menu_input_poll 还在 core0 每 10ms 写 current_level/need_repaint。
 * 两核一读一写同一批非 volatile 变量就是真竞争,按键会丢。
 * 所以边界划在原语层:core0 独占菜单状态,core2 独占屏幕。
 *
 * 【为什么必须整页搬,不能只搬出图】zf_device_ips200.c:74-87 那批
 * pencolor/bgcolor/ips200_spi 全是文件级 static,ui_bar 每行都改 pencolor。
 * 只搬出图会让两核抢这些 static → 花屏。而且文本占 48%(135 字 x 256 字节),
 * 只搬出图 355ms 只降到 174ms,过不了 max_exec_us < 10000 的验收。
 *
 * 【节流语义】core0 产生一帧要 50ms,core2 排完要 355ms,生产比消费快 7 倍。
 * 所以按整帧节流:core2 还在画就直接丢掉这次重画(跟 vision_submit 一样),
 * 不排队。排队会让屏上画面越来越滞后,那比丢帧难查得多。 */

/* =1 走 core2 代理;=0 退回 core0 本地直画(原行为)。
 * 留这个开关是为了现场出问题能立刻拿到一个能跑的版本:改 0 重编,
 * 屏幕行为跟搬核前逐字节相同,只是 355ms 卡顿回来。 */
#define DRAW_ON_CORE2      (1)

/* 一帧最多几条指令。热路径实测:s3_live 约 20 条、kart_camera 页约 25 条、
 * settings 约 15 条。给到 96 是留余量,超了就丢尾部并累加 overflow 计数
 * (宁可少画一行也不能越界写)。 */
#define DRAW_CMD_MAX       (96)

/* 定宽文本最长 30 字(kart_menu.c UI_COLS),36 留结尾和余量。 */
#define DRAW_STR_MAX       (36)

typedef enum
{
    DRAW_OP_NONE = 0,
    DRAW_OP_CLEAR,         /* ips200_clear(),整屏 76800 像素 */
    DRAW_OP_STRING,        /* set_color + show_string,颜色随指令带 */
    DRAW_OP_LINE,
    DRAW_OP_POINT,
    DRAW_OP_IMAGE          /* 直接读 scc8660_image,不带像素数据 */
}draw_op_t;

/* core0 侧:开一帧。返回 0 = core2 还在画上一帧,本次整帧跳过,
 * 调用方【不要】等,直接 return(屏幕少刷一帧没有后果)。 */
uint8 draw_begin(void);

/* core0 侧:入队。只在 begin 返回 1 之后调,内部不再检查 busy。 */
void draw_clear(uint16 fg, uint16 bg);
void draw_string(uint16 x, uint16 y, const char *s, uint16 fg, uint16 bg);
void draw_line(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color);
void draw_point(uint16 x, uint16 y, uint16 color);
void draw_image(uint16 x, uint16 y);

/* core0 侧:提交给 core2。begin 返回 1 就必须配一次 commit,否则 busy 不释放。 */
void draw_commit(void);

/* 诊断:已画帧数、被丢帧数、指令溢出次数、core2 上一帧耗时(us)、历史最大(us)。 */
uint32 draw_frames(void);
uint32 draw_dropped(void);
uint32 draw_overflow(void);
uint32 draw_last_us(void);
uint32 draw_max_us(void);

/* One service attempt. Returns 1 when a request was executed. */
uint8 kart_multicore_core1_service(void);
uint8 kart_multicore_core2_service(void);
uint8 kart_multicore_core3_service(void);

/* Diagnostic counters; one count means one completed kart_remote request. */
uint32 kart_multicore_get_heartbeat(uint8 core_id);

#endif
