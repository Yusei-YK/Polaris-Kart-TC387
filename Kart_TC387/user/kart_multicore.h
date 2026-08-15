#ifndef KART_MULTICORE_H_
#define KART_MULTICORE_H_

#include "zf_common_headfile.h"
#include "kart_vision.h"

/*
 * Phase-1 multicore compatibility scheduler.
 *
 * The caller keeps the original execution order and waits for the target
 * core to finish.  This moves ownership and hot data to the intended core
 * without changing menu/remote/control behaviour.  Set to 0 for an immediate
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
#define KART_MC_VISION_ENABLE           (1)

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
uint8 kart_multicore_vision_submit(const uint16 *img, int16 w, int16 h);

/* core3 是否已算完一帧新结果(未被 fetch 过)。 */
uint8 kart_multicore_vision_ready(void);

/* 取最近一次算完的结果快照。ready==0 时返回的是上一次的旧值,
 * 所以调用方必须靠 ready 自己维护年龄,不能拿它当"新鲜"的证据。
 * 返回的是 CPU0 侧私有副本,core3 不会在你读的时候改它。 */
const kart_vision_result_t *kart_multicore_vision_get(void);

/* 诊断:core3 算完的帧数、上一帧耗时(us)、历史最大耗时(us)、投递被拒次数。 */
uint32 kart_multicore_vision_frames(void);
uint32 kart_multicore_vision_last_us(void);
uint32 kart_multicore_vision_max_us(void);
uint32 kart_multicore_vision_reject(void);

/* One service attempt. Returns 1 when a request was executed. */
uint8 kart_multicore_core1_service(void);
uint8 kart_multicore_core2_service(void);
uint8 kart_multicore_core3_service(void);

/* Diagnostic counters; one count means one completed remote request. */
uint32 kart_multicore_get_heartbeat(uint8 core_id);

#endif
