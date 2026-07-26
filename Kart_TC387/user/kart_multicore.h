#ifndef KART_MULTICORE_H_
#define KART_MULTICORE_H_

#include "zf_common_headfile.h"

/*
 * Phase-1 multicore compatibility scheduler.
 *
 * The caller keeps the original execution order and waits for the target
 * core to finish.  This moves ownership and hot data to the intended core
 * without changing menu/remote/control behaviour.  Set to 0 for an immediate
 * local-execution fallback during hardware bring-up.
 */
#define KART_MULTICORE_COMPAT_ENABLE    (0)

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

/* One service attempt. Returns 1 when a request was executed. */
uint8 kart_multicore_core1_service(void);
uint8 kart_multicore_core2_service(void);
uint8 kart_multicore_core3_service(void);

/* Diagnostic counters; one count means one completed remote request. */
uint32 kart_multicore_get_heartbeat(uint8 core_id);

#endif
