#ifndef KART_MISSION_H_
#define KART_MISSION_H_

#include "zf_common_headfile.h"
#include "kart_light.h"

/*
 * 卡丁快跑比赛任务状态机
 *
 * 设计边界：
 * 1. 本模块只决定“当前处于哪个比赛阶段、希望底层执行什么动作”，不直接写 PWM。
 * 2. 所有会动车的科目都必须通过就绪位检查；底层未完成时无法进入 RUNNING。
 * 3. 传感器/路径规划完成一个阶段后，通过 kart_mission_signal() 推进状态。
 * 4. IPS200、串口和按键只调用本文件的接口，不允许自行改内部状态。
 */

/* 底层能力就绪位。只有经过台架验证的模块才允许置 1。 */
#define KART_MISSION_READY_SPEED_CTRL       (1UL << 0)
#define KART_MISSION_READY_STEER_CTRL       (1UL << 1)
#define KART_MISSION_READY_YAW_CTRL         (1UL << 2)
#define KART_MISSION_READY_SAFE_STOP        (1UL << 3)
#define KART_MISSION_READY_PATH_FOLLOW      (1UL << 4)
#define KART_MISSION_READY_VOICE            (1UL << 5)
#define KART_MISSION_READY_LIGHT            (1UL << 6)
#define KART_MISSION_READY_HORN             (1UL << 7)
#define KART_MISSION_READY_PATH_RECORD      (1UL << 8)
#define KART_MISSION_READY_GUIDE_FOLLOW     (1UL << 9)

#define KART_MISSION_S1_REQUIRED            (KART_MISSION_READY_SPEED_CTRL  | \
                                             KART_MISSION_READY_STEER_CTRL  | \
                                             KART_MISSION_READY_YAW_CTRL    | \
                                             KART_MISSION_READY_SAFE_STOP   | \
                                             KART_MISSION_READY_PATH_FOLLOW)

#define KART_MISSION_S2_REQUIRED            (KART_MISSION_READY_SPEED_CTRL  | \
                                             KART_MISSION_READY_STEER_CTRL  | \
                                             KART_MISSION_READY_YAW_CTRL    | \
                                             KART_MISSION_READY_SAFE_STOP   | \
                                             KART_MISSION_READY_PATH_FOLLOW | \
                                             KART_MISSION_READY_VOICE       | \
                                             KART_MISSION_READY_LIGHT       | \
                                             KART_MISSION_READY_HORN)

#define KART_MISSION_S3_REQUIRED            (KART_MISSION_READY_SPEED_CTRL   | \
                                             KART_MISSION_READY_STEER_CTRL   | \
                                             KART_MISSION_READY_YAW_CTRL     | \
                                             KART_MISSION_READY_SAFE_STOP    | \
                                             KART_MISSION_READY_PATH_FOLLOW  | \
                                             KART_MISSION_READY_PATH_RECORD  | \
                                             KART_MISSION_READY_GUIDE_FOLLOW)

/* 科目二原文只明确“四区域共 8 条”，未给文字版分区配额。
 * 当前按场地往返流程推定为 2+2+1+2+1；拿到正式抽签任务表后需复核这些宏。 */
#define KART_MISSION_S2_LIGHT_TASK_COUNT     (2U)
#define KART_MISSION_S2_HORN_TASK_COUNT      (2U)
#define KART_MISSION_S2_OUT_GATE_TASK_COUNT  (1U)
#define KART_MISSION_S2_MOTION_TASK_COUNT    (2U)
#define KART_MISSION_S2_BACK_GATE_TASK_COUNT (1U)
#define KART_MISSION_S2_TOTAL_TASK_COUNT     (8U)

typedef enum
{
    KART_SUBJECT_NONE = 0,
    KART_SUBJECT_1_AUTO_DRIVE,
    KART_SUBJECT_2_INTERACTION,
    KART_SUBJECT_3_PATH_RETURN
} kart_subject_t;

typedef enum
{
    KART_MISSION_IDLE = 0,
    KART_MISSION_ARMED,
    KART_MISSION_RUNNING,
    KART_MISSION_FINISHED,
    KART_MISSION_ABORTED,
    KART_MISSION_FAULT
} kart_mission_run_state_t;

typedef enum
{
    KART_S1_DISABLED = 0,
    KART_S1_WAIT_START,
    KART_S1_LEAVE_START,
    KART_S1_SLALOM,
    KART_S1_APPROACH_GARAGE,
    KART_S1_REVERSE_ALIGN,
    KART_S1_REVERSE_PARK,
    KART_S1_FINISHED
} kart_subject1_state_t;

typedef enum
{
    KART_S2_DISABLED = 0,
    KART_S2_WAIT_LIGHT,
    KART_S2_EXEC_LIGHT,
    KART_S2_WAIT_HORN,
    KART_S2_EXEC_HORN,
    KART_S2_WAIT_OUT_GATE,
    KART_S2_EXEC_OUT_GATE,
    KART_S2_WAIT_MOTION,
    KART_S2_EXEC_MOTION,
    KART_S2_WAIT_BACK_GATE,
    KART_S2_EXEC_BACK_GATE,
    KART_S2_FINISHED
} kart_subject2_state_t;

typedef enum
{
    KART_S3_DISABLED = 0,
    KART_S3_WAIT_RECORD,
    KART_S3_RECORDING,
    KART_S3_READY_RETURN,
    KART_S3_RETURNING,
    KART_S3_FINISHED
} kart_subject3_state_t;

/* 科目二鸣笛命令。规则文字称“八种”，但实际逐项列出了以下九种。 */
typedef enum
{
    KART_HORN_NONE = 0,
    KART_HORN_1_SECOND,
    KART_HORN_2_SECONDS,
    KART_HORN_3_SECONDS,
    KART_HORN_2_BEEPS,
    KART_HORN_3_BEEPS,
    KART_HORN_4_BEEPS,
    KART_HORN_LONG_SHORT,
    KART_HORN_RAPID,
    KART_HORN_ALARM,
    KART_HORN_COUNT
} kart_horn_task_t;

typedef enum
{
    KART_GATE_NONE = 0,
    KART_GATE_OUT_1_LEFT,
    KART_GATE_OUT_1,
    KART_GATE_OUT_2,
    KART_GATE_OUT_3,
    KART_GATE_OUT_3_RIGHT,
    KART_GATE_BACK_1_RIGHT,
    KART_GATE_BACK_1,
    KART_GATE_BACK_2,
    KART_GATE_BACK_3,
    KART_GATE_BACK_3_LEFT,
    KART_GATE_COUNT
} kart_gate_task_t;

typedef enum
{
    KART_MOTION_NONE = 0,
    KART_MOTION_FORWARD_DISTANCE,
    KART_MOTION_REVERSE_DISTANCE,
    KART_MOTION_SNAKE_FORWARD,
    KART_MOTION_SNAKE_REVERSE,
    KART_MOTION_CIRCLE_CCW,
    KART_MOTION_CIRCLE_CW,
    KART_MOTION_TURN_LEFT,
    KART_MOTION_TURN_RIGHT,
    KART_MOTION_COUNT
} kart_motion_task_t;

/* 状态机向后续路径规划/控制层发出的动作请求，不等同于电机输出。 */
typedef enum
{
    KART_REQUEST_STOP = 0,
    KART_REQUEST_S1_FORWARD_PATH,
    KART_REQUEST_S1_REVERSE_ALIGN,
    KART_REQUEST_S1_REVERSE_PARK,
    KART_REQUEST_S2_GATE,
    KART_REQUEST_S2_MOTION,
    KART_REQUEST_S3_RECORD_PATH,
    KART_REQUEST_S3_AUTO_RETURN
} kart_mission_motion_request_t;

typedef enum
{
    KART_MISSION_EVENT_NONE = 0,
    KART_MISSION_EVENT_S1_ENTER_SLALOM,
    KART_MISSION_EVENT_S1_SLALOM_DONE,
    KART_MISSION_EVENT_S1_GARAGE_REACHED,
    KART_MISSION_EVENT_S1_ALIGN_DONE,
    KART_MISSION_EVENT_S1_PARK_DONE,
    KART_MISSION_EVENT_TASK_DONE,
    KART_MISSION_EVENT_S3_RECORD_DONE,
    KART_MISSION_EVENT_S3_RETURN_START,
    KART_MISSION_EVENT_S3_RETURN_DONE
} kart_mission_event_t;

typedef enum
{
    KART_MISSION_ERROR_NONE = 0,
    KART_MISSION_ERROR_NO_SUBJECT,
    KART_MISSION_ERROR_NOT_READY,
    KART_MISSION_ERROR_BAD_STATE,
    KART_MISSION_ERROR_BAD_TASK,
    KART_MISSION_ERROR_READY_LOST,
    KART_MISSION_ERROR_EXTERNAL = 100
} kart_mission_error_t;

typedef struct
{
    kart_mission_motion_request_t motion;
    kart_light_command_t light;
    kart_horn_task_t horn;
    kart_gate_task_t gate;
    kart_motion_task_t motion_task;
} kart_mission_request_t;

typedef struct
{
    kart_subject_t subject;
    kart_mission_run_state_t run_state;
    uint8 subject_state;
    uint8 subject2_tasks_done;
    uint8 subject2_zone_tasks_done;
    uint16 transition_count;
    uint32 ready_mask;
    uint32 required_mask;
    uint32 missing_mask;
    uint32 state_elapsed_ms;
    uint32 total_elapsed_ms;
    kart_mission_error_t last_error;
} kart_mission_status_t;

void  kart_mission_init                (void);
void  kart_mission_tick                (uint16 elapsed_ms);
void  kart_mission_poll                (void);

uint8 kart_mission_select_subject      (kart_subject_t subject);
uint8 kart_mission_arm                 (void);
uint8 kart_mission_start               (void);
void  kart_mission_abort               (void);
void  kart_mission_fault               (uint16 external_code);
void  kart_mission_reset               (void);

void  kart_mission_set_ready_mask      (uint32 ready_mask);
void  kart_mission_set_ready           (uint32 mask, uint8 ready);

uint8 kart_mission_signal              (kart_mission_event_t event);
uint8 kart_mission_submit_light        (kart_light_command_t task);
uint8 kart_mission_submit_horn         (kart_horn_task_t task);
uint8 kart_mission_submit_gate         (kart_gate_task_t task);
uint8 kart_mission_submit_motion       (kart_motion_task_t task);

void  kart_mission_get_status          (kart_mission_status_t *status);
void  kart_mission_get_request         (kart_mission_request_t *request);
uint8 kart_mission_stop_is_required    (void);

const char *kart_mission_subject_name  (kart_subject_t subject);
const char *kart_mission_run_name      (kart_mission_run_state_t state);
const char *kart_mission_stage_name    (void);

#endif
