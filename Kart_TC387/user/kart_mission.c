#include "kart_mission.h"

#include <string.h>

/* 5ms 中断更新计时/故障，主循环读取并提交事件，因此共享状态必须保持可见性。 */
static volatile kart_mission_status_t kart_mission_status;
static volatile kart_mission_request_t kart_mission_request;

static uint32 kart_mission_required_mask(kart_subject_t subject)
{
    switch(subject)
    {
        case KART_SUBJECT_1_AUTO_DRIVE:
            return KART_MISSION_S1_REQUIRED;

        case KART_SUBJECT_2_INTERACTION:
            return KART_MISSION_S2_REQUIRED;

        case KART_SUBJECT_3_PATH_RETURN:
            return KART_MISSION_S3_REQUIRED;

        default:
            return 0U;
    }
}

static uint32 kart_mission_saturating_add(uint32 value, uint16 increment)
{
    uint32 result = value + (uint32)increment;

    if(result < value)
    {
        result = 0xFFFFFFFFUL;
    }

    return result;
}

static void kart_mission_clear_request(void)
{
    kart_mission_request.motion = KART_REQUEST_STOP;
    kart_mission_request.light = KART_LIGHT_CMD_OFF;
    kart_mission_request.horn = KART_HORN_NONE;
    kart_mission_request.gate = KART_GATE_NONE;
    kart_mission_request.motion_task = KART_MOTION_NONE;
}

static void kart_mission_set_stage(uint8 state)
{
    kart_mission_status.subject_state = state;
    kart_mission_status.state_elapsed_ms = 0U;
    kart_mission_status.transition_count++;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
}

static void kart_mission_request_stop(void)
{
    kart_mission_clear_request();
}

static void kart_mission_complete(uint8 finished_state)
{
    kart_mission_set_stage(finished_state);
    kart_mission_status.run_state = KART_MISSION_FINISHED;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_request_stop();
}

static void kart_mission_prepare_subject(void)
{
    kart_mission_status.subject2_tasks_done = 0U;
    kart_mission_status.subject2_zone_tasks_done = 0U;
    kart_mission_status.total_elapsed_ms = 0U;
    kart_mission_status.state_elapsed_ms = 0U;
    kart_mission_status.transition_count = 0U;
    kart_mission_clear_request();

    switch(kart_mission_status.subject)
    {
        case KART_SUBJECT_1_AUTO_DRIVE:
            kart_mission_status.subject_state = (uint8)KART_S1_DISABLED;
            break;

        case KART_SUBJECT_2_INTERACTION:
            kart_mission_status.subject_state = (uint8)KART_S2_DISABLED;
            break;

        case KART_SUBJECT_3_PATH_RETURN:
            kart_mission_status.subject_state = (uint8)KART_S3_DISABLED;
            break;

        default:
            kart_mission_status.subject_state = 0U;
            break;
    }
}

static void kart_mission_enter_s2_wait(kart_subject2_state_t next_state)
{
    kart_mission_status.subject2_zone_tasks_done = 0U;
    kart_mission_clear_request();
    kart_mission_set_stage((uint8)next_state);
}

void kart_mission_init(void)
{
    memset((void *)&kart_mission_status, 0, sizeof(kart_mission_status));
    kart_mission_status.subject = KART_SUBJECT_NONE;
    kart_mission_status.run_state = KART_MISSION_IDLE;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_clear_request();
}

void kart_mission_tick(uint16 elapsed_ms)
{
    if(kart_mission_status.run_state != KART_MISSION_RUNNING)
    {
        return;
    }

    /* 中断上下文只累计时间，不修改状态或动作请求，避免与主循环事件转换竞争。 */
    kart_mission_status.state_elapsed_ms = kart_mission_saturating_add(
        kart_mission_status.state_elapsed_ms, elapsed_ms);
    kart_mission_status.total_elapsed_ms = kart_mission_saturating_add(
        kart_mission_status.total_elapsed_ms, elapsed_ms);
}

void kart_mission_poll(void)
{
    kart_mission_status.required_mask = kart_mission_required_mask(kart_mission_status.subject);
    kart_mission_status.missing_mask = kart_mission_status.required_mask &
                                       ~kart_mission_status.ready_mask;

    /* 所有状态转换都留在主循环，保证 fault 与 submit/signal 不会交叉覆盖请求。 */
    if(kart_mission_status.run_state == KART_MISSION_RUNNING &&
       kart_mission_status.missing_mask != 0U)
    {
        kart_mission_status.run_state = KART_MISSION_FAULT;
        kart_mission_status.last_error = KART_MISSION_ERROR_READY_LOST;
        kart_mission_request_stop();
    }
}

uint8 kart_mission_select_subject(kart_subject_t subject)
{
    if(subject < KART_SUBJECT_1_AUTO_DRIVE || subject > KART_SUBJECT_3_PATH_RETURN)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_NO_SUBJECT;
        return 0U;
    }

    if(kart_mission_status.run_state == KART_MISSION_ARMED ||
       kart_mission_status.run_state == KART_MISSION_RUNNING)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    kart_mission_status.subject = subject;
    kart_mission_status.run_state = KART_MISSION_IDLE;
    kart_mission_status.required_mask = kart_mission_required_mask(subject);
    kart_mission_status.missing_mask = kart_mission_status.required_mask &
                                       ~kart_mission_status.ready_mask;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_prepare_subject();
    return 1U;
}

uint8 kart_mission_arm(void)
{
    if(kart_mission_status.subject == KART_SUBJECT_NONE)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_NO_SUBJECT;
        return 0U;
    }

    if(kart_mission_status.run_state != KART_MISSION_IDLE)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    kart_mission_status.required_mask = kart_mission_required_mask(kart_mission_status.subject);
    kart_mission_status.missing_mask = kart_mission_status.required_mask &
                                       ~kart_mission_status.ready_mask;
    if(kart_mission_status.missing_mask != 0U)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_NOT_READY;
        return 0U;
    }

    kart_mission_prepare_subject();
    kart_mission_status.run_state = KART_MISSION_ARMED;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_request_stop();

    switch(kart_mission_status.subject)
    {
        case KART_SUBJECT_1_AUTO_DRIVE:
            kart_mission_set_stage((uint8)KART_S1_WAIT_START);
            break;

        case KART_SUBJECT_2_INTERACTION:
            kart_mission_set_stage((uint8)KART_S2_WAIT_LIGHT);
            break;

        case KART_SUBJECT_3_PATH_RETURN:
            kart_mission_set_stage((uint8)KART_S3_WAIT_RECORD);
            break;

        default:
            return 0U;
    }

    return 1U;
}

uint8 kart_mission_start(void)
{
    if(kart_mission_status.run_state != KART_MISSION_ARMED)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    kart_mission_status.missing_mask = kart_mission_status.required_mask &
                                       ~kart_mission_status.ready_mask;
    if(kart_mission_status.missing_mask != 0U)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_NOT_READY;
        return 0U;
    }

    kart_mission_status.run_state = KART_MISSION_RUNNING;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;

    switch(kart_mission_status.subject)
    {
        case KART_SUBJECT_1_AUTO_DRIVE:
            kart_mission_request.motion = KART_REQUEST_S1_FORWARD_PATH;
            kart_mission_set_stage((uint8)KART_S1_LEAVE_START);
            break;

        case KART_SUBJECT_2_INTERACTION:
            /* 发车区先保持停车，等待第一条灯光语音任务。 */
            kart_mission_set_stage((uint8)KART_S2_WAIT_LIGHT);
            break;

        case KART_SUBJECT_3_PATH_RETURN:
            kart_mission_request.motion = KART_REQUEST_S3_RECORD_PATH;
            kart_mission_set_stage((uint8)KART_S3_RECORDING);
            break;

        default:
            kart_mission_status.last_error = KART_MISSION_ERROR_NO_SUBJECT;
            return 0U;
    }

    return 1U;
}

void kart_mission_abort(void)
{
    if(kart_mission_status.run_state == KART_MISSION_RUNNING ||
       kart_mission_status.run_state == KART_MISSION_ARMED)
    {
        kart_mission_status.run_state = KART_MISSION_ABORTED;
        kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
        kart_mission_request_stop();
    }
}

void kart_mission_fault(uint16 external_code)
{
    if(kart_mission_status.run_state == KART_MISSION_IDLE)
    {
        return;
    }

    kart_mission_status.run_state = KART_MISSION_FAULT;
    kart_mission_status.last_error = (external_code == 0U)
        ? KART_MISSION_ERROR_EXTERNAL
        : (kart_mission_error_t)external_code;
    kart_mission_request_stop();
}

void kart_mission_reset(void)
{
    kart_mission_status.run_state = KART_MISSION_IDLE;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_status.required_mask = kart_mission_required_mask(kart_mission_status.subject);
    kart_mission_status.missing_mask = kart_mission_status.required_mask &
                                       ~kart_mission_status.ready_mask;
    kart_mission_prepare_subject();
    kart_mission_request_stop();
}

void kart_mission_set_ready_mask(uint32 ready_mask)
{
    kart_mission_status.ready_mask = ready_mask;
    kart_mission_status.missing_mask = kart_mission_status.required_mask & ~ready_mask;
}

void kart_mission_set_ready(uint32 mask, uint8 ready)
{
    if(ready != 0U)
    {
        kart_mission_status.ready_mask |= mask;
    }
    else
    {
        kart_mission_status.ready_mask &= ~mask;
    }

    kart_mission_status.missing_mask = kart_mission_status.required_mask &
                                       ~kart_mission_status.ready_mask;
}

uint8 kart_mission_signal(kart_mission_event_t event)
{
    if(kart_mission_status.run_state != KART_MISSION_RUNNING)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    if(kart_mission_status.subject == KART_SUBJECT_1_AUTO_DRIVE)
    {
        switch(event)
        {
            case KART_MISSION_EVENT_S1_ENTER_SLALOM:
                if(kart_mission_status.subject_state == (uint8)KART_S1_LEAVE_START)
                {
                    kart_mission_set_stage((uint8)KART_S1_SLALOM);
                    return 1U;
                }
                break;

            case KART_MISSION_EVENT_S1_SLALOM_DONE:
                if(kart_mission_status.subject_state == (uint8)KART_S1_SLALOM)
                {
                    kart_mission_set_stage((uint8)KART_S1_APPROACH_GARAGE);
                    return 1U;
                }
                break;

            case KART_MISSION_EVENT_S1_GARAGE_REACHED:
                if(kart_mission_status.subject_state == (uint8)KART_S1_APPROACH_GARAGE)
                {
                    kart_mission_request.motion = KART_REQUEST_S1_REVERSE_ALIGN;
                    kart_mission_set_stage((uint8)KART_S1_REVERSE_ALIGN);
                    return 1U;
                }
                break;

            case KART_MISSION_EVENT_S1_ALIGN_DONE:
                if(kart_mission_status.subject_state == (uint8)KART_S1_REVERSE_ALIGN)
                {
                    kart_mission_request.motion = KART_REQUEST_S1_REVERSE_PARK;
                    kart_mission_set_stage((uint8)KART_S1_REVERSE_PARK);
                    return 1U;
                }
                break;

            case KART_MISSION_EVENT_S1_PARK_DONE:
                if(kart_mission_status.subject_state == (uint8)KART_S1_REVERSE_PARK)
                {
                    kart_mission_complete((uint8)KART_S1_FINISHED);
                    return 1U;
                }
                break;

            default:
                break;
        }
    }
    else if(kart_mission_status.subject == KART_SUBJECT_2_INTERACTION &&
            event == KART_MISSION_EVENT_TASK_DONE)
    {
        switch((kart_subject2_state_t)kart_mission_status.subject_state)
        {
            case KART_S2_EXEC_LIGHT:
                kart_mission_status.subject2_tasks_done++;
                kart_mission_status.subject2_zone_tasks_done++;
                kart_mission_request.light = KART_LIGHT_CMD_OFF;
                if(kart_mission_status.subject2_zone_tasks_done < KART_MISSION_S2_LIGHT_TASK_COUNT)
                {
                    kart_mission_set_stage((uint8)KART_S2_WAIT_LIGHT);
                }
                else
                {
                    kart_mission_enter_s2_wait(KART_S2_WAIT_HORN);
                }
                return 1U;

            case KART_S2_EXEC_HORN:
                kart_mission_status.subject2_tasks_done++;
                kart_mission_status.subject2_zone_tasks_done++;
                kart_mission_request.horn = KART_HORN_NONE;
                if(kart_mission_status.subject2_zone_tasks_done < KART_MISSION_S2_HORN_TASK_COUNT)
                {
                    kart_mission_set_stage((uint8)KART_S2_WAIT_HORN);
                }
                else
                {
                    kart_mission_enter_s2_wait(KART_S2_WAIT_OUT_GATE);
                }
                return 1U;

            case KART_S2_EXEC_OUT_GATE:
                kart_mission_status.subject2_tasks_done++;
                kart_mission_enter_s2_wait(KART_S2_WAIT_MOTION);
                return 1U;

            case KART_S2_EXEC_MOTION:
                kart_mission_status.subject2_tasks_done++;
                kart_mission_status.subject2_zone_tasks_done++;
                kart_mission_request.motion_task = KART_MOTION_NONE;
                kart_mission_request.motion = KART_REQUEST_STOP;
                if(kart_mission_status.subject2_zone_tasks_done < KART_MISSION_S2_MOTION_TASK_COUNT)
                {
                    kart_mission_set_stage((uint8)KART_S2_WAIT_MOTION);
                }
                else
                {
                    kart_mission_enter_s2_wait(KART_S2_WAIT_BACK_GATE);
                }
                return 1U;

            case KART_S2_EXEC_BACK_GATE:
                kart_mission_status.subject2_tasks_done++;
                kart_mission_complete((uint8)KART_S2_FINISHED);
                return 1U;

            default:
                break;
        }
    }
    else if(kart_mission_status.subject == KART_SUBJECT_3_PATH_RETURN)
    {
        if(event == KART_MISSION_EVENT_S3_RECORD_DONE &&
           kart_mission_status.subject_state == (uint8)KART_S3_RECORDING)
        {
            kart_mission_request.motion = KART_REQUEST_STOP;
            kart_mission_set_stage((uint8)KART_S3_READY_RETURN);
            return 1U;
        }

        if(event == KART_MISSION_EVENT_S3_RETURN_START &&
           kart_mission_status.subject_state == (uint8)KART_S3_READY_RETURN)
        {
            kart_mission_request.motion = KART_REQUEST_S3_AUTO_RETURN;
            kart_mission_set_stage((uint8)KART_S3_RETURNING);
            return 1U;
        }

        if(event == KART_MISSION_EVENT_S3_RETURN_DONE &&
           kart_mission_status.subject_state == (uint8)KART_S3_RETURNING)
        {
            kart_mission_complete((uint8)KART_S3_FINISHED);
            return 1U;
        }
    }

    kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
    return 0U;
}

uint8 kart_mission_submit_light(kart_light_command_t task)
{
    if(kart_mission_status.run_state != KART_MISSION_RUNNING ||
       kart_mission_status.subject != KART_SUBJECT_2_INTERACTION ||
       kart_mission_status.subject_state != (uint8)KART_S2_WAIT_LIGHT)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    if(task <= KART_LIGHT_CMD_OFF || task >= KART_LIGHT_CMD_COUNT)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_TASK;
        return 0U;
    }

    kart_mission_request.light = task;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_set_stage((uint8)KART_S2_EXEC_LIGHT);
    return 1U;
}

uint8 kart_mission_submit_horn(kart_horn_task_t task)
{
    if(kart_mission_status.run_state != KART_MISSION_RUNNING ||
       kart_mission_status.subject != KART_SUBJECT_2_INTERACTION ||
       kart_mission_status.subject_state != (uint8)KART_S2_WAIT_HORN)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    if(task <= KART_HORN_NONE || task >= KART_HORN_COUNT)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_TASK;
        return 0U;
    }

    kart_mission_request.horn = task;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_set_stage((uint8)KART_S2_EXEC_HORN);
    return 1U;
}

uint8 kart_mission_submit_gate(kart_gate_task_t task)
{
    uint8 is_out_task = (uint8)(task >= KART_GATE_OUT_1_LEFT &&
                                task <= KART_GATE_OUT_3_RIGHT);
    uint8 is_back_task = (uint8)(task >= KART_GATE_BACK_1_RIGHT &&
                                 task <= KART_GATE_BACK_3_LEFT);

    if(kart_mission_status.run_state != KART_MISSION_RUNNING ||
       kart_mission_status.subject != KART_SUBJECT_2_INTERACTION)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    if(kart_mission_status.subject_state == (uint8)KART_S2_WAIT_OUT_GATE && is_out_task != 0U)
    {
        kart_mission_request.gate = task;
        kart_mission_request.motion = KART_REQUEST_S2_GATE;
        kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
        kart_mission_set_stage((uint8)KART_S2_EXEC_OUT_GATE);
        return 1U;
    }

    if(kart_mission_status.subject_state == (uint8)KART_S2_WAIT_BACK_GATE && is_back_task != 0U)
    {
        kart_mission_request.gate = task;
        kart_mission_request.motion = KART_REQUEST_S2_GATE;
        kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
        kart_mission_set_stage((uint8)KART_S2_EXEC_BACK_GATE);
        return 1U;
    }

    kart_mission_status.last_error = (is_out_task == 0U && is_back_task == 0U)
        ? KART_MISSION_ERROR_BAD_TASK
        : KART_MISSION_ERROR_BAD_STATE;
    return 0U;
}

uint8 kart_mission_submit_motion(kart_motion_task_t task)
{
    if(kart_mission_status.run_state != KART_MISSION_RUNNING ||
       kart_mission_status.subject != KART_SUBJECT_2_INTERACTION ||
       kart_mission_status.subject_state != (uint8)KART_S2_WAIT_MOTION)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_STATE;
        return 0U;
    }

    if(task <= KART_MOTION_NONE || task >= KART_MOTION_COUNT)
    {
        kart_mission_status.last_error = KART_MISSION_ERROR_BAD_TASK;
        return 0U;
    }

    kart_mission_request.motion_task = task;
    kart_mission_request.motion = KART_REQUEST_S2_MOTION;
    kart_mission_status.last_error = KART_MISSION_ERROR_NONE;
    kart_mission_set_stage((uint8)KART_S2_EXEC_MOTION);
    return 1U;
}

void kart_mission_get_status(kart_mission_status_t *status)
{
    if(status != NULL)
    {
        *status = kart_mission_status;
    }
}

void kart_mission_get_request(kart_mission_request_t *request)
{
    if(request != NULL)
    {
        *request = kart_mission_request;
    }
}

uint8 kart_mission_stop_is_required(void)
{
    /* ARM、等待口令、完成、中止和故障状态都持续压住停机，直到回到 IDLE。 */
    return (uint8)(kart_mission_status.run_state != KART_MISSION_IDLE &&
                   kart_mission_request.motion == KART_REQUEST_STOP);
}

const char *kart_mission_subject_name(kart_subject_t subject)
{
    switch(subject)
    {
        case KART_SUBJECT_1_AUTO_DRIVE:  return "S1 AUTO DRIVE";
        case KART_SUBJECT_2_INTERACTION: return "S2 INTERACT";
        case KART_SUBJECT_3_PATH_RETURN: return "S3 PATH RETURN";
        default:                         return "NO SUBJECT";
    }
}

const char *kart_mission_run_name(kart_mission_run_state_t state)
{
    switch(state)
    {
        case KART_MISSION_IDLE:     return "IDLE";
        case KART_MISSION_ARMED:    return "ARMED";
        case KART_MISSION_RUNNING:  return "RUNNING";
        case KART_MISSION_FINISHED: return "FINISHED";
        case KART_MISSION_ABORTED:  return "ABORTED";
        case KART_MISSION_FAULT:    return "FAULT";
        default:                    return "UNKNOWN";
    }
}

const char *kart_mission_stage_name(void)
{
    if(kart_mission_status.subject == KART_SUBJECT_1_AUTO_DRIVE)
    {
        switch((kart_subject1_state_t)kart_mission_status.subject_state)
        {
            case KART_S1_WAIT_START:       return "WAIT START";
            case KART_S1_LEAVE_START:      return "LEAVE START";
            case KART_S1_SLALOM:           return "SLALOM";
            case KART_S1_APPROACH_GARAGE:  return "TO GARAGE";
            case KART_S1_REVERSE_ALIGN:    return "REV ALIGN";
            case KART_S1_REVERSE_PARK:     return "REV PARK";
            case KART_S1_FINISHED:         return "DONE";
            default:                       return "DISABLED";
        }
    }

    if(kart_mission_status.subject == KART_SUBJECT_2_INTERACTION)
    {
        switch((kart_subject2_state_t)kart_mission_status.subject_state)
        {
            case KART_S2_WAIT_LIGHT:      return "WAIT LIGHT";
            case KART_S2_EXEC_LIGHT:      return "EXEC LIGHT";
            case KART_S2_WAIT_HORN:       return "WAIT HORN";
            case KART_S2_EXEC_HORN:       return "EXEC HORN";
            case KART_S2_WAIT_OUT_GATE:   return "WAIT OUT GATE";
            case KART_S2_EXEC_OUT_GATE:   return "EXEC OUT GATE";
            case KART_S2_WAIT_MOTION:     return "WAIT MOTION";
            case KART_S2_EXEC_MOTION:     return "EXEC MOTION";
            case KART_S2_WAIT_BACK_GATE:  return "WAIT BACK GATE";
            case KART_S2_EXEC_BACK_GATE:  return "EXEC BACK GATE";
            case KART_S2_FINISHED:        return "DONE 8/8";
            default:                      return "DISABLED";
        }
    }

    if(kart_mission_status.subject == KART_SUBJECT_3_PATH_RETURN)
    {
        switch((kart_subject3_state_t)kart_mission_status.subject_state)
        {
            case KART_S3_WAIT_RECORD:  return "WAIT RECORD";
            case KART_S3_RECORDING:    return "RECORDING";
            case KART_S3_READY_RETURN: return "READY RETURN";
            case KART_S3_RETURNING:    return "RETURNING";
            case KART_S3_FINISHED:     return "DONE";
            default:                   return "DISABLED";
        }
    }

    return "DISABLED";
}
