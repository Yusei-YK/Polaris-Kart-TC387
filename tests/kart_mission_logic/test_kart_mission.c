#include <assert.h>
#include <stdio.h>

#include "kart_mission.h"

static kart_mission_status_t get_status(void)
{
    kart_mission_status_t status;
    kart_mission_get_status(&status);
    return status;
}

static kart_mission_request_t get_request(void)
{
    kart_mission_request_t request;
    kart_mission_get_request(&request);
    return request;
}

static void test_readiness_gate_and_subject1(void)
{
    kart_mission_status_t status;
    kart_mission_request_t request;

    kart_mission_init();
    assert(kart_mission_select_subject(KART_SUBJECT_1_AUTO_DRIVE) == 1U);
    assert(kart_mission_arm() == 0U);
    status = get_status();
    assert(status.run_state == KART_MISSION_IDLE);
    assert(status.last_error == KART_MISSION_ERROR_NOT_READY);
    assert(status.missing_mask == KART_MISSION_S1_REQUIRED);

    kart_mission_set_ready_mask(KART_MISSION_S1_REQUIRED);
    assert(kart_mission_arm() == 1U);
    assert(kart_mission_stop_is_required() == 1U);
    status = get_status();
    assert(status.run_state == KART_MISSION_ARMED);
    assert(status.subject_state == (uint8)KART_S1_WAIT_START);

    assert(kart_mission_start() == 1U);
    assert(kart_mission_stop_is_required() == 0U);
    request = get_request();
    assert(request.motion == KART_REQUEST_S1_FORWARD_PATH);

    kart_mission_tick(5U);
    kart_mission_tick(15U);
    status = get_status();
    assert(status.total_elapsed_ms == 20U);
    assert(status.state_elapsed_ms == 20U);

    assert(kart_mission_signal(KART_MISSION_EVENT_S1_SLALOM_DONE) == 0U);
    assert(kart_mission_signal(KART_MISSION_EVENT_S1_ENTER_SLALOM) == 1U);
    assert(get_status().last_error == KART_MISSION_ERROR_NONE);
    assert(kart_mission_signal(KART_MISSION_EVENT_S1_SLALOM_DONE) == 1U);
    assert(kart_mission_signal(KART_MISSION_EVENT_S1_GARAGE_REACHED) == 1U);
    assert(get_request().motion == KART_REQUEST_S1_REVERSE_ALIGN);
    assert(kart_mission_signal(KART_MISSION_EVENT_S1_ALIGN_DONE) == 1U);
    assert(get_request().motion == KART_REQUEST_S1_REVERSE_PARK);
    assert(kart_mission_signal(KART_MISSION_EVENT_S1_PARK_DONE) == 1U);

    status = get_status();
    assert(status.run_state == KART_MISSION_FINISHED);
    assert(status.subject_state == (uint8)KART_S1_FINISHED);
    assert(get_request().motion == KART_REQUEST_STOP);
    assert(kart_mission_stop_is_required() == 1U);
    assert(kart_mission_stop_is_required() == 1U);
}

static void complete_s2_task(void)
{
    assert(kart_mission_signal(KART_MISSION_EVENT_TASK_DONE) == 1U);
}

static void test_subject2_eight_task_flow(void)
{
    kart_mission_status_t status;

    kart_mission_init();
    kart_mission_set_ready_mask(KART_MISSION_S2_REQUIRED);
    assert(kart_mission_select_subject(KART_SUBJECT_2_INTERACTION) == 1U);
    assert(kart_mission_arm() == 1U);
    assert(kart_mission_stop_is_required() == 1U);
    assert(kart_mission_start() == 1U);

    assert(kart_mission_submit_light(KART_LIGHT_CMD_LEFT_TURN) == 1U);
    assert(get_request().light == KART_LIGHT_CMD_LEFT_TURN);
    complete_s2_task();
    status = get_status();
    assert(status.subject2_tasks_done == 1U);
    assert(status.subject_state == (uint8)KART_S2_WAIT_LIGHT);

    assert(kart_mission_submit_light(KART_LIGHT_CMD_HIGH_BEAM) == 1U);
    complete_s2_task();
    status = get_status();
    assert(status.subject2_tasks_done == 2U);
    assert(status.subject_state == (uint8)KART_S2_WAIT_HORN);

    assert(kart_mission_submit_horn(KART_HORN_1_SECOND) == 1U);
    complete_s2_task();
    assert(kart_mission_submit_horn(KART_HORN_ALARM) == 1U);
    complete_s2_task();
    status = get_status();
    assert(status.subject2_tasks_done == 4U);
    assert(status.subject_state == (uint8)KART_S2_WAIT_OUT_GATE);

    /* 去程阶段不能误接收返程门洞命令。 */
    assert(kart_mission_submit_gate(KART_GATE_BACK_1) == 0U);
    assert(kart_mission_submit_gate(KART_GATE_OUT_2) == 1U);
    assert(get_request().motion == KART_REQUEST_S2_GATE);
    complete_s2_task();
    status = get_status();
    assert(status.subject2_tasks_done == 5U);
    assert(status.subject_state == (uint8)KART_S2_WAIT_MOTION);

    assert(kart_mission_submit_motion(KART_MOTION_FORWARD_DISTANCE) == 1U);
    complete_s2_task();
    assert(kart_mission_submit_motion(KART_MOTION_TURN_LEFT) == 1U);
    complete_s2_task();
    status = get_status();
    assert(status.subject2_tasks_done == 7U);
    assert(status.subject_state == (uint8)KART_S2_WAIT_BACK_GATE);

    /* 返程阶段同样不能误接收去程门洞命令。 */
    assert(kart_mission_submit_gate(KART_GATE_OUT_3) == 0U);
    assert(kart_mission_submit_gate(KART_GATE_BACK_3_LEFT) == 1U);
    complete_s2_task();

    status = get_status();
    assert(status.subject2_tasks_done == KART_MISSION_S2_TOTAL_TASK_COUNT);
    assert(status.subject_state == (uint8)KART_S2_FINISHED);
    assert(status.run_state == KART_MISSION_FINISHED);
    assert(kart_mission_stop_is_required() == 1U);
}

static void test_subject3_and_ready_lost(void)
{
    kart_mission_status_t status;

    kart_mission_init();
    kart_mission_set_ready_mask(KART_MISSION_S3_REQUIRED);
    assert(kart_mission_select_subject(KART_SUBJECT_3_PATH_RETURN) == 1U);
    assert(kart_mission_arm() == 1U);
    assert(kart_mission_stop_is_required() == 1U);
    assert(kart_mission_start() == 1U);
    assert(get_request().motion == KART_REQUEST_S3_RECORD_PATH);
    assert(kart_mission_signal(KART_MISSION_EVENT_S3_RECORD_DONE) == 1U);
    assert(get_request().motion == KART_REQUEST_STOP);
    assert(kart_mission_signal(KART_MISSION_EVENT_S3_RETURN_START) == 1U);
    assert(get_request().motion == KART_REQUEST_S3_AUTO_RETURN);
    assert(kart_mission_signal(KART_MISSION_EVENT_S3_RETURN_DONE) == 1U);
    assert(get_status().run_state == KART_MISSION_FINISHED);

    kart_mission_init();
    kart_mission_set_ready_mask(KART_MISSION_S1_REQUIRED);
    assert(kart_mission_select_subject(KART_SUBJECT_1_AUTO_DRIVE) == 1U);
    assert(kart_mission_arm() == 1U);
    assert(kart_mission_stop_is_required() == 1U);
    assert(kart_mission_start() == 1U);
    kart_mission_set_ready(KART_MISSION_READY_STEER_CTRL, 0U);
    kart_mission_tick(5U);
    kart_mission_poll();
    status = get_status();
    assert(status.run_state == KART_MISSION_FAULT);
    assert(status.last_error == KART_MISSION_ERROR_READY_LOST);
    assert(get_request().motion == KART_REQUEST_STOP);
    assert(kart_mission_stop_is_required() == 1U);
}

int main(void)
{
    test_readiness_gate_and_subject1();
    test_subject2_eight_task_flow();
    test_subject3_and_ready_lost();

    puts("kart_mission logic tests passed");
    return 0;
}
