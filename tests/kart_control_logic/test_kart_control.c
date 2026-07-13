#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "kart_control.h"

static int16 test_left_delta;
static int16 test_right_delta;
static int16 test_left_duty;
static int16 test_right_duty;
static uint32 test_lock_depth;

uint32 interrupt_global_disable(void)
{
    uint32 was_disabled = (test_lock_depth != 0U) ? 1U : 0U;
    test_lock_depth++;
    return was_disabled;
}

void interrupt_global_enable(uint32 primask)
{
    assert(test_lock_depth > 0U);
    test_lock_depth--;
    if(primask != 0U)
    {
        assert(test_lock_depth > 0U);
    }
}

void kart_encoder_update(void)
{
}

int16 kart_encoder_get_left_delta(void)
{
    return test_left_delta;
}

int16 kart_encoder_get_right_delta(void)
{
    return test_right_delta;
}

void power_set_rear_duty(int16 left_duty, int16 right_duty)
{
    test_left_duty = left_duty;
    test_right_duty = right_duty;
}

static void test_init_and_immediate_disable(void)
{
    float kp;
    float ki;
    float kd;

    kart_control_init();
    assert(kart_control_is_enabled() == 0U);
    assert(kart_control_get_output() == 0);

    kart_control_get_pid(&kp, &ki, &kd);
    assert(fabsf(kp - 10.0f) < 0.0001f);
    assert(fabsf(ki) < 0.0001f);
    assert(fabsf(kd) < 0.0001f);

    test_left_delta = 5;
    test_right_delta = 7;
    test_left_duty = 1234;
    test_right_duty = 1234;
    kart_control_speed_update();
    assert(test_left_duty == 0);
    assert(test_right_duty == 0);

    kart_control_set_enable(1U);
    kart_control_set_target(10.0f);
    kart_control_speed_update();
    assert(kart_control_is_enabled() == 1U);
    assert(kart_control_get_output() > 0);

    kart_control_set_enable(0U);
    assert(kart_control_is_enabled() == 0U);
    assert(kart_control_get_output() == 0);
    assert(test_left_duty == 0);
    assert(test_right_duty == 0);
}

static void test_pid_snapshot_api(void)
{
    float kp;
    float ki;
    float kd;

    kart_control_set_pid(12.5f, 0.75f, 3.0f);
    kart_control_get_pid(&kp, &ki, &kd);
    assert(fabsf(kp - 12.5f) < 0.0001f);
    assert(fabsf(ki - 0.75f) < 0.0001f);
    assert(fabsf(kd - 3.0f) < 0.0001f);
    assert(test_lock_depth == 0U);
}

int main(void)
{
    test_init_and_immediate_disable();
    test_pid_snapshot_api();
    puts("kart_control logic tests passed");
    return 0;
}
