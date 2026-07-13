#include <assert.h>
#include <stdio.h>

#include "kart_light.h"

static const uint16 frame_blank[KART_LIGHT_ROW_NUM] =
{
    0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U
};

static const uint16 frame_high_beam[KART_LIGHT_ROW_NUM] =
{
    0x47EEU, 0x4491U, 0x4490U, 0x7C97U, 0x4491U, 0x4491U, 0x47EEU
};

static const uint16 frame_low_beam[KART_LIGHT_ROW_NUM] =
{
    0x41D1U, 0x4231U, 0x4231U, 0x4235U, 0x4235U, 0x4235U, 0x7DCAU
};

static const uint16 frame_fog[KART_LIGHT_ROW_NUM] =
{
    0x7DCEU, 0x4231U, 0x4230U, 0x7A37U, 0x4231U, 0x4231U, 0x41CEU
};

static const uint16 frame_hazard[KART_LIGHT_ROW_NUM] =
{
    0x1084U, 0x1084U, 0x1084U, 0x1084U, 0x1084U, 0x0000U, 0x1084U
};

static const uint16 frame_cabin[KART_LIGHT_ROW_NUM] =
{
    0x39DEU, 0x4631U, 0x4231U, 0x43FEU, 0x4231U, 0x4631U, 0x3A3EU
};

static const uint16 frame_left[KART_LIGHT_ROW_NUM] =
{
    0x0008U, 0x0004U, 0x0002U, 0x0001U, 0x0002U, 0x0004U, 0x0008U
};

static const uint16 frame_left_2[KART_LIGHT_ROW_NUM] =
{
    0x0108U, 0x0084U, 0x0042U, 0x0021U, 0x0042U, 0x0084U, 0x0108U
};

static const uint16 frame_left_3[KART_LIGHT_ROW_NUM] =
{
    0x2108U, 0x1084U, 0x0842U, 0x0421U, 0x0842U, 0x1084U, 0x2108U
};

static const uint16 frame_right[KART_LIGHT_ROW_NUM] =
{
    0x0800U, 0x1000U, 0x2000U, 0x4000U, 0x2000U, 0x1000U, 0x0800U
};

static const uint16 frame_right_2[KART_LIGHT_ROW_NUM] =
{
    0x0840U, 0x1080U, 0x2100U, 0x4200U, 0x2100U, 0x1080U, 0x0840U
};

static const uint16 frame_right_3[KART_LIGHT_ROW_NUM] =
{
    0x0842U, 0x1084U, 0x2108U, 0x4210U, 0x2108U, 0x1084U, 0x0842U
};

static const uint16 frame_wiper_left[KART_LIGHT_ROW_NUM] =
{
    0x4210U, 0x2108U, 0x2108U, 0x1084U, 0x0842U, 0x0842U, 0x0421U
};

static const uint16 frame_wiper_center[KART_LIGHT_ROW_NUM] =
{
    0x1084U, 0x1084U, 0x1084U, 0x1084U, 0x1084U, 0x1084U, 0x1084U
};

static const uint16 frame_wiper_right[KART_LIGHT_ROW_NUM] =
{
    0x0421U, 0x0842U, 0x0842U, 0x1084U, 0x2108U, 0x2108U, 0x4210U
};

static void assert_frame(const uint16 expected[KART_LIGHT_ROW_NUM])
{
    uint16 copied[KART_LIGHT_ROW_NUM];
    uint8 row;

    kart_light_copy_frame(copied);
    for(row = 0; row < KART_LIGHT_ROW_NUM; row++)
    {
        assert(kart_light_get_row(row) == expected[row]);
        assert(copied[row] == expected[row]);
    }
}

static void test_static_commands(void)
{
    kart_light_set_command(KART_LIGHT_CMD_HIGH_BEAM);
    assert_frame(frame_high_beam);

    kart_light_set_command(KART_LIGHT_CMD_LOW_BEAM);
    assert_frame(frame_low_beam);

    kart_light_set_command(KART_LIGHT_CMD_FOG);
    assert_frame(frame_fog);

    kart_light_set_command(KART_LIGHT_CMD_CABIN);
    assert_frame(frame_cabin);

    kart_light_update(5000U);
    assert_frame(frame_cabin);
}

static void test_turn_animations(void)
{
    kart_light_set_command(KART_LIGHT_CMD_LEFT_TURN);
    assert_frame(frame_left);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_left_2);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_left_3);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_blank);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_left);

    kart_light_set_command(KART_LIGHT_CMD_RIGHT_TURN);
    assert_frame(frame_right);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_right_2);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_right_3);
    kart_light_update(KART_LIGHT_TURN_STEP_MS);
    assert_frame(frame_blank);

    /* 一次跨过五个周期，应直接落在相位 0，而不是只推进一帧。 */
    kart_light_update(KART_LIGHT_TURN_STEP_MS * 5U);
    assert_frame(frame_right);
}

static void test_hazard_and_wiper(void)
{
    kart_light_set_command(KART_LIGHT_CMD_HAZARD);
    assert_frame(frame_hazard);
    kart_light_update(KART_LIGHT_HAZARD_STEP_MS - 1U);
    assert_frame(frame_hazard);
    kart_light_update(1U);
    assert_frame(frame_blank);
    kart_light_update(KART_LIGHT_HAZARD_STEP_MS);
    assert_frame(frame_hazard);

    kart_light_set_command(KART_LIGHT_CMD_WIPER);
    assert_frame(frame_wiper_left);
    kart_light_update(KART_LIGHT_WIPER_STEP_MS);
    assert_frame(frame_wiper_center);
    kart_light_update(KART_LIGHT_WIPER_STEP_MS);
    assert_frame(frame_wiper_right);
    kart_light_update(KART_LIGHT_WIPER_STEP_MS);
    assert_frame(frame_wiper_center);
    kart_light_update(KART_LIGHT_WIPER_STEP_MS);
    assert_frame(frame_wiper_left);
}

int main(void)
{
    kart_light_init();
    assert(kart_light_get_command() == KART_LIGHT_CMD_OFF);
    assert(kart_light_get_brightness() == KART_LIGHT_DEFAULT_BRIGHTNESS);
    assert_frame(frame_blank);

    kart_light_set_brightness(0U);
    assert(kart_light_get_brightness() == 0U);
    kart_light_set_brightness(12000U);
    assert(kart_light_get_brightness() == KART_LIGHT_MAX_BRIGHTNESS);

    test_static_commands();
    test_turn_animations();
    test_hazard_and_wiper();

    kart_light_set_command((kart_light_command_t)-1);
    assert(kart_light_get_command() == KART_LIGHT_CMD_OFF);
    assert_frame(frame_blank);

    kart_light_set_command((kart_light_command_t)255);
    assert(kart_light_get_command() == KART_LIGHT_CMD_OFF);
    assert_frame(frame_blank);

    assert(kart_light_get_row(KART_LIGHT_ROW_NUM) == 0U);
    kart_light_copy_frame(NULL);

    puts("kart_light logic tests passed");
    return 0;
}
