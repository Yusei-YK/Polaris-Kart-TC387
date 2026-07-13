#include "kart_steer_abs.h"

static uint16 kart_steer_abs_raw = 0;
static uint16 kart_steer_abs_frame = 0;
static int16 kart_steer_abs_delta = 0;
static int16 kart_steer_abs_deg_x100 = 0;
static uint8 kart_steer_abs_ready = 0;

#define KART_STEER_ABS_SPI_W            (0x80)
#define KART_STEER_ABS_SPI_R            (0x40)
#define KART_STEER_ABS_ZERO_L_REG       (0x00)
#define KART_STEER_ABS_ZERO_H_REG       (0x01)
#define KART_STEER_ABS_DIR_REG          (0x09)
#define KART_STEER_ABS_STATUS_REG       (0x06)
#define KART_STEER_ABS_TIMEOUT_COUNT    (100)

static int16 kart_steer_abs_wrap_delta(uint16 raw, uint16 center)
{
    int16 delta = (int16)raw - (int16)center;

    if(delta > 2048)
    {
        delta -= 4096;
    }
    else if(delta < -2048)
    {
        delta += 4096;
    }

    return delta;
}

static void kart_steer_abs_cs(uint8 state)
{
    if(state)
    {
        gpio_high(KART_STEER_ABS_CS_GPIO_PIN);
    }
    else
    {
        gpio_low(KART_STEER_ABS_CS_GPIO_PIN);
    }
}

static void kart_steer_abs_write_register(uint8 reg, uint8 data)
{
    kart_steer_abs_cs(0);
    spi_write_8bit(KART_STEER_ABS_SPI_INDEX, (uint8)(reg | KART_STEER_ABS_SPI_W));
    spi_write_8bit(KART_STEER_ABS_SPI_INDEX, data);
    kart_steer_abs_cs(1);
    system_delay_us(1);

    kart_steer_abs_cs(0);
    spi_read_8bit(KART_STEER_ABS_SPI_INDEX);
    spi_read_8bit(KART_STEER_ABS_SPI_INDEX);
    kart_steer_abs_cs(1);
}

static uint8 kart_steer_abs_read_register(uint8 reg)
{
    uint8 data;

    kart_steer_abs_cs(0);
    spi_write_8bit(KART_STEER_ABS_SPI_INDEX, (uint8)(reg | KART_STEER_ABS_SPI_R));
    spi_write_8bit(KART_STEER_ABS_SPI_INDEX, 0x00);
    kart_steer_abs_cs(1);
    system_delay_us(1);

    kart_steer_abs_cs(0);
    data = spi_read_8bit(KART_STEER_ABS_SPI_INDEX);
    spi_read_8bit(KART_STEER_ABS_SPI_INDEX);
    kart_steer_abs_cs(1);

    return data;
}

static uint8 kart_steer_abs_self_check(void)
{
    uint8 i;
    uint16 time_count = 0;
    const uint8 dat[6] = {0, 0, 0, 0xC0, 0xFF, 0x1C};

    while(0x1C != kart_steer_abs_read_register(KART_STEER_ABS_STATUS_REG))
    {
        for(i = 0; i < 6; i++)
        {
            kart_steer_abs_write_register((uint8)(i + 1), dat[i]);
            system_delay_ms(1);
        }

        if(KART_STEER_ABS_TIMEOUT_COUNT < time_count++)
        {
            return 1;
        }
    }

    return 0;
}

static uint16 kart_steer_abs_read_frame(void)
{
    uint16 data;

    kart_steer_abs_cs(0);
    data = ((uint16)spi_read_8bit(KART_STEER_ABS_SPI_INDEX) & 0x00FF) << 8;
    data |= spi_read_8bit(KART_STEER_ABS_SPI_INDEX);
    kart_steer_abs_cs(1);

    return data;
}

uint8 kart_steer_abs_init(void)
{
    kart_steer_abs_ready = 0;
    gpio_init(KART_STEER_ABS_CS_GPIO_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    spi_init(KART_STEER_ABS_SPI_INDEX,
             KART_STEER_ABS_SPI_MODE,
             KART_STEER_ABS_SPI_BAUD,
             KART_STEER_ABS_SPI_SCK_PIN,
             KART_STEER_ABS_SPI_MOSI_PIN,
             KART_STEER_ABS_SPI_MISO_PIN,
             KART_STEER_ABS_SPI_HW_CS_PIN);

    if(0 != kart_steer_abs_self_check())
    {
        /* 自检失败时不能继续把随机 SPI 数据当成转角反馈。 */
        return 1;
    }

    {
        uint16 zero_position = 0;
        kart_steer_abs_write_register(KART_STEER_ABS_DIR_REG, 0x00);
        kart_steer_abs_write_register(KART_STEER_ABS_ZERO_L_REG, (uint8)zero_position);
        kart_steer_abs_write_register(KART_STEER_ABS_ZERO_H_REG, (uint8)(zero_position >> 8));
    }

    kart_steer_abs_ready = 1;
    kart_steer_abs_update();
    return 0;
}

void kart_steer_abs_update(void)
{
    if(0 == kart_steer_abs_ready)
    {
        return;
    }

    kart_steer_abs_frame = kart_steer_abs_read_frame();
    kart_steer_abs_raw = (uint16)((kart_steer_abs_frame >> KART_STEER_ABS_RAW_SHIFT) & 0x0FFF);
    kart_steer_abs_delta = kart_steer_abs_wrap_delta(kart_steer_abs_raw, KART_STEER_ABS_CENTER_RAW);
    kart_steer_abs_deg_x100 = (int16)(((int32)kart_steer_abs_delta * 36000) / 4096);
}

uint16 kart_steer_abs_get_frame(void)
{
    return kart_steer_abs_frame;
}

uint16 kart_steer_abs_get_raw(void)
{
    return kart_steer_abs_raw;
}

int16 kart_steer_abs_get_center_delta(void)
{
    return kart_steer_abs_delta;
}

int16 kart_steer_abs_get_deg_x100(void)
{
    return kart_steer_abs_deg_x100;
}

uint8 kart_steer_abs_is_ready(void)
{
    return kart_steer_abs_ready;
}
