#include "kart_steer_abs.h"

static uint16 kart_steer_abs_raw = 0;
static uint16 kart_steer_abs_frame = 0;
static int16 kart_steer_abs_delta = 0;
static int16 kart_steer_abs_deg_x100 = 0;

#define KART_STEER_ABS_SPI_W            (0x80)
#define KART_STEER_ABS_SPI_R            (0x40)
#define KART_STEER_ABS_ZERO_L_REG       (0x00)
#define KART_STEER_ABS_ZERO_H_REG       (0x01)
#define KART_STEER_ABS_DIR_REG          (0x09)
#define KART_STEER_ABS_STATUS_REG       (0x06)
#define KART_STEER_ABS_TIMEOUT_COUNT    (100)

/* 把 raw - center 折回 ±2048。为什么非折不可:中位贴着 raw 0(当前 164),
 * 过零时 raw 在 0 与 4095 之间跳,不折就会读出 ±4000 的假打角,而软限位是拿
 * 这个值比的 —— 那一下就是打死(kart_calib.h 第四节记了这件事)。
 * 【注意】2048 / 4096 是写死的,没用 kart_calib.h 的 KART_STEER_ABS_RAW_FULL。
 * 哪天换成不是 12 位的编码器,这里两个数要跟着改,光改那个宏不够。 */
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

/* 上电自检:把 6 个配置寄存器反复写一遍,直到状态寄存器读回 0x1C 才算好,
 * 超过 100 轮放弃并返回 1。那组寄存器地址和值来自厂商例程,本工程没有这颗
 * 芯片的手册,不要凭猜改数。
 * 【已知缺口】init() 里自检失败只是跳过零位与方向配置,既不报警、也不置故障位、
 * 也不拦着后面出数 —— 现象是转向角看着有值但不可信,而转向角是软限位唯一
 * 依据。要补就在 init() 里把返回值存成模块状态,由菜单或 VOFA 露出来。 */
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

void kart_steer_abs_init(void)
{
    gpio_init(KART_STEER_ABS_CS_GPIO_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    spi_init(KART_STEER_ABS_SPI_INDEX,
             KART_STEER_ABS_SPI_MODE,
             KART_STEER_ABS_SPI_BAUD,
             KART_STEER_ABS_SPI_SCK_PIN,
             KART_STEER_ABS_SPI_MOSI_PIN,
             KART_STEER_ABS_SPI_MISO_PIN,
             KART_STEER_ABS_SPI_HW_CS_PIN);

    if(0 == kart_steer_abs_self_check())
    {
        /* 芯片零位写 0:不把零点烧进编码器,中位放在软件里
         * (kart_calib.h 的 CENTER_RAW,当前 164)。这样换齿轮重标只改一个宏,
         * 不用重新配芯片;代价是中位贴着 raw 0,过零环绕交给 wrap_delta 处理。 */
        uint16 zero_position = 0;
        kart_steer_abs_write_register(KART_STEER_ABS_DIR_REG, 0x00);
        kart_steer_abs_write_register(KART_STEER_ABS_ZERO_L_REG, (uint8)zero_position);
        kart_steer_abs_write_register(KART_STEER_ABS_ZERO_H_REG, (uint8)(zero_position >> 8));
    }

    kart_steer_abs_update();
}

void kart_steer_abs_update(void)
{
    kart_steer_abs_frame = kart_steer_abs_read_frame();
    kart_steer_abs_raw = (uint16)((kart_steer_abs_frame >> KART_STEER_ABS_RAW_SHIFT) & 0x0FFF);
    kart_steer_abs_delta = kart_steer_abs_wrap_delta(kart_steer_abs_raw, KART_STEER_ABS_CENTER_RAW);
    /* 36000 = 360.00 度 × 100。先乘后除并走 int32:反过来先除会把 12 位量
     * 的精度丢光。 */
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
