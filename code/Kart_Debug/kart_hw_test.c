#include "kart_hw_test.h"
#include "zf_device_ips200.h"

/*
 * 硬件自测实现 —— 见 kart_hw_test.h 头注释。
 * 旋钮用软件正交解码(P11.2/3 不在硬件编码器定时器候选,只能软件读)。
 * 按键上拉输入,按下接地读 0。
 */

static int32 knob_count = 0;
static uint8 knob_a_last = 1;
static uint8 knob_b_last = 1;

/* 初始化旋钮 + 按键为输入 */
static void hw_test_gpio_init(void)
{
    /* 上拉在主板侧,按键板公共端接 GND(按下读 0),故一律浮空输入,不叠片内上拉。
     * 与 kart_menu_init 保持同一配置,免得自测通过、正式跑却读不到。 */
    gpio_init(KART_KNOB_A_PIN,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_KNOB_B_PIN,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_KNOB_SW_PIN, GPI, 0, GPI_FLOATING_IN);

    gpio_init(KART_KEY_UP_PIN,    GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_KEY_DOWN_PIN,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_KEY_LEFT_PIN,  GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_KEY_RIGHT_PIN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KART_KEY_MID_PIN,   GPI, 0, GPI_FLOATING_IN);
    gpio_init(KEY_START_PIN, GPI, 0, GPI_FLOATING_IN);

    knob_a_last = gpio_get_level(KART_KNOB_A_PIN);
    knob_b_last = gpio_get_level(KART_KNOB_B_PIN);
}

/* 旋钮软件解码:AB 双相 4 倍频解码（最简单可靠）
 * 检测 AB 任意一相变化，根据另一相当前状态判断方向
 * 顺时针：A 上升 B=0, A 下降 B=1, B 上升 A=1, B 下降 A=0
 * 逆时针：A 上升 B=1, A 下降 B=0, B 上升 A=0, B 下降 A=1 */
static void hw_test_knob_scan(void)
{
    uint8 a = gpio_get_level(KART_KNOB_A_PIN);
    uint8 b = gpio_get_level(KART_KNOB_B_PIN);

    /* A 相变化 */
    if(a != knob_a_last)
    {
        if(a == 1 && b == 0)      knob_count++;  /* A 上升 B=0 → 顺时针 */
        else if(a == 0 && b == 1) knob_count++;  /* A 下降 B=1 → 顺时针 */
        else if(a == 1 && b == 1) knob_count--;  /* A 上升 B=1 → 逆时针 */
        else if(a == 0 && b == 0) knob_count--;  /* A 下降 B=0 → 逆时针 */
        knob_a_last = a;
    }

    /* B 相变化 */
    if(b != knob_b_last)
    {
        if(b == 1 && a == 1)      knob_count++;  /* B 上升 A=1 → 顺时针 */
        else if(b == 0 && a == 0) knob_count++;  /* B 下降 A=0 → 顺时针 */
        else if(b == 1 && a == 0) knob_count--;  /* B 上升 A=0 → 逆时针 */
        else if(b == 0 && a == 1) knob_count--;  /* B 下降 A=1 → 逆时针 */
        knob_b_last = b;
    }
}

/* 屏上刷三色验证无花点(每色 500ms) */
static void hw_test_color_sweep(void)
{
    ips200_full(RGB565_RED);
    system_delay_ms(500);
    ips200_full(RGB565_GREEN);
    system_delay_ms(500);
    ips200_full(RGB565_BLUE);
    system_delay_ms(500);
    ips200_clear();
}

void kart_hw_test_run(void)
{
    hw_test_gpio_init();

    /* SPI 方式初始化 IPS200(屏幕丝印 BLK/CS/DC/RST/SDA/SCL，不是并口)
     * 引脚见 zf_device_ips200.h 的 SPI 模式定义 */
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);

    hw_test_color_sweep();

    ips200_show_string(0, 0, "HW TEST");

    while(1)
    {
        hw_test_knob_scan();

        ips200_show_string(0, 32,  "KNOB:");
        ips200_show_int(80, 32, knob_count, 6);

        ips200_show_string(0, 64,  "UP  :");
        ips200_show_int(80, 64, gpio_get_level(KART_KEY_UP_PIN), 2);
        ips200_show_string(0, 80,  "DOWN:");
        ips200_show_int(80, 80, gpio_get_level(KART_KEY_DOWN_PIN), 2);
        ips200_show_string(0, 96,  "KART_LEFT:");
        ips200_show_int(80, 96, gpio_get_level(KART_KEY_LEFT_PIN), 2);
        ips200_show_string(0, 112, "RGHT:");
        ips200_show_int(80, 112, gpio_get_level(KART_KEY_RIGHT_PIN), 2);
        ips200_show_string(0, 128, "MID :");
        ips200_show_int(80, 128, gpio_get_level(KART_KEY_MID_PIN), 2);
        ips200_show_string(0, 144, "ESW :");
        ips200_show_int(80, 144, gpio_get_level(KART_KNOB_SW_PIN), 2);
        ips200_show_string(0, 160, "STRT:");
        ips200_show_int(80, 160, gpio_get_level(KEY_START_PIN), 2);

        system_delay_ms(2);         /* 2ms 轮询,够软件读旋钮 */
    }
}
