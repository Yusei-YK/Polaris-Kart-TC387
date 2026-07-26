/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_dot_matrix_screen
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.12
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-22       Seekfree            first version
********************************************************************************************************************/
/********************************************************************************************************************
* ���߶��壺
*                  ------------------------------------
*                  TLD7002����ģ��      ��Ƭ���ܽ�
*                  SR0                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR0_PIN�궨��
*                  SR1                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR1_PIN�궨��
*                  SR2                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR2_PIN�궨��
*                  SR3                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR3_PIN�궨��
*                  SR4                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR4_PIN�궨��
*                  SR5                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR5_PIN�궨��
*                  SR6                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR6_PIN�궨��
*                  SYNC                �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SYNC_PIN�궨��
*                  GND                 ��Դ��
*                  ------------------------------------
********************************************************************************************************************/

#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_exti.h"
#include "zf_driver_uart.h"
#include "zf_device_tld7002.h"
#include "board_pins.h"
#include "isr.h"                /* g_dot_sync_edges:SYNC 边沿累计,全亮自检要读 */

/* 74HC238 行地址设置:A0/A1/A2 = row 低三位(内联,避免每处重复三行) */
#define DOT_MATRIX_SET_ROW_ADDR(row)                                                                    \
    do {                                                                                                \
        if((row) & 0x01) gpio_high(DOT_MATRIX_SCREEN_ROW_A0_PIN); else gpio_low(DOT_MATRIX_SCREEN_ROW_A0_PIN); \
        if((row) & 0x02) gpio_high(DOT_MATRIX_SCREEN_ROW_A1_PIN); else gpio_low(DOT_MATRIX_SCREEN_ROW_A1_PIN); \
        if((row) & 0x04) gpio_high(DOT_MATRIX_SCREEN_ROW_A2_PIN); else gpio_low(DOT_MATRIX_SCREEN_ROW_A2_PIN); \
    } while(0)

#if defined(__TASKING__)
#pragma section all "cpu3_dsram"
#endif

#include "zf_device_dot_matrix_screen.h"


// 卡丁车灯板使用74HC238译码器(输出高有效),用A0/A1/A2三位二进制选择7行(000~110),EN(E3)高有效使能;禁用时全部输出拉低=消隐

/* 初值填空格(0x20):防止首个全帧 memcpy 前 scan() 用零初值算 font[0-' ']=font[-32] 越界读 */
int8    dot_matrix_screen_data[3]        = {' ', ' ', ' '};
uint16  dot_matrix_screen_brightness = 10000;
uint8   dot_matrix_screen_all_on = 0;   // =1 时忽略字库,15 列全亮(标定中指示)

/* =1 时 scan() 立即返回:探针自检函数独占 A0/A1/A2/EN/tld7002_duty,不让中断抢硬件。
 * 注意 EXTI 仍开着 —— isr.c 里 g_dot_sync_edges++ 在调 scan() 之前,故 SYNC 边沿照常累计,
 * 一次烧写同时拿到"静态能不能点亮"和"SYNC 到底几 Hz"两个结论。 */
volatile uint8 dot_matrix_screen_probe = 0;

/* ── 图案帧缓冲(2026-07-26 新增)──
 * dot_fb[row] 的 bit i = 第 i 列(C0..C14)亮灭,bit0=C0=最左。
 * 意义:scan() 不再在中断里查字库、不再只能显 3 个 ASCII 字符 ——
 * 任意 7x15 图案(箭头/雨刷/双闪等)都能画,这是科目二语音控图案的基础。
 * 双缓冲:主循环写 pending 并置 dirty,scan() 只在帧头整帧搬入 active,
 * 避免一帧中途换画面造成撕裂。 */
static volatile uint16 dot_fb_pending[DOT_MATRIX_SCREEN_ROW_NUM] = {0};
static uint16          dot_fb_active [DOT_MATRIX_SCREEN_ROW_NUM] = {0};
static volatile uint8  dot_fb_dirty = 0;

/* 字符串→帧缓冲:在主循环/低优先级上下文做字库解码,不在 ISR 里做。 */
static void dot_matrix_screen_text_to_fb(const int8 *text);


// ������Ļȡģ����
const uint8 tld7002_ascii_font_5x7[][7] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  //    0
    { 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00},  // !  1
    { 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00},  // "  2
    { 0x14, 0x14, 0x3E, 0x14, 0x3E, 0x14, 0x14},  // #  3
    { 0x08, 0x3C, 0x0A, 0x1C, 0x28, 0x1E, 0x08},  // $  4
    { 0x30, 0x32, 0x04, 0x08, 0x10, 0x26, 0x06},  // %  5
    { 0x0C, 0x12, 0x0A, 0x04, 0x2A, 0x12, 0x2C},  // &  6
    { 0x0C, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00},  // '  7
    { 0x10, 0x08, 0x04, 0x04, 0x04, 0x08, 0x10},  // (  8
    { 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, 0x04},  // )  9
    { 0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00},  // * 10
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00},  // + 11
    { 0x00, 0x00, 0x00, 0x00, 0x18, 0x10, 0x08},  // , 12
    { 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00},  // - 13
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},  // . 14
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01},  // / 15
    { 0x1C, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x1C},  // 0 16
    { 0x08, 0x0C, 0x08, 0x08, 0x08, 0x08, 0x1C},  // 1 17
    { 0x1C, 0x22, 0x20, 0x10, 0x08, 0x04, 0x3E},  // 2 18
    { 0x3E, 0x10, 0x08, 0x10, 0x20, 0x22, 0x1C},  // 3 19
    { 0x10, 0x18, 0x14, 0x12, 0x3E, 0x10, 0x10},  // 4 20
    { 0x3E, 0x02, 0x1E, 0x20, 0x20, 0x22, 0x1C},  // 5 21
    { 0x18, 0x04, 0x02, 0x1E, 0x22, 0x22, 0x1C},  // 6 22
    { 0x3E, 0x20, 0x10, 0x08, 0x04, 0x04, 0x04},  // 7 23
    { 0x1C, 0x22, 0x22, 0x1C, 0x22, 0x22, 0x1C},  // 8 24
    { 0x1C, 0x22, 0x22, 0x3C, 0x20, 0x10, 0x0C},  // 9 25
    { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},  // : 26
    { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x08, 0x04},  // ; 27
    { 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10},  // < 28
    { 0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00},  // = 29
    { 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04},  // > 30
    { 0x1C, 0x22, 0x20, 0x10, 0x08, 0x00, 0x08},  // ? 31
    { 0x1C, 0x22, 0x20, 0x2C, 0x3A, 0x22, 0x1C},  // @ 32
    { 0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22},  // A 33
    { 0x1E, 0x22, 0x22, 0x1E, 0x22, 0x22, 0x1E},  // B 34
    { 0x1C, 0x22, 0x02, 0x02, 0x02, 0x22, 0x1C},  // C 35
    { 0x0E, 0x12, 0x22, 0x22, 0x22, 0x12, 0x0E},  // D 36
    { 0x3E, 0x02, 0x02, 0x1E, 0x02, 0x02, 0x3E},  // E 37
    { 0x3E, 0x02, 0x02, 0x1E, 0x02, 0x02, 0x02},  // F 38
    { 0x1C, 0x22, 0x02, 0x3A, 0x22, 0x22, 0x3C},  // G 39
    { 0x22, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22},  // H 40
    { 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C},  // I 41
    { 0x38, 0x10, 0x10, 0x10, 0x10, 0x12, 0x0C},  // J 42
    { 0x22, 0x12, 0x0A, 0x06, 0x0A, 0x12, 0x22},  // K 43
    { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x3E},  // L 44
    { 0x22, 0x36, 0x2A, 0x2A, 0x22, 0x22, 0x22},  // M 45
    { 0x22, 0x22, 0x26, 0x2A, 0x32, 0x22, 0x22},  // N 46
    { 0x1C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C},  // O 47
    { 0x1E, 0x22, 0x22, 0x1E, 0x02, 0x02, 0x02},  // P 48
    { 0x1C, 0x22, 0x22, 0x22, 0x2A, 0x12, 0x2C},  // Q 49
    { 0x1E, 0x22, 0x22, 0x1E, 0x0A, 0x12, 0x22},  // R 50
    { 0x3C, 0x02, 0x02, 0x1C, 0x20, 0x20, 0x1E},  // S 51
    { 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},  // T 52
    { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C},  // U 53
    { 0x22, 0x22, 0x22, 0x22, 0x22, 0x14, 0x08},  // V 54
    { 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x2A, 0x14},  // W 55
    { 0x22, 0x22, 0x14, 0x08, 0x14, 0x22, 0x22},  // X 56
    { 0x22, 0x22, 0x22, 0x14, 0x08, 0x08, 0x08},  // Y 57
    { 0x3E, 0x20, 0x10, 0x08, 0x04, 0x02, 0x3E},  // Z 58
    { 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1C},  // [ 59
    { 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20},  // \ 60
    { 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C},  // ] 61
    { 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00},  // ^ 62
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E},  // _ 63
    { 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00},  // ` 64
    { 0x00, 0x00, 0x1C, 0x20, 0x3C, 0x22, 0x3C},  // a 65
    { 0x02, 0x02, 0x1A, 0x26, 0x22, 0x22, 0x1E},  // b 66
    { 0x00, 0x00, 0x1C, 0x02, 0x02, 0x22, 0x1C},  // c 67
    { 0x20, 0x20, 0x2C, 0x32, 0x22, 0x22, 0x3C},  // d 68
    { 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x02, 0x1C},  // e 69
    { 0x18, 0x24, 0x04, 0x0E, 0x04, 0x04, 0x04},  // f 70
    { 0x00, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x1C},  // g 71
    { 0x02, 0x02, 0x1A, 0x26, 0x22, 0x22, 0x22},  // h 72
    { 0x08, 0x00, 0x0C, 0x08, 0x08, 0x08, 0x1C},  // i 73
    { 0x10, 0x00, 0x18, 0x10, 0x10, 0x12, 0x0C},  // j 74
    { 0x02, 0x02, 0x12, 0x0A, 0x06, 0x0A, 0x12},  // k 75
    { 0x0C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C},  // l 76
    { 0x00, 0x00, 0x16, 0x2A, 0x2A, 0x22, 0x22},  // m 77
    { 0x00, 0x00, 0x1A, 0x26, 0x22, 0x22, 0x22},  // n 78
    { 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C},  // o 79
    { 0x00, 0x1E, 0x22, 0x22, 0x1E, 0x02, 0x02},  // p 80
    { 0x00, 0x2C, 0x32, 0x32, 0x2C, 0x20, 0x20},  // q 81
    { 0x00, 0x00, 0x1A, 0x26, 0x02, 0x02, 0x02},  // r 82
    { 0x00, 0x00, 0x1C, 0x02, 0x1C, 0x20, 0x1E},  // s 83
    { 0x04, 0x04, 0x0E, 0x04, 0x04, 0x24, 0x18},  // t 84
    { 0x00, 0x00, 0x22, 0x22, 0x22, 0x32, 0x2C},  // u 85
    { 0x00, 0x00, 0x22, 0x22, 0x22, 0x14, 0x08},  // v 86
    { 0x00, 0x00, 0x22, 0x22, 0x2A, 0x2A, 0x14},  // w 87
    { 0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22},  // x 88
    { 0x00, 0x22, 0x22, 0x22, 0x3C, 0x20, 0x1C},  // y 89
    { 0x00, 0x00, 0x3E, 0x10, 0x08, 0x04, 0x3E},  // z 90
    { 0x0C, 0x04, 0x06, 0x02, 0x06, 0x04, 0x0C},  // { 91
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},  // | 92
    { 0x0C, 0x08, 0x18, 0x10, 0x18, 0x08, 0x0C},  // } 93
};

//-------------------------------------------------------------------------------------------------------------------
//  �������      ������ɨ�躯��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      �˺�����Ҫ��exti_ch3_ch7_isr�жϵ�ERU_CH7_REQ16_P15_1��֧�е���
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_scan(void)
{
    uint8 i;
    static uint8 entry_num = 0;     /* 本函数被调次数:2 个 entry = 1 行,7 行共 14 entry/帧 */
    uint8 display_row_now;          /* 当前处理第几行 = entry_num/2 */

    /* 探针模式:自检函数独占行地址/EN/占空比,本函数直接退出,
     * 只保留 isr.c 里的 SYNC 边沿计数(那行在调本函数之前)。 */
    if(dot_matrix_screen_probe)
    {
        entry_num = 0;
        return;
    }

    /* ── 官方 SYNC/EXTI 逐 entry 扫描(2026-07-25 上)──
     *   TLD7002 每个 PWM 周期在 SYNC(P15.8)吐一个下降沿 → 触发 exti_ch1_ch5_isr → 调本函数。
     *   关键:TLD7002 占空比写进影子寄存器后,要等"下一个 PWM 周期"DC_SYNC 才真正生效
     *   (见官方 FuncLayer 注释)。故每行拆 2 个 entry:
     *     偶 entry:消隐(EN 低)+ 下发本行占空比到影子寄存器(还没生效);
     *     奇 entry:占空比已过一个 PWM 周期生效 → 设行地址 + EN 高点亮。
     *   这样行切换严格锁在 PWM 边界,消除跨周期残影/错行(原每行不等待 PWM 边界,
     *   导致某些行占空比串到相邻行 → 2/4 行不亮、其余闪)。
     *   帧率 = SYNC 频率 / 14。 */

    display_row_now = entry_num / 2;

    if(0 == (entry_num % 2))
    {
        /* ===== 偶 entry:消隐 + 设行地址 + 下发占空比(下个 PWM 周期才生效) ===== */
        gpio_low(DOT_MATRIX_SCREEN_ROW_EN_PIN);     /* EN 高有效,拉低=全行消隐,防改占空比时串行 */

        /* 消隐态下提前设地址:地址有整整一个 PWM 周期稳定,再在奇 entry 拉高 EN,
         * 避免\"地址刚变就使能译码器\"导致的短暂错行(2026-07-25 按官方节拍调整)。 */
        DOT_MATRIX_SET_ROW_ADDR(display_row_now);

        if(dot_matrix_screen_all_on)
        {
            /* 全亮模式:忽略帧缓冲,15 列全点亮(标定中指示) */
            for(i = 0; 15 > i; i++)
            {
                tld7002_duty[i] = dot_matrix_screen_brightness;
            }
        }
        else
        {
            /* 只展开帧缓冲的位:字库解码已移到 show_string()/主循环,
             * ISR 里只剩 15 次位测试,比原来三轮字库查表快得多。 */
            uint16 row_bits = dot_fb_active[display_row_now];

            for(i = 0; 15 > i; i++)
            {
                tld7002_duty[i] = (row_bits & ((uint16)1 << i)) ? dot_matrix_screen_brightness : 0;
            }
        }
        tld7002_set_duty(1);
    }
    else
    {
        /* ===== 奇 entry:占空比已过一个 PWM 周期生效、地址也已稳定 → 只拉高 EN 点亮 ===== */
        gpio_high(DOT_MATRIX_SCREEN_ROW_EN_PIN);    /* EN 高=使能 74HC238,选中行 Y 输出高→高边导通 */
    }

    entry_num++;
    if((DOT_MATRIX_SCREEN_ROW_NUM * 2) <= entry_num)
    {
        /* 整帧 14 entry 扫完:归零 + 帧尾整帧搬入新画面(只在这一瞬间切,不可能撕裂) */
        entry_num = 0;
        if(dot_fb_dirty)
        {
            uint8 r;
            for(r = 0; DOT_MATRIX_SCREEN_ROW_NUM > r; r++)
            {
                dot_fb_active[r] = dot_fb_pending[r];
            }
            dot_fb_dirty = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      �������ַ�����ʾ����
//  ����˵��      *str          ��Ҫ��ʾ���ַ��������֧����ʾ3���ַ�
//  ���ز���      void
//  ʹ��ʾ��      dot_matrix_screen_show_string("OK");
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_show_string(const char *str)
{
    uint8 i;
    for(i = 0; i < 3; i++)
    {
        if(0 == str[i])
        {
            break;
        }
        dot_matrix_screen_data[i] = str[i];
    }

    /* 字库解码在这里(主循环上下文)完成,ISR 只读现成位图。 */
    dot_matrix_screen_text_to_fb(dot_matrix_screen_data);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      字符串→帧缓冲解码(内部用,只在主循环上下文调)
//  参数说明      text          3 个 ASCII 字符
//  返回参数      void
//  说明          字库是 5x7:每字符占 5 列。原 scan() 里的位取法为
//                font[ch][row] & (1 << (i+1)),即 bit1..bit5 对应该字符的 5 列,
//                这里完全沏用,保证与之前的字形方向一致。
//-------------------------------------------------------------------------------------------------------------------
static void dot_matrix_screen_text_to_fb(const int8 *text)
{
    uint8  row;
    uint8  ch_idx;
    uint8  i;
    uint16 bits;

    for(row = 0; DOT_MATRIX_SCREEN_ROW_NUM > row; row++)
    {
        bits = 0;

        for(ch_idx = 0; 3 > ch_idx; ch_idx++)
        {
            /* 防越界:字库只涵盖 ' '(0x20) 到 '}'(0x7D),其余当空格。 */
            int8 c = text[ch_idx];
            const uint8 *glyph;

            if((' ' > c) || ('}' < c))
            {
                c = ' ';
            }
            glyph = tld7002_ascii_font_5x7[c - ' '];

            for(i = 0; 5 > i; i++)
            {
                if(glyph[row] & ((uint16)1 << (i + 1)))
                {
                    bits |= (uint16)1 << (ch_idx * 5 + i);
                }
            }
        }

        dot_fb_pending[row] = bits;
    }

    dot_fb_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      直接下发一整帧 7x15 图案(科目二语音控图案用)
//  参数说明      rows          7 个 uint16,rows[r] 的 bit i = 第 i 列(C0..C14)亮灭
//  返回参数      void
//  说明          与 show_string 二选一:后调的赢。写的是 pending 缓冲,
//                scan() 在下一个帧头整帧生效,不会出现半帧画面。
//                可以直接喷 kart_light_copy_frame() 抽出来的帧。
//  使用示例      uint16 f[7]; kart_light_copy_frame(f); dot_matrix_screen_show_frame(f);
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_show_frame(const uint16 *rows)
{
    uint8 row;

    if(NULL == rows)
    {
        return;
    }

    for(row = 0; DOT_MATRIX_SCREEN_ROW_NUM > row; row++)
    {
        dot_fb_pending[row] = (uint16)(rows[row] & 0x7FFFU);
    }

    dot_fb_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      清屏(帧缓冲全 0)
//  参数说明      void
//  返回参数      void
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_clear(void)
{
    uint8 row;

    for(row = 0; DOT_MATRIX_SCREEN_ROW_NUM > row; row++)
    {
        dot_fb_pending[row] = 0;
    }

    dot_fb_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ���������ȵ���
//  ����˵��      brightness    ���õ���Ļ���� 0-10000
//  ���ز���      void
//  ʹ��ʾ��      dot_matrix_screen_show_string("OK");
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_set_brightness(uint16 brightness)
{
    dot_matrix_screen_brightness = brightness;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      点阵屏全亮开关(标定中指示,=1 忽略字库把 15 列全点亮)
//  参数说明      on            1=全亮  0=恢复字库显示
//  返回参数      void
//  使用示例      dot_matrix_screen_set_all_on(1);
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_set_all_on(uint8 on)
{
    dot_matrix_screen_all_on = on ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      静态单行硬件自检(诊断用):停扫描,锁第0行 EN 常通,15 列拉满,阻塞保持 hold_ms
//  参数说明      hold_ms       保持点亮的毫秒数;=0 时无限常亮(死循环,专供量表)
//  返回参数      void
//  说明          把"扫描时序"与"TLD7002/灯板硬件"解耦:
//                  第0行亮 → TLD7002+该行通路 OK,问题在扫描时序;
//                  仍全黑 → TLD7002 无输出或灯板供电/接线问题,需上表测。
//                EN 高电平有效:拉高使能译码器选中行(238 的 E3 使能高有效,选中输出为高)。
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_test_row0_static(uint16 hold_ms)
{
    uint8  i;
    uint16 t;

    /* 独占硬件(2026-07-26 修):置 probe 让 scan() 空转。
     * 原来靠"扫描由 SYNC 驱动、这里不开 EXTI"隐式独占,
     * 但 DOT_MATRIX_SCREEN_USE_PIT_SCAN=1 后扫描改由 1ms PIT 驱动,
     * 不置 probe 就会被 scan() 抢 A0/A1/A2/EN/duty → 本函数不再是"静态"。 */
    dot_matrix_screen_probe = 1;

    /* 选中第 0 行:A0/A1/A2 = 000 */
    gpio_low(DOT_MATRIX_SCREEN_ROW_A0_PIN);
    gpio_low(DOT_MATRIX_SCREEN_ROW_A1_PIN);
    gpio_low(DOT_MATRIX_SCREEN_ROW_A2_PIN);

    /* 15 列全部拉满 */
    for(i = 0; 15 > i; i++)
    {
        tld7002_duty[i] = dot_matrix_screen_brightness;
    }
    tld7002_set_duty(1);

    /* EN 拉高使能该行(高有效,常通,不再翻转) */
    gpio_high(DOT_MATRIX_SCREEN_ROW_EN_PIN);

    /* hold_ms==0:无限常亮死循环,专供量表(期间反复下发占空比防 TLD7002 超时熄灭) */
    if(0 == hold_ms)
    {
        uint16 diag_tick = 0;
        for(;;)
        {
            tld7002_set_duty(1);
            system_delay_ms(10);

            /* 每约 1s 一轮"纯 UART0 发送自测":绕开整个 TLD7002 协议栈,直接往 UART0
             * (TLD7002 端口)发 4 个原始字节,再看 rx_count 是否因 1kΩ 半双工回环跳动。
             * 硬件已确认 P14.0↔P14.1 有 1kΩ 桥接(模块内 RX/HSLI_L 互连),故:
             *   TX 只要真发出 → 回环必进 RX → raw_loop_delta > 0
             *   raw_loop_delta == 0 → UART0 TX 根本没发出字节(TX 链路/引脚复用/初始化顺序)
             * VOFA JustFloat 4 通道:
             *   ch0=ERR ch1=RX(ISR累计) ch2=ISR增量(依赖全局中断) ch3=轮询增量(不依赖中断)
             *   ch2>0            → 中断开+TX好 → 问题在协议/芯片
             *   ch2==0 且 ch3>0  → 中断没开+TX好 → 修法:core0 开全局中断
             *   ch2==0 且 ch3==0 → TX 真死 → 引脚复用/init 顺序 */
            if(++diag_tick >= 100)
            {
                static const uint8 vofa_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
                static const uint8 raw_probe[4]  = {0x55U, 0xAAU, 0x55U, 0xAAU};
                uint32 rx_before;
                uint32 raw_loop_delta;    /* ISR 计数增量:靠 uart0_rx_isr(依赖全局中断) */
                uint32 poll_loop_cnt = 0; /* 轮询计数:直接读硬件 RX FIFO,不依赖中断 */
                float  frame[4];
                uint8  k;
                uint8  dummy;

                rx_before = tld7002_rx_count;
                for(k = 0; 4U > k; k++)
                {
                    uart_write_byte(TLD7002_UART_INDEX, raw_probe[k]);
                }
                system_delay_ms(2);     /* 等回环字节走完(2Mbaud 4字节约20us) */

                /* ISR 路径增量(全局中断关则恒为 0) */
                raw_loop_delta = tld7002_rx_count - rx_before;

                /* 轮询路径:直接读硬件 FIFO,与 ISR 抢同一 FIFO。
                 * 若中断没开→ISR 没搬走字节→这里能捞到→poll_loop_cnt>0。 */
                for(k = 0; 8U > k; k++)
                {
                    if(uart_query_byte(TLD7002_UART_INDEX, &dummy))
                    {
                        poll_loop_cnt++;
                    }
                }

                /* 2026-07-24 决定性诊断:切开回环与芯片应答。
                 * ch0=ERR(0=芯片正确应答/1=COMM_ERROR)
                 * ch1=init 期间发出字节数(tx_after_init) —— 半双工必等量回环
                 * ch2=init 期间收到字节数(rx_after_init) —— 回环 + 芯片应答
                 * ch3=芯片真实应答字节 = rx_after_init - tx_after_init:
                 *      ==0 → 芯片全程哑巴(收得到帧不回)→ 没进 ACTIVE / 芯片坏
                 *      >0  → 芯片在应答,帧被判无效 → 查 2M 波特率/时序 */
                frame[0] = (float)tld7002_init_err;                             /* ch0=ERR */
                frame[1] = (float)tld7002_tx_after_init;                        /* ch1=发出(回环基准) */
                frame[2] = (float)tld7002_rx_after_init;                        /* ch2=收到(回环+应答) */
                frame[3] = (float)((int32)tld7002_rx_after_init - (int32)tld7002_tx_after_init); /* ch3=芯片应答字节 */
                (void)raw_loop_delta;
                (void)poll_loop_cnt;
                uart_write_buffer(BOARD_AUX_UART_INDEX, (const uint8 *)frame, sizeof(frame));
                uart_write_buffer(BOARD_AUX_UART_INDEX, vofa_tail, 4U);
                diag_tick = 0;
            }
        }
    }

    /* 阻塞保持:期间反复下发占空比,防 TLD7002 掉电/超时熄灭 */
    for(t = 0; t < (hold_ms / 10); t++)
    {
        tld7002_set_duty(1);
        system_delay_ms(10);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      静态逐行硬件排查(死循环,永不返回):15 列只写一次全亮,逐行切地址每行保持 1s
//  参数说明      void
//  返回参数      void(死循环)
//  说明          与 test_all_on_scan 的区别:不快扫,每行常亮 1s,便于肉眼+万用表逐级测。
//                列占空比只下发一次(不每行重发 tld7002_set_duty),彻底把\"行译码链\"与\"列通信\"解耦:
//                  某行不亮 → 该行 74HC238 输出/S8050/AO3401 通路故障;
//                  按缺行组合判地址线:2/4/6 行全缺→A0;3/4/7→A1;5/6/7→A2(A0/A1/A2=P20.8/9/10)。
//                地址与行:R0=000(Y0) R1=001(Y1) ... R6=110(Y6)。
//                测完必须改回正常主流程。
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_test_rows_static(void)
{
    uint8 i;
    uint8 row;

    /* 独占硬件(2026-07-26 修):必须置 probe,不能只关 EXTI。
     * 原来只 exti_disable() —— 那在 SYNC 驱动扫描的年代成立;
     * 现在 DOT_MATRIX_SCREEN_USE_PIT_SCAN=1,扫描时基是 1ms PIT(CCU61_CH0),
     * 关 EXTI 根本停不掉 scan(),本函数会被抢 EN/地址/列数据 → 乱闪,
     * 也就是说改软扫之后这个诊断工具一直是坏的。probe=1 才真能独占。 */
    dot_matrix_screen_probe = 1;
    exti_disable(DOT_MATRIX_SCREEN_SYNC_PIN);   /* SYNC 已降级为诊断计数,顺手关掉省中断 */

    gpio_low(DOT_MATRIX_SCREEN_ROW_EN_PIN);     /* 先全消隐 */

    /* 15 列 + OUT15 同步通道:只配置一次 */
    for(i = 0; 15 > i; i++)
    {
        tld7002_duty[i] = dot_matrix_screen_brightness;
    }
    tld7002_duty[15] = 5000;
    tld7002_set_duty(1);
    system_delay_ms(5);                         /* 等占空比进入实际输出 */

    for(;;)
    {
        for(row = 0; DOT_MATRIX_SCREEN_ROW_NUM > row; row++)
        {
            uint16 t;

            gpio_low(DOT_MATRIX_SCREEN_ROW_EN_PIN);     /* 消隐 */
            DOT_MATRIX_SET_ROW_ADDR(row);               /* 设行地址 */
            system_delay_us(2);                         /* 地址 + 三极管/MOS 稳定时间 */
            gpio_high(DOT_MATRIX_SCREEN_ROW_EN_PIN);    /* 点亮该行 */

            /* 切行时往 VOFA 发当前行号(单通道 JustFloat):对着屏看\"VOFA 显示几→哪行亮\",
             * 建立 地址(A0/A1/A2)→物理行 映射,解决\"哪边是上\"的疑问。 */
            {
                static const uint8 vofa_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
                float row_f = (float)row;
                uart_write_buffer(BOARD_AUX_UART_INDEX, (const uint8 *)&row_f, sizeof(row_f));
                uart_write_buffer(BOARD_AUX_UART_INDEX, vofa_tail, 4U);
            }

            /* 每行保持 1s:期间每 10ms 重发一次占空比,防 TLD7002 通信看门狗超时把列拉暗/掉电
             * (原只发一次→列在几十 ms 后被降到安全态→其他行低亮乱闪、顺序也被打乱)。 */
            for(t = 0; t < 100; t++)
            {
                tld7002_set_duty(1);
                system_delay_ms(10);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      静态点亮 + SYNC 测频探针(死循环,永不返回)
//  参数说明      void
//  返回参数      void(死循环)
//  说明          置 dot_matrix_screen_probe=1 让 scan() 空转,独占行译码硬件;
//                15 列直接写 duty(不走 all_on 标志,那个标志只在 scan() 里才展开),
//                7 行逐行各静态点亮 1s;EXTI 保持开启,只用于累计 SYNC 边沿。
//                循环里每 10ms 重发一次占空比喂 TLD7002 通信看门狗(2000ms,
//                otp_reg[33]=0x8007 → DIAG_WDT_SET=7),且全程不 reinit 芯片 ——
//                PM_CHANGE(INIT) 会关掉全部输出。
//
//                每 1s(=每换一行)往 VOFA 发 6 通道 JustFloat:
//                  ch0 = 本秒 SYNC 边沿数 ≈ SYNC 频率(Hz)
//                  ch1 = 累计 SYNC 边沿
//                  ch2 = 开机 tld7002_init() 的返回码(0=NO_ERR,非0=COMM_ERROR)
//                  ch3 = 当前点亮的行地址 0~6(对着屏记"地址→物理行",为 DOT_ROW_MAP 打底)
//                  ch4 = 本秒 UART1 收到字节数(>0 说明喂狗帧真发出去了、半双工回环在跑)
//                  ch5 = init 阶段芯片应答字节 = rx_after_init - tx_after_init
//                判读:
//                  屏亮 + ch0≈2000  → 芯片/供电/列驱动/SYNC 全好 → 故障在 scan() 时序
//                  屏亮 + ch0 偏低/为 0 → 芯片好但 SYNC 整形链(OUT15→LMV321→P15.8 飞线)不行
//                  屏黑 + ch4>0     → MCU 侧通信正常但芯片不出流:查 VS/VDD/R3/GPIN0/芯片地址
//                  屏黑 + ch4==0    → UART1 收发链断:查 P11.12/P11.10 复用与飞线
//                本车实测结论:屏亮、ch0==0、ch4≈2300 → SYNC 整形链不振荡,
//                故已改 1ms PIT 软扫(DOT_MATRIX_SCREEN_USE_PIT_SCAN=1),SYNC 只留作诊断。
//                测完必须把 KART_DOT_ALLON_TEST 改回 0,否则死循环进不了主循环。
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_test_all_on_sync(void)
{
    static const uint8 vofa_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    uint32 total_edges = 0;
    uint32 edges;
    uint32 rx_before;
    uint8  row = 0;
    uint8  i;
    float  init_err_f;
    float  chip_reply_f;

    /* 探针接管硬件:scan() 立即空转,但 EXTI 不关 —— 还要靠它数 SYNC 边沿。 */
    dot_matrix_screen_probe = 1;
    dot_matrix_screen_all_on = 0;

    /* 开机那一次 init 的结果(dot_matrix_screen_init 里已调过 tld7002_init),
     * 本函数全程不再 reinit,避免 PM_CHANGE(INIT) 把输出关掉。 */
    init_err_f   = (float)tld7002_init_err;
    chip_reply_f = (float)((int32)tld7002_rx_after_init - (int32)tld7002_tx_after_init);

    /* 15 列全拉满 + OUT15 给中等占空比(SYNC 源)。直接写 duty,不经 all_on 标志。 */
    for(i = 0; 15 > i; i++)
    {
        tld7002_duty[i] = dot_matrix_screen_brightness;
    }
    tld7002_duty[15] = 5000;

    gpio_low(DOT_MATRIX_SCREEN_ROW_EN_PIN);         /* 先全消隐 */
    tld7002_set_duty(1);
    system_delay_ms(5);                             /* 等占空比进入实际输出 */

    edges     = 0;
    rx_before = tld7002_rx_count;
    g_dot_sync_edges = 0;

    for(;;)
    {
        float  frame[6];
        uint16 t;

        gpio_low(DOT_MATRIX_SCREEN_ROW_EN_PIN);     /* 消隐 */
        DOT_MATRIX_SET_ROW_ADDR(row);               /* 设行地址 */
        system_delay_us(2);                         /* 地址 + 三极管/MOS 稳定时间 */
        gpio_high(DOT_MATRIX_SCREEN_ROW_EN_PIN);    /* 点亮该行 */

        /* 先报后保持:上一版把 VOFA 帧放在 1s hold 之后发,帧到达的瞬间
         * 屏上已经切到下一行 → 永远错位一行。现在点亮后立即发(460800波特率
         * 28 字节约 0.6ms,相对 1s 可忽略),ch3 与眼睛看到的亮行同步。
         * 代价:ch0/ch4 报的是"上一行那秒"的计数,而这两个量与行无关,不影响判读。 */
        edges        = g_dot_sync_edges;
        total_edges += edges;

        frame[0] = (float)edges;                                /* ≈SYNC Hz(上一秒) */
        frame[1] = (float)total_edges;
        frame[2] = init_err_f;
        frame[3] = (float)row;                                  /* 与当前亮的行同步 */
        frame[4] = (float)(tld7002_rx_count - rx_before);       /* 上一秒收到字节 */
        frame[5] = chip_reply_f;

        uart_write_buffer(BOARD_AUX_UART_INDEX, (const uint8 *)frame, sizeof(frame));
        uart_write_buffer(BOARD_AUX_UART_INDEX, vofa_tail, 4U);

        g_dot_sync_edges = 0;
        rx_before = tld7002_rx_count;

        /* 本行保持 1s:每 10ms 重发占空比喂 2000ms 通信看门狗。 */
        for(t = 0; 100U > t; t++)
        {
            tld7002_set_duty(1);
            system_delay_ms(10);
        }

        row++;
        if(DOT_MATRIX_SCREEN_ROW_NUM <= row)
        {
            row = 0;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  �������      ��������ʼ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      ��Ҫ�ڵ���
//-------------------------------------------------------------------------------------------------------------------
void dot_matrix_screen_init(void)
{
    /* 1. 先配行译码器 IO:EN 初值拉低=全行消隐,保证\"任何时刻(含后续第一次 SYNC)
     *    A0/A1/A2 与 EN 都已就绪\",避免第一个 SYNC 边沿到来时 IO 还没配置就误开行。 */
    gpio_init(DOT_MATRIX_SCREEN_ROW_A0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DOT_MATRIX_SCREEN_ROW_A1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DOT_MATRIX_SCREEN_ROW_A2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DOT_MATRIX_SCREEN_ROW_EN_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);  /* EN 高有效,初值拉低=禁用 */

    /* 2. 初始化 TLD7002(列驱动) */
    tld7002_init();

    /* 3. 第 15 通道给个占空比让 TLD7002 起振输出 → SYNC 脚才有周期性下降沿驱动 EXTI */
    tld7002_duty[15] = 5000;
    tld7002_set_duty(1);
    system_delay_ms(10);

    /* 4. 开 SYNC 中断。注意:软扫模式下它已降级为纯诊断 —— ISR 里只
     *    g_dot_sync_edges++ 不再调 scan()(见 isr.c 的条件编译)。保留它是为了
     *    随时能从 VOFA 看 SYNC 活没活,万一以后灯板整形链修好了可以一键切回。 */
    exti_flag_clear(DOT_MATRIX_SCREEN_SYNC_PIN);
    exti_init(DOT_MATRIX_SCREEN_SYNC_PIN, EXTI_TRIGGER_FALLING);

    /* 5. 清空帧缓冲后再开扫描时基,避免第一帧括出随机亮点。 */
    dot_matrix_screen_clear();

#if DOT_MATRIX_SCREEN_USE_PIT_SCAN
    /* 1ms PIT 软扫:2 entry/行、14 entry/帧 → 帧率 ≈ 71Hz。
     * 放在最后:保证中断第一次进来时 IO、TLD7002、帧缓冲都已就绪。 */
    pit_ms_init(DOT_MATRIX_SCREEN_PIT_CH, 1);
#endif

    dot_matrix_screen_set_brightness(dot_matrix_screen_brightness);
    dot_matrix_screen_show_string("   ");
}

#if defined(__TASKING__)
#pragma section all restore
#endif
