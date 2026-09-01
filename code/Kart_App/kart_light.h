#ifndef KART_LIGHT_H_
#define KART_LIGHT_H_
#include "zf_common_headfile.h"

/*
 * 卡丁快跑灯板逻辑层
 *
 * 本模块负责把比赛灯光命令转换成 7x15 单色点阵帧，不直接初始化 UART、
 * TLD7002、SYNC 中断或行选 GPIO。实际显示由 user/cpu0_main.c 的
 * kart_task_light_10ms() 取帧,经点阵层 dot_matrix_screen_show_frame() 下发。
 *
 * 点阵坐标约定：row=0 为最上行，col=0 为观察者看到的最左列；
 * kart_light_get_row() 返回值中的 bit14 对应 col0，bit0 对应 col14。
 * 【注意】点阵层 dot_matrix_screen_show_frame() 的列位约定与此相反
 * (它的 bit0 是最左列)，帧必须先过 cpu0_main.c 的
 * kart_light_bits_to_dot() 翻转。左右箭头一旦镜像就是反向指示，
 * 属于评分错误，这层不能省。
 *
 * 推荐调用流程：
 *
 * 1. 系统初始化时调用一次：
 *
 *      kart_light_init();
 *      kart_light_set_brightness(7000U);
 *
 * 2. 收到比赛灯光命令或车辆状态改变时，切换显示内容：
 *
 *      kart_light_set_command(KART_LIGHT_CMD_LEFT_TURN);
 *
 * 3. 在主循环或周期任务中推进动画。参数必须是两次调用之间实际
 *    经过的毫秒数，函数内部不会阻塞，也不会调用 delay：
 *
 *      void app_task_10ms(void)
 *      {
 *          kart_light_update(10U);
 *      }
 *
 * 4. 消费方在下发一帧之前复制完整帧，并读取亮度：
 *
 *      uint16 scan_rows[KART_LIGHT_ROW_NUM];
 *      uint16 brightness;
 *
 *      kart_light_copy_frame(scan_rows);
 *      brightness = kart_light_get_brightness();
 *
 *    随后把 scan_rows[0]~scan_rows[6] 和 brightness 交给点阵层。
 *    2026-07-26 起行扫描时基是 1ms PIT 软扫(见 zf_device_dot_matrix_screen.h
 *    的 DOT_MATRIX_SCREEN_USE_PIT_SCAN，原 TLD7002 SYNC 整形链实测不出波)，
 *    所以不要在一轮扫描中途再次复制帧，否则同一屏会出现前后两个画面。
 */

#define KART_LIGHT_ROW_NUM              (7U)
#define KART_LIGHT_COL_NUM              (15U)
#define KART_LIGHT_MAX_BRIGHTNESS       (10000U)
#define KART_LIGHT_DEFAULT_BRIGHTNESS   (7000U)

#define KART_LIGHT_TURN_STEP_MS         (180U)
#define KART_LIGHT_HAZARD_STEP_MS       (400U)
#define KART_LIGHT_WIPER_STEP_MS        (160U)

typedef enum
{
    KART_LIGHT_CMD_OFF = 0,       /* 关闭灯板，显示全黑帧。 */
    KART_LIGHT_CMD_LEFT_TURN,     /* 左转向灯，循环播放向左箭头。 */
    KART_LIGHT_CMD_RIGHT_TURN,    /* 右转向灯，循环播放向右箭头。 */
    KART_LIGHT_CMD_HIGH_BEAM,     /* 远光灯，暂时使用 HIG 表示。 */
    KART_LIGHT_CMD_LOW_BEAM,      /* 近光灯，暂时使用 LOW 表示。 */
    KART_LIGHT_CMD_FOG,           /* 雾灯，暂时使用 FOG 表示。 */
    KART_LIGHT_CMD_HAZARD,        /* 双闪灯，循环闪烁 !!! 图案。 */
    KART_LIGHT_CMD_CABIN,         /* 车内照明灯，暂时使用 CAB 表示。 */
    KART_LIGHT_CMD_WIPER,         /* 雨刷器，循环播放摆动动画。 */
    KART_LIGHT_CMD_COUNT          /* 命令总数，仅用于范围检查。 */
} kart_light_command_t;

/*
 * 清空点阵帧并初始化灯板逻辑状态。
 * 本函数不初始化 TLD7002，也不会操作任何 GPIO 或串口。
 */
void                 kart_light_init            (void);

/*
 * 设置需要显示的比赛灯光命令。
 * 命令改变后会立即从新动画的第 0 帧开始；非法值自动回到关闭状态。
 */
void                 kart_light_set_command     (kart_light_command_t command);

/*
 * 返回当前正在显示的灯光命令。
 */
kart_light_command_t kart_light_get_command     (void);

/*
 * 设置逻辑亮度，取值范围为 0~10000。
 * 超过 KART_LIGHT_MAX_BRIGHTNESS 的值会被限制到最大值。
 */
void                 kart_light_set_brightness  (uint16 brightness);

/*
 * 返回当前逻辑亮度，取值范围为 0~10000。
 */
uint16               kart_light_get_brightness  (void);

/*
 * 使用距离上次调用实际经过的毫秒数推进动画。
 * 本函数不阻塞且不包含 delay，当前由 10ms 拍调用(cpu0_main.c 传
 * 2U * KART_MAIN_LOOP_PERIOD_MS)。
 */
void                 kart_light_update          (uint16 elapsed_ms);

/*
 * 返回指定点阵行的 15 位像素数据，row 的有效范围为 0~6。
 * 供调试和单元测试单行取值用；正式流程走 kart_light_copy_frame()。
 * bit14 是最左侧像素，bit0 是最右侧像素；行号越界时返回 0。
 */
uint16               kart_light_get_row         (uint8 row);

/*
 * 把当前完整的 7 行画面复制到 out_rows，供底层完成一轮稳定扫描。
 * 传入空指针时直接返回。
 */
void                 kart_light_copy_frame      (uint16 out_rows[KART_LIGHT_ROW_NUM]);

#endif
