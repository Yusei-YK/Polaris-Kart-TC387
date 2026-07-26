#include "zf_common_headfile.h"
#include "kart_encoder.h"
#include "kart_power.h"
#include "kart_steer_abs.h"
#include "kart_debug_uart.h"
#include "kart_imu.h"
#include "kart_control.h"
#include "kart_odom.h"
#include "kart_steer_ctrl.h"
#include "kart_voice.h"
#include "kart_horn.h"
#include "kart_motion.h"
#include "kart_record.h"
#include "kart_playback.h"
#include "kart_mission.h"
#include "kart_remote.h"
#include "kart_hw_test.h"
#include "kart_menu.h"
#include "zf_device_dot_matrix_screen.h"
#include "kart_light.h"
#include "kart_multicore.h"
#include "isr.h"                 /* g_kart_tick_5ms + 调度器监测 g_sched_* */

/* 硬件自测开关:=1 时开机进 kart_hw_test_run() 死循环(验证并口屏/旋钮/按键),
 * 不跑正式主循环。硬件确认完必须改回 0。 */
#define KART_HW_TEST    (0)

/* 点阵屏静态单行诊断开关:=1 时 dot init 后锁第0行常亮 3s(解耦扫描时序与硬件)。
 * 第0行亮=TLD7002+通路OK问题在扫描;仍黑=硬件/供电/接线。测完必须改回 0。 */
#define KART_DOT_ROW0_TEST  (0)

/* 点阵屏逐行静态排查开关:=1 时 dot init 后进死循环,15 列恒亮、7 行逐行各保持 1s。
 * 用途:先把\"行译码链(74HC238/S8050/AO3401)\"与\"扫描时序\"彻底解耦——
 *   看哪几行不亮 → 缺 2/4/6→A0,缺 3/4/7→A1,缺 5/6/7→A2;单行缺→该行输出链故障。
 * 这是修显示前的第一步硬件确认。测完必须改回 0。 */
#define KART_DOT_ROWS_TEST  (0)

/* 点阵屏"SYNC 驱动全亮"自检开关:
 * =1 时 dot init 后进死循环,只置 all_on 标志,扫描全交给 SYNC(P15.8)→exti_ch1_ch5_isr→scan()。
 * 与 ROW0/ROWS 两个静态测试不同,本测试不自己碰硬件,故能同时验证
 *   ①SYNC 边沿是否真到 MCU ②EXTI 通道是否挂对 ③scan() 时序是否正确。
 * 每秒重跑一次芯片 init 并往 VOFA 发 6 通道:
 *   ch0=本秒 SYNC_Hz ch1=init 返回码 ch2=本轮发出字节 ch3=本轮收到字节
 *   ch4=芯片应答字节(ch3-ch2) ch5=累计 SYNC 边沿
 * 期望:屏 7×15 全亮 + ch1=0 + ch0 数千。判读表见函数注释。测完必须改回 0。 */
#define KART_DOT_ALLON_TEST (0)

/* 菜单系统开关:=1 时启用 IPS200 菜单(取代 VOFA m 命令切科目) */
#define KART_USE_MENU   (1)

/* 协作式调度器开关(A/B 对照):
 *   =1  5ms PIT 节拍(g_kart_tick_5ms)驱动的分频调度,控制窗口对齐硬件拍;
 *   =0  回退旧主循环(末尾 system_delay_ms(5),周期=执行耗时+5ms,非恒定)。
 * 出问题可临时改 0 用旧主循环 A/B 对照定位。 */
#define KART_USE_SCHEDULER  (1)

#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif

/* ===== 协作式调度器运行时监测(isr.h extern,VOFA 读)=====
 *   last_exec_us  —— 上一拍任务分发耗时(us)
 *   max_exec_us   —— 历史最大分发耗时(us),观察最坏情况
 *   overrun_count —— 漏周期累计(主循环一次跨 >1 tick 即累加差值,不补跑) */
volatile uint32 g_sched_last_exec_us  = 0;
volatile uint32 g_sched_max_exec_us   = 0;
volatile uint32 g_sched_overrun_count = 0;

#if KART_USE_SCHEDULER
/* 点阵屏显当前模式号:000=待机 111=科目一 222=科目二 444=科目三 555=科目四 333=遥控 FFF=故障 */
static void kart_dot_show_mode(void)
{
    switch(kart_mission_get_mode())
    {
        case MISSION_SUBJECT_1: kart_multicore_dot_show_string("111"); break;
        case MISSION_SUBJECT_2: kart_multicore_dot_show_string("222"); break;
        case MISSION_SUBJECT_4: kart_multicore_dot_show_string("555"); break;
        case MISSION_REMOTE:    kart_multicore_dot_show_string("333"); break;
        case MISSION_FAULT:     kart_multicore_dot_show_string("FFF"); break;
        case MISSION_IDLE:
        default:                kart_multicore_dot_show_string("000"); break;
    }
}

/* 5ms 拍(每 tick):后轮控制权仲裁 + 转向 + 复现 + 输出下发。
 * 全是有实时要求的控制链,必须每 5ms 跑一次对齐 5ms 中断的传感器/速度环更新。
 * power_check_poll/power_sync 各自按"每调用一次算一拍"计时/限幅,只能放 5ms 拍。 */
static void kart_task_5ms(void)
{
    power_check_poll();

    /* 刷新转向绝对编码器(SPI 读),steer_raw/center_delta 依赖它跟方向盘变。 */
    kart_steer_abs_update();

    /* 复现一拍:Pure Pursuit 找前视点覆写航向外环目标。
     * 必须在 steer_abs_update 之后、steer_ctrl_update 之前。 */
    kart_playback_poll();

    /* 转向串级一拍:紧跟 steer_abs_update 吃到本拍最新 center_delta。 */
    kart_steer_ctrl_update();

    /* 速度环未使能时强制后轮归零;转向串级独立运行,不受速度环门控。 */
    if(!kart_control_is_enabled())
    {
        power_set_rear_duty(0, 0);
    }

    power_sync();               /* slew 限幅 + 写三路 PWM,每拍步进一次 */
    /* 点阵屏扫描不在本拍:已由 1ms PIT(CCU61_CH0)的 cc61_pit_ch0_isr 逐 entry 驱动,
     * 2 entry/行、14 entry/帧 → 约 71Hz(见 zf_device_dot_matrix_screen.c / isr.c)。 */
}

/* 灯板一拍(10ms):推进动画相位,再把整帧 7x15 位图下发给点阵屏。
 * 字库/图案解码全在这里(主循环上下文)完成,ISR 里的 scan() 只读现成位图。
 *
 * 与 kart_dot_show_mode()(100ms 拍发"000/111/222")的仲裁:
 *   灯光命令 != OFF 时灯板图案优先,不发模式号,否则两边 100ms/10ms 交替抢屏。
 *   回 OFF 后 100ms 拍自然把模式号写回去。 */
/* 位序适配:两个模块的列位约定是相反的,必须在这里翻转,否则图案左右镜像
 *   kart_light.h  : bit14 = col0(最左) ... bit0 = col13(最右)
 *   show_frame()  : bit0  = C0 (最左) ... bit14 = C14(最右)
 * 左右转向箭头一旦镜像就是反向指示,属于评分错误,所以这层不能省。
 * 若实机发现整体还是左右颠倒,把 KART_LIGHT_MIRROR_COL 改成 0 即可。 */
#define KART_LIGHT_MIRROR_COL   (1)

static uint16 kart_light_bits_to_dot(uint16 v)
{
#if KART_LIGHT_MIRROR_COL
    uint16 r = 0;
    uint8  i;

    for(i = 0; KART_LIGHT_COL_NUM > i; i++)
    {
        /* kart_light 的 bit(14-i) 是第 i 列 -> 放到 dot 的 bit i */
        if(v & ((uint16)1 << (14U - i)))
        {
            r |= (uint16)1 << i;
        }
    }
    return r;
#else
    return (uint16)(v & 0x7FFFU);
#endif
}

static void kart_task_light_10ms(void)
{
    uint16 rows[KART_LIGHT_ROW_NUM];
    uint8  r;

    if(KART_LIGHT_CMD_OFF == kart_light_get_command())
    {
        return;                 /* 无灯光命令:屏幕归 kart_dot_show_mode() 管 */
    }

    kart_light_update(2U * KART_MAIN_LOOP_PERIOD_MS);   /* 传真实经过周期 10ms */
    kart_light_copy_frame(rows);

    for(r = 0; KART_LIGHT_ROW_NUM > r; r++)
    {
        rows[r] = kart_light_bits_to_dot(rows[r]);
    }

    /* 亮度也从灯光层取:kart_light 默认 7000,点阵层默认 10000,不同步会白得刺眼 */
    dot_matrix_screen_set_brightness(kart_light_get_brightness());
    dot_matrix_screen_show_frame(rows);
}

/* 10ms 拍:遥控失联计时 + 科目状态机 + 录制采样 + 日志采样组帧。
 * 均为事件/阈值驱动或有内部节拍,20ms 精度足够,让出控制窗口。
 * 日志实际串口发送不在此,靠主循环每 spin 的 background_poll 非阻塞排空。 */
static void kart_task_10ms(void)
{
    kart_remote_poll(2U * KART_MAIN_LOOP_PERIOD_MS);   /* 传真实经过周期 10ms */
    kart_mission_poll();
    kart_multicore_record_poll();
    kart_debug_uart_poll();     /* 只采样组帧入环形缓冲(内部再 4tick=20ms 门控) */

    kart_task_light_10ms();     /* 灯板动画推进 + 帧下发(仅科目二有灯光命令时生效) */
}

/* 50ms 拍:菜单按键扫描 + IPS200 屏幕刷新(全屏刷新耗时大,严禁进控制窗口)。 */
static void kart_task_50ms(void)
{
#if KART_USE_MENU
    kart_menu_poll();
#endif
}

/* 100ms 拍:状态显示(点阵模式号),纯观测,低频即可。 */
static void kart_task_100ms(void)
{
    /* 灯光图案在显时不写模式号,避免与 kart_task_light_10ms 交替抢屏。 */
    if(KART_LIGHT_CMD_OFF == kart_light_get_command())
    {
        kart_dot_show_mode();
    }
}
#endif  /* KART_USE_SCHEDULER */

int core0_main(void)
{
    clock_init();
    debug_init();
    kart_multicore_init();

#if KART_HW_TEST
    kart_hw_test_run();     /* 内部死循环,不返回 */
#endif

    power_init();
    kart_encoder_init();
    kart_steer_abs_init();
    kart_debug_uart_init();
    kart_control_init();        // 速度环:填默认 PID、清滤波、默认不使能(等 VOFA 发 e1 才输出)
    kart_steer_ctrl_init();     // 转向串级:填内/外环默认 PID,默认全不使能(等 VOFA 发 se1 才驱动转向电机)

    /* 点阵屏提前初始化(原在标定后):标定阻塞约 6s,期间点阵全亮作"标定中"指示,
     * 别再黑屏。system_delay 走 STM 忙等,不依赖 pit,提前 init 安全。 */
    dot_matrix_screen_init();
    dot_matrix_screen_set_brightness(10000);

#if KART_DOT_ROW0_TEST
    /* 静态单行常亮自检(死循环永不返回),专供万用表逐级定位断点:
     * row0 恒选(A0/A1/A2=000)、EN 恒高使能 74HC238、15 列恒满占空比。
     * 各节点都是稳定直流,便于测量(全亮扫描是 1ms 快切,表读平均值不好判)。
     * 期望链路:P33_8=3.3V → 238 Y0(SR0)=3.3V → 高边 → 行0线=8V;列线被 TLD7002 灌流拉低。
     * 测完必须把 KART_DOT_ROW0_TEST 改回 0,否则死循环进不了主循环。 */
    dot_matrix_screen_test_row0_static(0);
#endif

#if KART_DOT_ROWS_TEST
    /* 逐行静态排查(死循环永不返回):15 列恒亮,7 行逐行各保持 1s。
     * 观察缺行组合定位 A0/A1/A2 或单路输出故障(见 KART_DOT_ROWS_TEST 注释)。
     * 测完必须把 KART_DOT_ROWS_TEST 改回 0。 */
    dot_matrix_screen_test_rows_static();
#endif

#if KART_DOT_ALLON_TEST
    /* SYNC 驱动全亮自检(死循环永不返回):必须放在 dot_matrix_screen_init() 之后
     * (init 里已 exti_init 开了 P15.8 中断),且放在 kart_imu_init() 之前 ——
     * IMU 标定要静置约 6s,没必要为看灯等它;此处也不需要 5ms 拍。
     * 每秒 VOFA 4 通道:ch0=SYNC_Hz ch1=帧率 ch2=init_err ch3=芯片应答字节。
     * 测完必须把 KART_DOT_ALLON_TEST 改回 0,否则死循环进不了主循环。 */
    dot_matrix_screen_test_all_on_sync();
#endif

    dot_matrix_screen_set_all_on(1);                    /* 全亮:标定中 */
    /* 标定期间点阵扫描已由 1ms PIT 自动驱动(dot_matrix_screen_init 末尾已 pit_ms_init),
     * 不再需要标定 hook 手动刷屏 —— hook 再调 scan 会与 PIT 抢 entry_num,故撤。 */

    /* IMU660RA 初始化 + 上电静止标定零偏。
     * 注意:kart_imu_init() 先静置 1s,再采 1000 次(约 5s),
     *       必须在"开 5ms 中断之前"跑完 —— 此时车必须放稳别动。 */
    kart_imu_init();

    dot_matrix_screen_set_all_on(0);                    /* 熄灭全亮,恢复字库显示 */

    /* 航位推算基准初始化：必须在 IMU/编码器就绪后、开 5ms 中断前。 */
    kart_odom_init();

    /* 开 5ms 周期中断,进 cc60_pit_ch0_isr 调 kart_imu_update()。
     * 放在标定之后:保证进中断时零偏已就绪,解算从第一帧就是准的。 */
    pit_ms_init(CCU60_CH0, KART_MAIN_LOOP_PERIOD_MS);

    /* 点阵屏已在标定前 init(见上),此处不再重复。
     * 2026-07-26:扫描时基已从 SYNC/EXTI 改为 1ms PIT 软扫(实测 SYNC 边沿数为 0,
     * OUT15→LMV321→P15.8 整形链不出波),见 zf_device_dot_matrix_screen.h 的
     * DOT_MATRIX_SCREEN_USE_PIT_SCAN。100ms 拍只更新显示内容。 */

    /* 语音/鸣笛底层 init 在启动时做一次;
     * 收帧/分发/节拍推进已交给科目二 loop,只在 SUBJECT_2 模式下轮询。
     *
     * 2026-07-26 解决 ASCLIN1 冲突后放开(此前一直注释着):
     * 旧问题:语音原接 P33.12/13 = UART_1 = ASCLIN1,与 TLD7002 灯板飞线
     *   (P11.12/P11.10 @2M)同一硬件外设,kart_voice_init 会把 2M 冲成 115200 → 灯灭。
     * 旧方案(已作废):把 TLD7002 挪回 UART_0(P14.0/14.1)—— UART0 实测收发不通
     *   (ERR=1/RX=0/回环 0,见 zf_device_tld7002.h),且 P14.2~P14.6 属 boot 相关引脚
     *   (见工程根目录《不建议使用的引脚.txt》),这条路封死。
     * 现方案:语音改到 UART_10(P13.0/P13.1)= 原无线模块排针,详见 board_pins.h。
     *   比赛不接无线模块 → ASCLIN10 本就空闲;灯板独占 ASCLIN1 不动;
     *   SBUS 遥控独占 ASCLIN3 不动(遥控是唯一人工接管兜底,不能为让位语音而拔)。
     * 该口与 VOFA 日志共用,波特率切换在进/出科目二时做(kart_mission.c)。
     * 必须放在 kart_debug_uart_init() 之后:共用时本函数刻意不碰波特率,
     * 让启动阶段保持 460800 跑日志。 */
    kart_voice_init();
    kart_horn_init();

    /* 灯板逻辑层(7x15 图案/动画渲染):2026-07-26 接入。
     * 之前 kart_light.c 写好了但全工程无人调用,现在由语音命令 0x04~0x0B
     * 驱动(kart_voice_dispatch),动画在 10ms 拍推进,帧下发给点阵屏。 */
    kart_light_init();
    kart_light_set_brightness(KART_LIGHT_DEFAULT_BRIGHTNESS);

    kart_motion_init();
    kart_record_init();
    kart_playback_init();

    /* 科目状态机:置 IDLE 并执行统一停机,保证上电无残留输出。 */
    kart_mission_init();

    /* SBUS 枪式遥控接收(UART3,P15.7 RX)。第一版只解析+失联计数,只上 VOFA 观测,
     * 不接管电机/转向(见 kart_remote.h 安全红线)。 */
    kart_remote_init();

#if KART_USE_MENU
    /* IPS200 菜单系统:科目选择 + 路线录制/复现管理。
     * 启用后取代 VOFA m 命令,通过屏幕菜单 + 五向按键操作。 */
    kart_menu_init();
#endif

    cpu_wait_event_ready();
    kart_multicore_enable_runtime();

#if KART_USE_SCHEDULER
    /* ===== 5ms PIT 节拍驱动的协作式调度器 =====
     * 时基:cc60_pit_ch0_isr 每 5ms 末尾 g_kart_tick_5ms++(硬件拍,不受主循环耗时影响)。
     * 调度:主循环忙等 tick 变化,每变一拍分发一次;分频出 10/50/100ms 拍。
     * 漏拍:一次跨 >1 tick(某拍耗时超 5ms 或被长任务挤占),累加 overrun 差值,只跑最新一拍,不补跑历史。
     * 空转:tick 未变时反复调 background_poll 非阻塞排空日志环形缓冲(每次 ≤16B,≤347us)。 */
    {
        uint32 last_tick = g_kart_tick_5ms;
        uint32 sched_count = 0;         /* 已分发拍计数,用于 10/50/100ms 分频 */

        for(;;)
        {
            uint32 now_tick;
            uint32 delta;

            /* 每 spin 非阻塞排空日志:有货发一块(≤16B),空转即返回。 */
            kart_debug_uart_background_poll();

            now_tick = g_kart_tick_5ms;
            if(now_tick == last_tick)
            {
                continue;               /* 本拍未到,继续排日志/空转 */
            }

            delta = now_tick - last_tick;   /* uint32 差分,天然处理回绕 */
            last_tick = now_tick;
            if(delta > 1U)
            {
                /* 漏拍:累加跨过的历史拍数,只跑最新一拍,不连续补跑旧控制周期。 */
                g_sched_overrun_count += (delta - 1U);
            }

            {
                uint32 t0 = system_getval_us();
                uint32 exec_us;

                /* 5ms 拍:每 tick 必跑(控制链) */
                kart_task_5ms();

                /* 分频:sched_count 每拍 +1,对齐 10/50/100ms */
                sched_count++;
                if((sched_count % 2U) == 0U)   kart_task_10ms();   /* 10ms */
                if((sched_count % 10U) == 0U)  kart_task_50ms();   /* 50ms */
                if((sched_count % 20U) == 0U)  kart_task_100ms();  /* 100ms */

                exec_us = system_getval_us() - t0;   /* uint32 差分处理回绕 */
                g_sched_last_exec_us = exec_us;
                if(exec_us > g_sched_max_exec_us)
                {
                    g_sched_max_exec_us = exec_us;
                }
            }
        }
    }
#else
    /* ===== 旧主循环(A/B 对照回退)=====
     * 周期 = 本轮执行耗时 + 5ms,非恒定;仅在排查调度器问题时临时启用。 */
    while(TRUE)
    {
        switch(kart_mission_get_mode())
        {
            case MISSION_SUBJECT_1: kart_multicore_dot_show_string("111"); break;
            case MISSION_SUBJECT_2: kart_multicore_dot_show_string("222"); break;
            case MISSION_SUBJECT_4: kart_multicore_dot_show_string("555"); break;
            case MISSION_REMOTE:    kart_multicore_dot_show_string("333"); break;
            case MISSION_FAULT:     kart_multicore_dot_show_string("FFF"); break;
            case MISSION_IDLE:
            default:                kart_multicore_dot_show_string("000"); break;
        }
        power_check_poll();
        kart_steer_abs_update();
        kart_playback_poll();
        kart_steer_ctrl_update();

        if(!kart_control_is_enabled())
        {
            power_set_rear_duty(0, 0);
        }

        power_sync();
        /* 点阵屏扫描由 1ms PIT 中断驱动,旧主循环同样不需要软扫。 */
        kart_remote_poll(KART_MAIN_LOOP_PERIOD_MS);
        kart_mission_poll();
        kart_multicore_record_poll();
        kart_debug_uart_poll();

#if KART_USE_MENU
        kart_menu_poll();
#endif

        system_delay_ms(KART_MAIN_LOOP_PERIOD_MS);
    }
#endif  /* KART_USE_SCHEDULER */
}

#if defined(__TASKING__)
#pragma section all restore
#endif
