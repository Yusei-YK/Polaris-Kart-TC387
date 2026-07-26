#include "kart_mission.h"
#include "kart_control.h"
#include "kart_steer_ctrl.h"
#include "kart_playback.h"
#include "kart_record.h"
#include "kart_voice.h"
#include "kart_horn.h"
#include "kart_motion.h"
#include "kart_odom.h"
#include "kart_remote.h"
#include "kart_debug_uart.h"
#include "kart_light.h"
#include "board_pins.h"
#include <math.h>

/*
 * 科目状态机实现 —— 见 kart_mission.h 头注释。
 * 边界:不改 PID/中断/里程系数/Pure Pursuit。
 * 科目一 = 绕桩(playback 复现现场手推轨迹) + 倒库(几何法直线倒车)。
 * 倒库不用航向外环:倒车时前轮打角对车尾是正反馈会发散,故方向物理回中
 *   (target_delta=0)+ 只开转角内环把方向盘按在中位 + 速度环给负速直倒,
 *   短距(1.4m)直线偏差可接受。
 */

static kart_mission_mode_t   mission_mode  = MISSION_IDLE;
static kart_subject1_stage_t subject1_stage = S1_WAIT_START;
static kart_subject4_stage_t subject4_stage = S4_PHASE1_RECORD;

/* 倒库里程基准:进 S1_REVERSE_IN 时记下当前累计路程,增量到阈值判停。 */
static float subject1_reverse_dist0 = 0.0f;
/* START 键上一拍电平(下降沿检测:上拉输入,按下 1→0)。 */
static uint8 subject1_start_key_last = 1;

/* ============ 统一停机:任意模式退出/进 IDLE/FAULT 都调 ============ */
/* 覆盖交接文档指出的"b0 不关内环"的完整停机缺口。 */
static void mission_stop_all(void)
{
    kart_voice_cmd_t drop;

    /* 1. 停录制/复现/语音运动 */
    kart_playback_stop();               /* 内部已关速度环+航向外环+清目标 */
    kart_record_stop();
    kart_motion_stop();                 /* 复位运动状态机,防切模式后残留 busy 阻塞分发 */

    /* 2. 后轮目标清零 + 关速度环(playback_stop 已做,这里兜底保证) */
    kart_control_set_target(0.0f);
    kart_control_set_enable(0);

    /* 3. 关航向外环 + 转向内环(完整停机,内环也关) */
    kart_steer_set_head_enable(0);
    kart_steer_set_angle_enable(0);

    /* 4. 蜂鸣器立即静音 + 清空语音命令队列(取空为止) */
    kart_horn_stop();
    while(kart_voice_get_cmd(&drop)) { }
}

/* ============ 各模式 enter ============ */
static void mission_enter(kart_mission_mode_t mode)
{
    switch(mode)
    {
        case MISSION_SUBJECT_1:
            subject1_stage = S1_WAIT_START;     /* 进科目一从等待发车开始,不自行启动 */
            /* 记下 START 键当前电平作边沿基准,避免进模式瞬间误触发。 */
            subject1_start_key_last = gpio_get_level(BOARD_START_KEY_PIN);
            break;

        case MISSION_SUBJECT_2:
            /* 语音/鸣笛 init 在 cpu0_main 启动时已做一次;这里不重复 init。
             * 进入即准备接收,poll/dispatch 交给本模式 loop。
             *
             * 2026-07-26 串口交接(见 board_pins.h 语音模块段):
             * 语音模块与 VOFA 日志共用 UART_10(P13.0/13.1),波特率不同(115200 / 460800)。
             * 顺序必须是"先停日志、再切波特率",不能反:
             *   若先切到 115200,background_poll 还会把已在环里的 JustFloat 字节
             *   以 115200 喷出去,直接灌进语音模块 RX,可能被它当指令误解析。
             * 硬件前提:进科目二前拔掉无线模块,插上语音模块。 */
#if BOARD_VOICE_SHARES_AUX_UART
            kart_debug_uart_set_enabled(0);
#endif
            kart_voice_uart_acquire();
            break;

        case MISSION_SUBJECT_4:
            /* 科目四第一阶段:与科目三录制完全相同(遥控走迷宫 + 录制)。
             * 区别只在返程:科目四不掉头,停车区按同一键直接开环倒车原路返回。
             *   ① odom 清零(录制起点车体系对齐);② 开录制(CPU2 采样,同帧记打角+里程);
             *   ③ 遥控急停一次防残留,之后 remote_control_update 接管。
             * 注意:开环倒车靠 odom 累计里程映射,故录制到倒车全程 odom 不再 reset。 */
            subject4_stage = S4_PHASE1_RECORD;
            subject1_start_key_last = gpio_get_level(BOARD_START_KEY_PIN);
            kart_odom_reset();
            kart_odom_set_active(1);
            kart_record_start();
            kart_remote_control_stop();
            break;

        case MISSION_REMOTE:
            /* 进遥控即急停一次(速度0关使能、转向回中关内环),
             * 之后每拍由 remote_loop 按三段挡位/摇杆接管。防进入瞬间残留动作。 */
            kart_remote_control_stop();
            break;

        case MISSION_IDLE:
        case MISSION_FAULT:
        default:
            break;
    }
}

/* ============ 各模式 loop ============ */

/* START 键下降沿检测(上拉输入,按下 1→0)。每拍调一次,返回 1 表示本拍触发。 */
static uint8 subject1_start_pressed(void)
{
    uint8 now = gpio_get_level(BOARD_START_KEY_PIN);
    uint8 edge = (subject1_start_key_last == 1 && now == 0) ? 1 : 0;
    subject1_start_key_last = now;
    return edge;
}

/* 车体是否停稳(左右滤波测速幅值都低于阈值)。 */
static uint8 subject1_is_stopped(void)
{
    return (fabsf(kart_control_get_left_meas())  < KART_S1_STOP_SPEED_EPS &&
            fabsf(kart_control_get_right_meas()) < KART_S1_STOP_SPEED_EPS) ? 1 : 0;
}

/* 复刻/倒库期间遥控急停钩子:遥控失联(超时或接收机失控)或三段拨到低挡 → 请求急停。
 * 只读通道判定,不读油门/方向,故遥控不会干扰复刻的转向和速度(用户需求:急停保留、
 * 油门转向不干扰)。注意:此判据把遥控当作 deadman,失联=急停,故复刻/倒库期间
 * 遥控必须保持开机且三段不在低挡,否则立即停车回 IDLE。 */
static uint8 subject1_estop_requested(void)
{
    return (!kart_remote_is_online() ||
            kart_remote_get_sw3() == KART_REMOTE_SW3_L) ? 1 : 0;
}

static void subject1_loop(void)
{
    /* 有动力输出的两个阶段(绕桩复刻 / 直线倒库)接受遥控急停:命中即完整停机回 IDLE。 */
    if((subject1_stage == S1_CONE_ROUTE || subject1_stage == S1_REVERSE_IN) &&
       subject1_estop_requested())
    {
        kart_mission_set_mode(MISSION_IDLE);
        return;
    }

    switch(subject1_stage)
    {
        case S1_WAIT_START:
            /* 等 START 键下降沿。触发后启动 playback 复现现场手推的绕桩轨迹
             * (绕桩方向已烘在轨迹里)。playback_start 内部会开速度环+航向外环。 */
            if(subject1_start_pressed())
            {
                if(kart_playback_start())
                {
                    subject1_stage = S1_CONE_ROUTE;
                }
                else
                {
                    /* 空路径或加载失败绝不能继续进入倒库流程。 */
                    mission_stop_all();
                    subject1_stage = S1_FAULT;
                }
            }
            break;

        case S1_CONE_ROUTE:
            /* 绕桩复现:实际跟踪由主循环的 kart_playback_poll() 执行,这里只等它跑完。
             * playback 到终点(A 点附近)会自动 stop → is_running 变 0,车随即停稳。 */
            if(!kart_playback_is_running())
            {
                subject1_stage = S1_GARAGE_APPROACH;
            }
            break;

        case S1_GARAGE_APPROACH:
            /* A 点:等车彻底停稳,再切倒车。playback_stop 已关速度环+航向外环。
             * 停稳后记倒车里程基准,准备直线倒库。 */
            if(subject1_is_stopped())
            {
                subject1_reverse_dist0 = kart_odom_get_dist();

                /* 方向物理回中 + 只开转角内环(不开航向外环:倒车时航向环是正反馈)。
                 * 内环把方向盘按在中位,速度环给负速直线倒。 */
                kart_steer_set_head_enable(0);
                kart_steer_set_angle_enable(1);
                kart_steer_set_target_delta(0.0f);

                kart_control_set_enable(1);
                kart_control_set_target(KART_S1_REVERSE_SPEED);

                subject1_stage = S1_REVERSE_IN;
            }
            break;

        case S1_REVERSE_IN:
        {
            /* 倒车里程增量(dist_sum 用 fabs 累加,倒车也往上加)。 */
            float d = kart_odom_get_dist() - subject1_reverse_dist0;

            /* 全程保持方向回中(内环持续按住中位,抵抗地面扰动)。 */
            kart_steer_set_target_delta(0.0f);

            if(d >= KART_S1_REVERSE_STOP_DIST)
            {
                /* 到停车点:完整停机收车。 */
                kart_control_set_target(0.0f);
                kart_control_set_enable(0);
                kart_steer_set_angle_enable(0);
                subject1_stage = S1_FINISHED;
            }
            else if(d >= KART_S1_REVERSE_SLOW_DIST)
            {
                /* 过减速点:降到慢速轻靠库底。 */
                kart_control_set_target(KART_S1_REVERSE_SLOW_SPEED);
            }
            break;
        }

        case S1_FINISHED:
            /* 停车结束:保持无输出。停机已在进入时做完,这里不重复下发。 */
            break;

        case S1_FAULT:
            /* 故障锁止:不产生任何运动输出。 */
            break;

        default:
            break;
    }
}

static void subject2_loop(void)
{
    /* 科目二人车交互:语音收帧解析入队 → 分发。
     * 鸣笛节拍机已迁移至 CCU60_CH1 独立1ms中断,不再需要主循环轮询。 */
    kart_voice_poll();
    kart_voice_dispatch();

    /* 运动动作状态机一拍:推进当前动作 + deadman 急停(idle 时空操作)。
     * 实际转向/速度 PWM 仍由主循环控制环产生,本调用只设目标+判完成。 */
    kart_motion_update();
}

static void remote_loop(void)
{
    /* SBUS 遥控接管:每拍按通道驱动转向内环 + 速度环。
     * 内部处理失联急停、三段挡位总闸、方向硬钳软限位(见 kart_remote.c 安全设计)。
     * 转向内环的实际 PWM 输出仍由主循环的 kart_steer_ctrl_update() 产生,
     * 本函数只负责设目标转角/速度和使能。 */
    kart_remote_control_update();
}

static void subject4_loop(void)
{
    /* 开环倒车阶段接受遥控急停:命中即完整停机回 IDLE(全程 deadman 保护)。
     * 录制阶段的急停由 remote_control_update 内部处理(遥控本身即 deadman)。 */
    if(subject4_stage == S4_PHASE2_REVERSE && subject1_estop_requested())
    {
        kart_mission_set_mode(MISSION_IDLE);
        return;
    }

    switch(subject4_stage)
    {
        case S4_PHASE1_RECORD:
            /* 第一阶段:遥控走迷宫 + 录制(同科目三)。转向/速度由遥控接管,
             * 录制采样 + 同帧打角/里程由 CPU2 的 kart_record_poll 每拍做。
             * 人开到停车区后按 MID/START 键 → 停录,同一按键直接触发开环倒车:
             * 车头不掉转,不搬车,不掉头,odom 不 reset(倒车里程接着录制里程算)。 */
            kart_remote_control_update();

            if(subject1_start_pressed())
            {
                kart_record_stop();
                /* 不调 kart_remote_control_stop():它第一行强制 sw3=L,而本函数在同一
                 * 次按键里紧接着启动开环倒车,下一个 5ms 拍 kart_playback_poll 的 deadman
                 * (sw3==L 即停)会读到这个假 L,在 kart_remote_poll(10ms 隔拍)恢复真实
                 * 挡位前就把倒车杀掉 → 车只动 1 拍等于不动。科目三因两次按键间有搬车间隔,
                 * poll 早已恢复 sw3 故无此问题。start_openloop_reverse 已完整设置转向内环
                 * (angle_enable+目标打角)与速度环(enable+OL_SPEED),无需再 stop 遥控;
                 * 失败分支有 mission_stop_all() 兜底。
                 * 开环倒车:按里程回放录制打角。odom 保持 active(不 reset),
                 * start 内部取当前 odom 里程作倒车基准 dist0。 */
                if(kart_playback_start_openloop_reverse())
                {
                    subject4_stage = S4_PHASE2_REVERSE;
                }
                else
                {
                    /* 路径无效(点数<2):绝不进入倒车,直接故障锁止。 */
                    mission_stop_all();
                    subject4_stage = S4_FAULT;
                }
            }
            break;

        case S4_PHASE2_REVERSE:
            /* 第二阶段:开环倒车原路返回发车区。实际打角/速度由主循环
             * kart_playback_poll() 的开环分支执行(按里程查表回放打角),
             * deadman(遥控失联/低挡)在 poll 入口判。返回发车区(剩余里程<阈值)
             * 或查到起点自动 stop → is_running 变 0,车停稳,返回完成。 */
            if(!kart_playback_is_running())
            {
                subject4_stage = S4_FINISHED;
            }
            break;

        case S4_FINISHED:
            /* 返回完成:保持无输出。停机已在 playback 跑完时做完。 */
            break;

        case S4_FAULT:
            /* 故障锁止:不产生任何运动输出。 */
            break;

        default:
            break;
    }
}

/* ============ 对外接口 ============ */
void kart_mission_init(void)
{
    gpio_init(BOARD_START_KEY_PIN, GPI, 0, GPI_PULL_UP);
    mission_mode   = MISSION_IDLE;
    subject1_stage = S1_WAIT_START;
    mission_stop_all();                 /* 上电即保证无残留输出 */
}

void kart_mission_set_mode(kart_mission_mode_t mode)
{
    if(mode == mission_mode)
    {
        return;                         /* 同模式不重复切 */
    }

    /* 统一 exit:任何模式退出都执行完整停机(清运动+蜂鸣器+队列)。 */
    mission_stop_all();

    /* 退出科目二:把共用串口还给 VOFA 日志(切回 460800)再开闸。
     * 放在 mission_enter 之前,避免与新模式的 enter 抢同一外设。
     * 顺序与进入时相反:先切波特率、后开闸 —— 开闸后立刻可能发字节,
     * 此时外设必须已经是 460800,否则第一批帧以 115200 发出会是乱码。 */
    if(MISSION_SUBJECT_2 == mission_mode)
    {
        kart_light_set_command(KART_LIGHT_CMD_OFF);   /* 灯板图案归零,屏幕交回模式号显示 */
        kart_voice_uart_release();
#if BOARD_VOICE_SHARES_AUX_UART
        kart_debug_uart_set_enabled(1);
#endif
    }

    mission_mode = mode;
    mission_enter(mode);
}

void kart_mission_poll(void)
{
    switch(mission_mode)
    {
        case MISSION_SUBJECT_1:
            subject1_loop();
            break;
        case MISSION_SUBJECT_2:
            subject2_loop();
            break;
        case MISSION_SUBJECT_4:
            subject4_loop();
            break;
        case MISSION_REMOTE:
            remote_loop();
            break;
        case MISSION_IDLE:
        case MISSION_FAULT:
        default:
            /* 待机/故障:不做任何输出。后轮由主循环速度环门控清零。 */
            break;
    }
}

kart_mission_mode_t kart_mission_get_mode(void)
{
    return mission_mode;
}

kart_subject1_stage_t kart_mission_get_subject1_stage(void)
{
    return subject1_stage;
}

kart_subject4_stage_t kart_mission_get_subject4_stage(void)
{
    return subject4_stage;
}
