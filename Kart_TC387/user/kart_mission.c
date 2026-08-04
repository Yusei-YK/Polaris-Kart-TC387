#include "kart_mission.h"
#include "kart_control.h"
#include "kart_flash.h"     /* KART_FLASH_S2R_FIRST_SLOT / _SLOT_NUM:返程槽号映射 */
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
#include "kart_params.h"
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
static kart_subject2_stage_t subject2_stage = S2_IDLE;
/* 语音返回目标槽(6~10)。只在 S2_RETURN_* 期间有意义。 */
static uint8 subject2_return_slot = 0;
/* GOTO 阶段超时兜底拍数(10ms/拍)。GOTO 自己也有超时,这里是第二道闸,
 * 阈值见 KART_S2_RETURN_GOTO_TICKS。由 subject2_loop 的 S2_RETURN_GOTO 分支累加。 */
static uint16 subject2_return_ticks = 0;
/* 返程模式:0=Voice A(自动 GOTO),1=Voice B(空闲时遥控接管,返回走方案B)。
 * 进场前由菜单定死,进场后不再改(不能碰板子)。切模式不清它。 */
static uint8 subject2_manual_return = 0;

/* 倒库里程基准:进 S1_REVERSE_IN 时记下当前累计路程,增量到阈值判停。 */
static float subject1_reverse_dist0 = 0.0f;
/* START 键上一拍电平(下降沿检测:上拉输入,按下 1→0)。 */
static uint8 subject1_start_key_last = 1;
/* 科目四自动停录:连续判到"车停住"的拍数(10ms/拍),达阈值即停录并开倒车。 */
static uint16 subject4_autostop_hold = 0;

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

            /* 2026-07-29 全局坐标系:整个科目二【只在这里清一次】odom。
             * 从此刻起 odom (x,y) 就是"发车区坐标系"里的绝对位置,车跑完
             * 门洞 + 一串随机语音动作之后依然知道发车区在 (0,0) ——
             * 语音返回(0x1A~0x1E)的 GOTO 就是靠这个坐标系找路的。
             * 原来每条门洞命令都 reset 一次(kart_voice.c),那等于每过一个洞
             * 就把发车区原点擦掉,返回功能根本没有立足点。
             * 【现场纪律】进"Voice Control"这一刻车必须已经停在发车区标记点、
             * 车头摆正 —— 这一拍的位姿就是全场的基准,摆歪多少全场偏多少。
             * yaw 不清(kart_odom_reset 本来就不动 yaw):IMU 上电标定后连续,
             * 实测漂 0.009°/s,4 分钟约 2°,比清零重新起算更可靠。 */
            subject2_stage = S2_IDLE;
            subject2_return_slot = 0;
            kart_odom_reset();
            kart_odom_set_active(1);
            break;

        case MISSION_SUBJECT_4:
            /* 科目四第一阶段:与科目三录制完全相同(遥控走迷宫 + 录制)。
             * 区别只在返程:科目四不掉头,停车区判停后直接倒车原路返回。
             *   ① odom 清零(录制起点车体系对齐);② 开录制(CPU2 采样,同帧记打角+里程);
             *   ③ 遥控急停一次防残留,之后 remote_control_update 接管。
             * 注意:倒车靠 odom(里程或位置)映射,故录制到倒车全程 odom 不再 reset。 */
            subject4_stage = S4_PHASE1_RECORD;
            subject4_autostop_hold = 0;
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
#if KART_S1_AUTO_REVERSE
                subject1_stage = S1_GARAGE_APPROACH;
#else
                /* 倒车入库已在录制轨迹里,playback 跑完就是全程跑完。
                 * playback_stop 已关速度环+航向环+转角内环,这里不必再收车。 */
                subject1_stage = S1_FINISHED;
#endif
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
                kart_control_set_target(kart_params_get(KART_PARAM_S1_REV_SPD));

                subject1_stage = S1_REVERSE_IN;
            }
            break;

        case S1_REVERSE_IN:
        {
            /* 倒车里程增量(dist_sum 用 fabs 累加,倒车也往上加)。 */
            float d = kart_odom_get_dist() - subject1_reverse_dist0;

            /* 停车里程走菜单(S1 RevStop):赛前量库位深浅不用重烧。
             * 减速点仍用宏,但钉在停车点前 0.1m 以内,防停车点调得比减速点
             * 还近时"永远不降速直接撞库底"。 */
            float stop_d = kart_params_get(KART_PARAM_S1_REV_STOP);
            float slow_d = KART_S1_REVERSE_SLOW_DIST;

            /* 全程保持方向回中(内环持续按住中位,抵抗地面扰动)。 */
            kart_steer_set_target_delta(0.0f);

            if(slow_d > stop_d - 0.10f) slow_d = stop_d - 0.10f;

            if(d >= stop_d)
            {
                /* 到停车点:完整停机收车。 */
                kart_control_set_target(0.0f);
                kart_control_set_enable(0);
                kart_steer_set_angle_enable(0);
                subject1_stage = S1_FINISHED;
            }
            else if(d >= slow_d)
            {
                /* 过减速点:降到慢速轻靠库底。 */
                /* 【必须取 min】慢速幅值不能超过巡航幅值,否则"减速段"反而是加速:
                 * 07-28 现场把 S1 RevSpd 调到 -5,而 SLOW_SPEED 写死 -12,
                 * 越靠近库底跑得越快。取 min 后 RevSpd 调多小都不会被这里反超。 */
                float rev  = kart_params_get(KART_PARAM_S1_REV_SPD);     /* 负 */
                float slow = KART_S1_REVERSE_SLOW_SPEED;                 /* 负 */
                if(slow < rev) slow = rev;   /* 均负:比 rev 更负=更快 → 钳成 rev */
                kart_control_set_target(slow);
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

    /* -------- Voice B:空闲拍遥控接管 --------
     * 比赛里按一次 Voice 就不能再碰板子了,所以"人工摆位"必须能在【同一次语音
     * 会话里】做完。判据取"三件事都不忙":运动动作、复现、以及返程时序不在跑。
     * 忙的时候一拍都不能下发遥控目标 —— 否则同一拍里遥控和状态机抢同一个
     * 转向/速度目标,后写的赢,固定动作会被摇杆带跑偏。
     * 交接是无缝的:固定动作一结束(motion 不忙)下一拍遥控就自动活,
     * 人开车回集结点,喊返回口令后 dispatch 走方案B,车归复现,遥控自动让位。 */
    if(subject2_manual_return &&
       subject2_stage == S2_IDLE &&
       !kart_motion_is_busy() &&
       !kart_playback_is_running())
    {
        kart_remote_control_update();
    }

    /* -------- 语音返回的两步交接(只有方案A 会进这些分支) --------
     * 放在 kart_motion_update() 【之后】:要看的是本拍推进完的最新状态,
     * 否则 GOTO 判 DONE 那一拍会被推迟一拍才交接。 */
    switch(subject2_stage)
    {
        case S2_RETURN_GOTO:
        {
            kart_motion_goto_state_t gs = kart_motion_get_goto_state();

            subject2_return_ticks++;

            if(gs == KART_MOTION_GOTO_DONE)
            {
                /* 【必须同时等 !is_busy()】GOTO 判 DONE 之后还有一段 MOTION_CENTER
                 * 原地回正(见 kart_motion.c 的 motion_finish)。回正期间 is_busy=1,
                 * 此时启动复现 = 两个模块同时写转向目标,方向盘会被抢。
                 * 等回正结束再交接,还顺带保证复现是从"方向盘在中位"起步的。 */
                if(!kart_motion_is_busy())
                {
                    /* 【这里用按录制原点启动,与方案B 的按当前位姿相反】
                     * GOTO 是靠 odom 把车开到"odom 认为的集结点"上的,所以此刻
                     * odom 位姿与录制原点【同系且已对齐】,按录制原点启动才能让
                     * Pure Pursuit 看见真实的残余横向偏差(GOTO 交接预算 ±0.5m),
                     * 靠门洞前的直线引入段把它压掉。若按当前位姿启动,整条路径连
                     * 门洞入口一起跟着这 0.5m 平移,前视再长也没用。 */
                    if(kart_playback_start_at_recorded_origin())
                    {
                        subject2_return_ticks = 0;
                        subject2_stage = S2_RETURN_PLAYBACK;
                    }
                    else
                    {
                        /* 载入过的槽这里不该失败(点数已校验过);真失败就停机,
                         * 不留"以为在返回其实没动"的中间态。 */
                        mission_stop_all();
                        subject2_stage = S2_RETURN_FAULT;
                    }
                }
            }
            else if(gs != KART_MOTION_GOTO_RUNNING ||
                    subject2_return_ticks >= KART_S2_RETURN_GOTO_TICKS)
            {
                /* FAULT/NONE(被 deadman 或切模式硬停)或第二道超时闸到点。
                 * 【绝不能继续】摆位没成 => 位姿未知 => 放出复现就是按错位姿全速
                 * 跑十几米撞门洞桩。停机等人接管,改用方案B。 */
                mission_stop_all();
                subject2_stage = S2_RETURN_FAULT;
            }
            break;
        }

        case S2_RETURN_PLAYBACK:
            /* 复现由主循环的 kart_playback_poll() 执行(5ms 拍),这里只等它跑完。
             * 到终点 playback 自己 stop → is_running 变 0,车随即停稳。
             * 方案B 直接进这个阶段,所以这段两个方案共用。 */
            if(!kart_playback_is_running())
            {
                subject2_stage = S2_RETURN_DONE;
            }
            break;

        case S2_RETURN_DONE:
        case S2_RETURN_FAULT:
            /* 终态:停在这里不自动回 IDLE。
             * 【为什么不自动回】回了就再也看不出上一次返回是成了还是废了 ——
             * 菜单/调试通道要能读到这个结果(kart_mission_get_subject2_stage)。
             * 下一次进科目二(mission_enter)会清回 S2_IDLE。 */
            break;

        case S2_IDLE:
        default:
            break;
    }
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
    /* 倒车阶段接受遥控急停:命中即完整停机回 IDLE(全程 deadman 保护)。
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
             * 结束不用按键:人把车开直回正再停住,车停 150ms 即自动停录并开倒车。
             * START 键保留为手动提前触发(不想等那 150ms 时用)。
             * 车头不掉转,不搬车,不掉头,odom 不 reset(倒车里程接着录制里程算)。 */
            kart_remote_control_update();

            /* 自动停录判据:已走够 ARM_DIST(防发车前静止误触发)+ 左右轮测速幅值
             * 连续低于 EPS 达 HOLD 拍。测速走的是 8 拍滑动平均,本身已滤过抖动。 */
            {
                float vl = kart_control_get_left_meas();
                float vr = kart_control_get_right_meas();

                if(vl < 0.0f) vl = -vl;
                if(vr < 0.0f) vr = -vr;

                if(kart_odom_get_dist() >= KART_S4_AUTOSTOP_ARM_DIST
                   && vl < KART_S4_AUTOSTOP_SPEED_EPS
                   && vr < KART_S4_AUTOSTOP_SPEED_EPS)
                {
                    subject4_autostop_hold++;
                }
                else
                {
                    subject4_autostop_hold = 0;
                }
            }

            if(subject1_start_pressed()
               || subject4_autostop_hold >= KART_S4_AUTOSTOP_HOLD_TICKS)
            {
                uint16 n = kart_record_get_count();

                kart_record_stop();

                if(n < 2)
                {
                    /* 路径无效(点数<2):绝不进入倒车,直接故障锁止。 */
                    mission_stop_all();
                    subject4_stage = S4_FAULT;
                    break;
                }

                /* 同一拍直接开倒车,不再插过渡等待:判停条件本身已含"连续 150ms
                 * 车速≈0",车此刻确实停稳,odom 基准不含滑行余量;打角也已由人
                 * 在停车前回正,录制末点打角≈0,起步不带角度。
                 * 不调 kart_remote_control_stop():它第一行强制 sw3=L,而
                 * kart_playback_poll 的 deadman(sw3==L 即停)会读到这个假 L
                 * 把刚启动的倒车杀掉。 */
                /* 2026-07-30:先失能清速度环积分。前进段 KI=0.8 攒的正积分会反抗
                 * 倒车负目标,拖慢起步。下面 start_openloop_reverse() 同拍会重新
                 * set_enable(1),只多断一拍后轮 PWM;转向使能/目标角在它里面且顺序
                 * 在后,不受影响。 */
                kart_control_set_enable(0);

                if(kart_playback_start_openloop_reverse())
                {
                    subject4_stage = S4_PHASE2_REVERSE;
                }
                else
                {
                    mission_stop_all();
                    subject4_stage = S4_FAULT;
                }
            }
            break;

        case S4_PHASE2_REVERSE:
            /* 第二阶段:倒车原路返回发车区。实际打角/速度由主循环
             * kart_playback_poll() 的倒车分支执行(菜单 S4 OLMode 选索引方式:
             * 0=里程查表 1=最近点+横向位置闭环),
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
    /* 发车键(按键板 SW3,P20.7)。主板侧有上拉,按下接地读 0,故浮空输入即可
     * (与 kart_menu_init 里其余按键口一致)。
     * 2026-07-28 换按键板后 START 已是独立引脚,不再与菜单 MID(P33.4)共用,
     * 也就不存在过去 menu/mission 对同一脚配两种模式、谁后 init 谁生效的问题。 */
    gpio_init(BOARD_START_KEY_PIN, GPI, 0, GPI_FLOATING_IN);
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

kart_subject2_stage_t kart_mission_get_subject2_stage(void)
{
    return subject2_stage;
}

void kart_mission_subject2_set_manual_return(uint8 en)
{
    subject2_manual_return = en ? 1 : 0;
}

uint8 kart_mission_subject2_get_manual_return(void)
{
    return subject2_manual_return;
}

/* 载入返程槽,并把 GOTO 的目标位姿(= 该槽的录制起点 = 集结点)取出来。
 * 两个方案共用这一步。返回 0 = 槽空/点数不足,调用方不得继续。
 * 【必须先 load 再取 origin】origin_x/y/yaw 是 kart_record 的模块内静态量,
 * 只有 kart_record_load_from_flash() 从 Flash 页头还原之后才是这条路径的起点;
 * 不 load 就取到的是【上一次录制或上一次载入】的起点 —— 那会让 GOTO 开去别的
 * 集结点,而且看不出错。 */
static uint8 subject2_load_return_slot(uint8 slot_index, float *tx, float *ty, float *tyaw)
{
    uint8 slot;

    if(slot_index >= KART_FLASH_S2R_SLOT_NUM) return 0;

    slot = (uint8)(KART_FLASH_S2R_FIRST_SLOT + slot_index);
    if(kart_record_load_from_flash(slot) < 2) return 0;

    *tx   = kart_record_get_origin_x();
    *ty   = kart_record_get_origin_y();
    *tyaw = kart_record_get_origin_yaw();
    subject2_return_slot = slot;
    return 1;
}

/* ================== 方案A:自动返回(GOTO 摆位 + 复现) ==================
 * 语音返回口令 0x1A~0x1E 走这里。两步:
 *   ① GOTO 把车从"固定动作结束后的任意位姿"摆到集结点(= 返程路径录制起点);
 *   ② GOTO 判 DONE 后,按【录制原点】启动复现,穿门洞回停车区。
 * 交接由 subject2_loop 每拍推进(见 S2_RETURN_GOTO 分支)。
 *
 * 【必须知道的风险 —— 这个方案离线仿真只有 261/304】
 * 失败的 43 个位姿全是"绕着前置点画圈直到超距保护",根因是阿克曼车不能原地转
 * (满舵 R=1.32m),前置点落进最小转弯圆里就出不来。这是几何硬限制,换控制律
 * 也一样(见 KART_MOTION_GOTO_LEAD 的注释)。失败时 GOTO 判 FAULT 停机,
 * 【不会】放出复现 —— 也就是说最坏情况是"车停在场地中间不动",不是撞桩。
 * 那时改用方案B:遥控把车开回集结点,喊/按方案B 的入口直接复现。
 *
 * 返回 0 = 未受理(参数越界/正忙/槽空),不改任何状态,车保持原样。 */
uint8 kart_mission_subject2_start_return(uint8 slot_index)
{
    float tx, ty, tyaw;

    /* 正忙就不受理:两个模块同时写执行机构 = 打角和速度各说各话。人重喊一次即可。 */
    if(kart_motion_is_busy() || kart_playback_is_running()) return 0;

    if(!subject2_load_return_slot(slot_index, &tx, &ty, &tyaw)) return 0;

    if(!kart_motion_start_goto(tx, ty, tyaw)) return 0;

    subject2_return_ticks = 0;
    subject2_stage = S2_RETURN_GOTO;
    return 1;
}

/* ================== 方案B:人工摆位后直接复现(兜底) ==================
 * 用法:遥控把车开回集结点、车头照着去程方向摆正,然后调这里。不跑 GOTO。
 *
 * 【为什么要有它】方案A 的 261/304 不是调参问题而是几何上限,场上抽到那 14%
 * 就只能停在场地里。有这条兜底 = 最坏情况从"这一项拿不到分"变成"慢一点但拿到"。
 *
 * 【为什么用 kart_playback_start() 而不是 ..._start_at_recorded_origin()】
 * 按录制原点启动要求 odom 世界系与录制时同一系 —— 车是被人开回来的,一路的
 * 里程/航向积分误差都算在 odom 上,此刻 odom 认为的"集结点"未必是真集结点;
 * 而人眼看到的车【确实】在集结点上。这时该信人眼:按当前位姿启动,路径原点
 * 就钉在车现在站的地方,等于把 odom 的累计误差一次性归零。
 * 代价:摆位误差直接变成整条路径的平移误差,所以人必须摆正 —— 这是本方案成立
 * 的前提。这条链路与去程五条路完全同构,已经实车跑通。
 *
 * 返回 0 = 未受理(正忙/槽空/点数不足)。 */
uint8 kart_mission_subject2_start_return_here(uint8 slot_index)
{
    float tx, ty, tyaw;

    if(kart_motion_is_busy() || kart_playback_is_running()) return 0;

    /* tx/ty/tyaw 这里用不上(不跑 GOTO),但仍走同一个载入函数:
     * 载入这一步才是关键,顺带保证两个方案的槽号映射永远一致。 */
    if(!subject2_load_return_slot(slot_index, &tx, &ty, &tyaw)) return 0;
    (void)tx; (void)ty; (void)tyaw;

    if(!kart_playback_start()) return 0;

    subject2_return_ticks = 0;
    subject2_stage = S2_RETURN_PLAYBACK;
    return 1;
}
