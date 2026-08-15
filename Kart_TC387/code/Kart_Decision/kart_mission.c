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
#include "kart_imu.h"
#include "kart_remote.h"
#include "kart_debug_uart.h"
#include "kart_light.h"
#include "kart_params.h"
#include "board_pins.h"
#include "kart_camera.h"        /* 科目三视觉跟随:取帧/还帧 */
#include "kart_vision.h"        /* 科目三视觉跟随:黄色引导板检测 */
#include "kart_follow.h"        /* 科目三视觉跟随:纯跟踪控制律 */
#include "kart_person_link.h"   /* 科目三 PLINK 源:TC4D7 人体跟踪帧 → vtrack */
#include "kart_preprocess.h"    /* 图像预处理:ROI裁剪+降噪+直方图均衡 */
#include "isr.h"                /* 切模式清调度器最坏值 g_sched_* */
#include "kart_multicore.h"    /* 科目三视觉搬到 core3 异步跑 */
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
static kart_subject3_stage_t subject3_stage = S3_PHASE1_FOLLOW;
static kart_subject2_stage_t subject2_stage = S2_IDLE;
/* 语音返回目标槽(6~10)。只在 S2_RETURN_* 期间有意义。 */
static uint8 subject2_return_slot = 0;
/* GOTO 阶段超时兜底拍数(10ms/拍)。GOTO 自己也有超时,这里是第二道闸,
 * 阈值见 KART_S2_RETURN_GOTO_TICKS。由 subject2_loop 的 S2_RETURN_GOTO 分支累加。 */
static uint16 subject2_return_ticks = 0;
/* 返程模式:0=Voice A(自动 GOTO),1=Voice B(空闲时遥控接管,返回走方案B)。
 * 进场前由菜单定死,进场后不再改(不能碰板子)。切模式不清它。 */
static uint8 subject2_manual_return = 0;

/* START 键上一拍电平(下降沿检测:上拉输入,按下 1→0)。 */
static uint8 subject1_start_key_last = 1;
/* 科目三语音信号阶段计时(10ms/拍),超时收车。 */
static uint16 subject3_signal_ticks = 0;
static uint8 subject3_vision_prev_valid = 0;
static uint8 subject3_camera_prev_ok = 1;
static uint8 subject3_vision_prev_reject = 0;
static uint8 subject3_follow_prev_near = 0;
static uint8 subject3_follow_prev_sat = 0;
static int8  subject3_follow_prev_scale = 0;
static kart_follow_state_enum subject3_follow_prev_state = KART_FOLLOW_IDLE;
/* 视觉快照年龄(拍):有新帧清零,否则每拍 +1 封顶在 MAX_AGE。 */
static uint16 s3_vision_age_ticks = KART_S3_VISION_MAX_AGE_TICKS;
static uint32 s3_vision_seq = 0;
/* 上一次收割到的 core3 结果帧号,用来判"这一拍是不是有新结果"。 */
static uint32 s3_vision_frames_seen = 0;
/* 科目三语音信号阶段是否已经至少执行过一条口令(菜单显示用)。 */
static uint8  subject3_signal_done = 0;

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

        case MISSION_SUBJECT_3:
            /* 科目三第一阶段:跟着人走一段并同时录轨。控制源二选一(编译期):
             *   SRC_REMOTE = 遥控接管(省赛已验证);SRC_VISION = 视觉跟随引导板。
             * 返程与省赛完全一致:车头不掉转,判停后直接倒车原路返回。
             *   ① odom 清零(录制起点车体系对齐);② 开录制(CPU2 采样,同帧记打角+里程);
             *   ③ 控制源初始化。
             * 注意:倒车靠 odom(里程或位置)映射,故录制到倒车全程 odom 不再 reset。 */
#if (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_PLINK)
            /* 4D7 只服务科目三：到这里才开 RX 中断，普通菜单阶段完全不收数据。 */
            kart_person_link_set_rx_enabled(1U);
#endif
            subject3_stage = S3_PHASE1_FOLLOW;
            subject3_signal_ticks  = 0;
            subject3_signal_done   = 0;
            subject1_start_key_last = gpio_get_level(BOARD_START_KEY_PIN);
            kart_odom_reset();
            kart_odom_set_active(1);
            kart_debug_uart_set_event(KART_EVENT_S3_ENTER, KART_EVENT_LEVEL_INFO);
            subject3_vision_prev_valid = 0;
            subject3_camera_prev_ok = 1;
            subject3_vision_prev_reject = 0;
            subject3_follow_prev_near = 0;
            subject3_follow_prev_sat = 0;
            subject3_follow_prev_scale = 0;
            subject3_follow_prev_state = KART_FOLLOW_IDLE;
            s3_vision_age_ticks = KART_S3_VISION_MAX_AGE_TICKS;
            s3_vision_seq = 0;
            s3_vision_frames_seen = kart_multicore_vision_frames();
            kart_record_start();
#if KART_S3_FOLLOW_IS_AUTO
            /* 自动源（VISION 或 PLINK）共用的进场动作。【条件从 VISION 改成 IS_AUTO】
             * 里面没有一行是摄头特有的：都是“车要自己跑”所需的使能与增益组。
             * 清跟随控制律的内部状态(上一拍速度/丢失计数),
             * 再开转向内环 + 速度环 —— 跟随【不开航向外环】:
             * 目标方位角本身就是相对车头的,纯跟踪直接出打角给内环,
             * 再套一层"锁定某个绝对航向"的外环会两个环打架。 */
            kart_follow_reset();
#if (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_VISION)
            /* Detector 的内部状态只有 VISION 源用得上。PLINK 源的“复位”就是
             * kart_follow_reset()：链路本身无状态（失联计时器由 poll 自己维护）。 */
            kart_vision_reset();
#endif
            /* 【必须切回前进增益组】视觉跟随是前进段,但这条入口原来一次都没切过组,
             * 于是内环沿用"上一次动作留下的"那组。上一次若是倒车复现(科目二/科目三
             * 返程都会切 use_back_gains),内环就还挂在 KP_BACK=5 上 —— 只有前进组
             * KP_DEFAULT=15 的 1/3,现象正是"舵机听得见响但推不动"(占空比出得来,
             * 克不住转向摩擦)。遥控源不犯这个病只是因为它进来前多半路过了
             * kart_motion.c 里那十处 use_fwd_gains 之一,属于蒙对,不是有人管。
             * 与 kart_motion.c 现有做法一致:谁要用哪组,自己进场时切。 */
            kart_steer_use_fwd_gains();
            kart_steer_set_head_enable(0);
            kart_steer_set_angle_enable(1);
            kart_steer_set_target_delta(0.0f);
            kart_control_set_target(0.0f);
            kart_control_set_enable(1);
#else
            kart_remote_control_stop();     /* 遥控急停一次防残留,之后 update 接管 */
#endif
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
    /* 路径复现期间遥控只作 deadman：失联或低挡立即停机回 IDLE。 */
    if(subject1_stage == S1_CONE_ROUTE && subject1_estop_requested())
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
                if(kart_playback_get_result() == KART_PLAYBACK_RESULT_COMPLETED)
                {
                    /* 倒车入库已在录制轨迹里，全程正常结束后自动交给遥控。 */
                    kart_mission_set_mode(MISSION_REMOTE);
                    return;
                }
                subject1_stage = S1_FAULT;
            }
            break;

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
                if(kart_playback_get_result() == KART_PLAYBACK_RESULT_COMPLETED)
                {
                    kart_mission_set_mode(MISSION_REMOTE);
                    return;
                }
                subject2_stage = S2_RETURN_FAULT;
            }
            break;

        case S2_RETURN_DONE:
        case S2_RETURN_FAULT:
            /* 返回结果保留在该状态，不自动回 IDLE；语音轮询和指令分发仍会继续。
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

#if (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_VISION)
/* ---- vision 结果 → vtrack 结果 ----
 * 【修一个先于 PLINK 就存在的错】原先这里直接 kart_follow_update(vis)，
 * 而 kart_follow.h:285 的入参是 const kart_vtrack_result_t *。两个结构体第三个
 * 字段一个是 float dist_m、一个是 enum scale_level：不报错（都是指针），但
 * follow 会把“米”当“档位”读，跟随行为鬼异。所以在这里显式换一道。
 *
 * scale_r 的参考距离取 KART_FOLLOW_L_NOMINAL_M(1.50m)：它在 kart_follow.h:135
 * 的注释里就是“期望跟车距离的中点”。仓里没有任何专门的“目标距离”宏，
 * 不新造一个；这个选择是判断，不是协议规定。
 * scale_r = L_NOMINAL / dist：人走近 → dist 小 → scale_r 大，与 vtrack 的
 * “像素尺寸比”同向，NEAR_THRESH/FAR_THRESH 两个阀值可直接用。
 * confidence：kart_vision_result_t 没有置信度字段，valid 就给满。
 * kart_follow.c 当前并不拿 confidence 做门限（只上报），不影响控制。 */
static const kart_vtrack_result_t *s3_vision_to_vtrack(const kart_vision_result_t *vis)
{
    static kart_vtrack_result_t vt;

    if((vis == NULL) || (!vis->valid))
    {
        vt.valid       = 0;
        vt.bearing_rad = 0.0f;
        vt.scale_level = KART_VTRACK_SCALE_NORMAL;
        vt.scale_r     = 1.0f;
        vt.confidence  = 0;
        vt.alive_count = 0;
        vt.total_count = 0;
        vt.fb_error_median = 0.0f;
        vt.frames_since_detector = (uint16)KART_VTRACK_MAX_FRAMES_SINCE_DET;
        return &vt;
    }

    vt.valid       = 1;
    vt.bearing_rad = vis->bearing_rad;      /* 两边同名同义：弧度，>0=右 */

    /* dist_m 可能为 0（框宽异常），除之前先护一下。 */
    if(vis->dist_m > 0.05f)
    {
        vt.scale_r = KART_FOLLOW_L_NOMINAL_M / vis->dist_m;
    }
    else
    {
        vt.scale_r = 1.0f;
    }

    if(vt.scale_r >= KART_VTRACK_SCALE_NEAR_THRESH)
    {
        vt.scale_level = KART_VTRACK_SCALE_TOO_NEAR;
    }
    else if(vt.scale_r <= KART_VTRACK_SCALE_FAR_THRESH)
    {
        vt.scale_level = KART_VTRACK_SCALE_TOO_FAR;
    }
    else
    {
        vt.scale_level = KART_VTRACK_SCALE_NORMAL;
    }

    vt.confidence  = vis->confidence;
    vt.alive_count = 1;
    vt.total_count = 1;
    vt.fb_error_median = 0.0f;
    vt.frames_since_detector = 0;
    return &vt;
}

/* ---- 科目三阶段1:视觉跟随一拍 ----
 * 链路:kart_camera_frame_ready() → kart_vision_process() → kart_follow_update()
 *       → 转向内环目标角 + 速度环目标速度,然后 kart_camera_frame_release()。
 *
 * 帧率与拍率不同步是【故意】允许的:相机约 30FPS(33ms/帧),本函数 10ms 一拍。
 * 没有新帧的拍不重算视觉,直接把上一拍的 follow 输出继续下发 —— 因为
 * kart_follow 内部对"同一个结果被读多次"是稳定的(纯函数式,无积分),
 * 而且丢目标计数按拍数算(LOST_TICKS 已按 10ms/拍 标定过,见 kart_follow.h)。
 */
static void subject3_vision_follow_tick(void)
{
    const kart_vision_result_t *vis = NULL;
    const kart_follow_out_t    *fo;

    /* ---- 投递:有新帧就整帧拷给 core3,立刻还帧,不等结果 ----
     * 【2026-08-15 改】原来在这一拍里同步跑 preprocess + vision_process。
     * 1.csv 实测:出新帧的行平均丢 34 拍(170ms),没新帧的行只丢 2.4 拍 ——
     * 整条流水线约 360ms,而它挂在 10ms 拍里。有效 tick 率被压到 26Hz,
     * 转向内环按 5ms 标定却 38ms 执行一次,1.12Hz 自激 = 乱打左右方向。
     * submit 只做一次整帧拷贝(几十 us)就返回,识别在 core3 上跑。 */
    if(kart_camera_frame_ready())
    {
        (void)kart_multicore_vision_submit((const uint16 *)scc8660_image[0],
                                           (int16)SCC8660_W, (int16)SCC8660_H);
        kart_camera_frame_release();    /* 拷完立刻还帧,别占着让相机丢帧 */
    }

    /* ---- 收割:core3 算完一帧才算"有新结果" ----
     * 年龄按【结果帧序号】涨,不再按图像帧。投递成功但 core3 还没算完的那些拍,
     * 年龄照常增加 —— 这才是 follow 眼里真实的信息新鲜度。 */
    if(kart_multicore_vision_frames() != s3_vision_frames_seen)
    {
        s3_vision_frames_seen = kart_multicore_vision_frames();
        vis = kart_multicore_vision_get();
        s3_vision_age_ticks = 0;
        s3_vision_seq++;
    }
    else
    {
        /* 本拍没有新帧:沿用上一次的视觉结果,但只在快照还新鲜时沿用。
         * 相机状态不足以兜底 —— RUNNING 由 VSYNC 判定,DMA 停了照样 RUNNING,
         * 那时 kart_vision_get() 一直交出 valid=1 的旧值,follow 拿几百毫秒前的
         * 方位角打方向,表现是照着已经不在那儿的目标转。超龄就当无目标,
         * 交给 follow 的丢失分支斜坡减速。 */
        if(s3_vision_age_ticks < KART_S3_VISION_MAX_AGE_TICKS)
        {
            s3_vision_age_ticks++;
        }

        if((kart_camera_state() == KART_CAM_STATE_RUNNING)
           && (s3_vision_age_ticks < KART_S3_VISION_MAX_AGE_TICKS))
        {
            vis = kart_multicore_vision_get();
        }
    }

    /* vis==NULL 或 vis->valid==0 时 follow 内部走丢失分支(先保持,再斜坡减速)。
     * 不能直接传 vis：入参类型是 kart_vtrack_result_t（见 s3_vision_to_vtrack 注释）。 */
    fo = kart_follow_update(s3_vision_to_vtrack(vis));

    /* 只在状态边沿记录；同一拍按故障/停车/识别原因的优先级只报一个。 */
    {
        uint8 valid = (uint8)((vis != NULL) && vis->valid);
        uint8 camera_ok = (uint8)(kart_camera_state() == KART_CAM_STATE_RUNNING);
        uint8 reject = (vis != NULL) ? vis->reject : 0U;
        uint16 event = 0U;
        uint8 level = KART_EVENT_LEVEL_INFO;

        if(!camera_ok && subject3_camera_prev_ok)
        {
            event = KART_EVENT_CAMERA_FAULT;
            level = KART_EVENT_LEVEL_ERROR;
        }
        else if(camera_ok && !subject3_camera_prev_ok)
        {
            event = KART_EVENT_CAMERA_RECOVERED;
        }
        else if(fo->near_stop && !subject3_follow_prev_near)
        {
            event = KART_EVENT_FOLLOW_NEAR_STOP;
            level = KART_EVENT_LEVEL_WARNING;
        }
        else if((fo->state == KART_FOLLOW_LOST) &&
                (subject3_follow_prev_state != KART_FOLLOW_LOST))
        {
            event = KART_EVENT_FOLLOW_LOST_STOP;
            level = KART_EVENT_LEVEL_WARNING;
        }
        else if((fo->state == KART_FOLLOW_TRACKING) &&
                ((subject3_follow_prev_state == KART_FOLLOW_LOST) ||
                 (subject3_follow_prev_state == KART_FOLLOW_HOLD)))
        {
            event = KART_EVENT_FOLLOW_RECOVERED;
        }
        else if(fo->saturated && !subject3_follow_prev_sat)
        {
            event = KART_EVENT_FOLLOW_STEER_SAT;
            level = KART_EVENT_LEVEL_WARNING;
        }
        else if(!fo->saturated && subject3_follow_prev_sat)
        {
            event = KART_EVENT_STEER_RECOVERED;
        }
        else if(valid && !subject3_vision_prev_valid)
        {
            event = KART_EVENT_VISION_ACQUIRED;
        }
        else if(!valid && subject3_vision_prev_valid)
        {
            if(reject == KART_VISION_REJ_AREA)       event = (vis->area_px == 0U) ? KART_EVENT_VISION_NO_TARGET : KART_EVENT_VISION_REJ_AREA;
            else if(reject == KART_VISION_REJ_WIDTH) event = KART_EVENT_VISION_REJ_WIDTH;
            else if(reject == KART_VISION_REJ_ASPECT)event = KART_EVENT_VISION_REJ_ASPECT;
            else if(reject == KART_VISION_REJ_FILL)  event = KART_EVENT_VISION_REJ_FILL;
            else                                     event = KART_EVENT_VISION_NO_TARGET;
            level = KART_EVENT_LEVEL_WARNING;
        }
        else if(!valid && reject != 0U && reject != subject3_vision_prev_reject)
        {
            if(reject == KART_VISION_REJ_AREA)       event = (vis->area_px == 0U) ? KART_EVENT_VISION_NO_TARGET : KART_EVENT_VISION_REJ_AREA;
            else if(reject == KART_VISION_REJ_WIDTH) event = KART_EVENT_VISION_REJ_WIDTH;
            else if(reject == KART_VISION_REJ_ASPECT)event = KART_EVENT_VISION_REJ_ASPECT;
            else if(reject == KART_VISION_REJ_FILL)  event = KART_EVENT_VISION_REJ_FILL;
            level = KART_EVENT_LEVEL_WARNING;
        }
        else if((fo->scale_level > 0) && (subject3_follow_prev_scale <= 0))
        {
            event = KART_EVENT_FOLLOW_TOO_FAR;
            level = KART_EVENT_LEVEL_WARNING;
        }

        if(event != 0U) kart_debug_uart_set_event(event, level);

        subject3_vision_prev_valid = valid;
        subject3_camera_prev_ok = camera_ok;
        subject3_vision_prev_reject = reject;
        subject3_follow_prev_near = fo->near_stop;
        subject3_follow_prev_sat = fo->saturated;
        subject3_follow_prev_scale = fo->scale_level;
        subject3_follow_prev_state = fo->state;
    }

    /* 下发:打角给转向内环,速度给速度环。
     * 单位:follow 输出的 target_v_pulse 已经是"脉冲/5ms",与 control 一致。 */
    kart_steer_set_target_delta(fo->target_delta);
    kart_control_set_target(fo->target_v_pulse);

}
#endif

#if (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_PLINK)
/* ---- 科目三阶段1：PLINK（TC4D7 人体视觉）跟随一拍 ----
 * 链路：4D7 推理 → 串口 25 字节帧 → uartX_rx_isr → kart_person_link_rx_callback()
 *       → kart_person_link_poll(10) 合成 vtrack → 本函数 kart_follow_update()
 *       → 转向内环目标角 + 速度环目标速度。
 *
 * 与 VISION 源的三个不同：
 *   ① 不碰相机、不取帧、不还帧——图像在 4D7 侧，387 只消费结论。
 *   ② 不需要“本拍有无新帧”分支——kart_person_link_vtrack() 永远返回最新快照，
 *      失联（>200ms 无 VALID 帧）时它自己把 valid 置 0，kart_follow 走丢失分支减速。
 *      也因此本函数无需 NULL 判断（接口承诺非空，见 kart_person_link.h:234）。
 *   ③ 链路帧率跟 4D7 推理走（约 30ms/帧），本函数 10ms 一拍，同一个结果会被
 *      连续读到 3 次——这与 VISION 源的情形一样，kart_follow 内部无积分，稳定。
 */
static void subject3_plink_follow_tick(void)
{
    const kart_vtrack_result_t *vt = kart_person_link_vtrack();
    const kart_follow_out_t    *fo;

    fo = kart_follow_update(vt);

    /* 下发：打角给转向内环，速度给速度环。
     * target_v_pulse 已经是“脉冲/5ms”，与 kart_control 同量纲，不再换算。
     * 转向内环的实际 PWM 仍由主循环的 kart_steer_ctrl_update() 产生。 */
    kart_steer_set_target_delta(fo->target_delta);
    kart_control_set_target(fo->target_v_pulse);

}
#endif

static void subject3_loop(void)
{
    /* 倒车阶段接受遥控急停:命中即完整停机回 IDLE(全程 deadman 保护)。
     * 遥控源的录制阶段,急停由 remote_control_update 内部处理(遥控本身即 deadman)。 */
    if(subject3_stage == S3_PHASE2_REVERSE && subject1_estop_requested())
    {
        kart_mission_set_mode(MISSION_IDLE);
        return;
    }

#if KART_S3_FOLLOW_IS_AUTO
    /* 自动源（VISION/PLINK）的阶段1【必须】补一道 deadman:这一段车是自己在动的,
     * 而遥控源那段的急停是藏在 remote_control_update 里的,自动源不调它,
     * 于是原本整个跟随过程没有任何人工急停通道 —— 误识别就没法叫停。
     * 【条件必须是 IS_AUTO 不能是 VISION】漏了 PLINK 就是少一道急停，
     * 而 PLINK 下 4D7 认错人、或共轨 5V 塔一下导致 4D7 重启，都需要这道门。
     * 现场纪律:举板子的人或另一人必须握着遥控,拨到 SW3=L 即全车停。
     * 遥控失联同样算急停(与科目一/倒车段判据完全一致)。 */
    if(subject3_stage == S3_PHASE1_FOLLOW && subject1_estop_requested())
    {
        kart_mission_set_mode(MISSION_IDLE);
        return;
    }
#endif

    switch(subject3_stage)
    {
        case S3_PHASE1_FOLLOW:
            /* 第一阶段:跟着人走一段 + 录轨。
             * 录制采样 + 同帧打角/里程由 CPU2 的 kart_record_poll 每拍做,两种控制源共用。
             * 车头不掉转,不搬车,不掉头,odom 不 reset(倒车里程接着录制里程算)。 */
#if (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_VISION)
            /* 视觉源:摄像头认引导板 → 纯跟踪出打角/速度。 */
            subject3_vision_follow_tick();
#elif (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_PLINK)
            /* PLINK 源：TC4D7 跑人体检测/跟踪，串口回传方位与远近档，387 只做运动控制。 */
            subject3_plink_follow_tick();
#else
            /* 遥控源(省赛已验证):转向/速度由遥控接管。 */
            kart_remote_control_update();
#endif

            /* 只有 START(P20.7) 下降沿才停止录制并开始倒车。
             * 车停住、视觉丢失和 MID 键都不再自动推进任务。 */
            if(subject1_start_pressed())
            {
                uint16 n = kart_record_get_count();

                kart_record_stop();

                if(n < 2)
                {
                    /* 路径无效(点数<2):绝不进入倒车,直接故障锁止。 */
                    kart_debug_uart_set_event(KART_EVENT_S3_ABORT, KART_EVENT_LEVEL_ERROR);
                    mission_stop_all();
                    subject3_stage = S3_FAULT;
                    break;
                }

                /* START 按下后同一拍停录并启动倒车。现场应先停稳、回正再按，
                 * 这样录制末点打角接近 0，倒车起步不带角度。
                 * 不调 kart_remote_control_stop():它第一行强制 sw3=L,而
                 * kart_playback_poll 的 deadman(sw3==L 即停)会读到这个假 L
                 * 把刚启动的倒车杀掉。 */
                /* 2026-07-30:先失能清速度环积分。前进段 KI=0.8 攒的正积分会反抗
                 * 倒车负目标,拖慢起步。下面 start_openloop_reverse() 同拍会重新
                 * set_enable(1),只多断一拍后轮 PWM;转向使能/目标角在它里面且顺序
                 * 在后,不受影响。 */
                kart_control_set_enable(0);

#if KART_S3_FOLLOW_IS_AUTO
                /* 自动源（VISION/PLINK）额外一步:把跟随的打角目标显式清零。
                 * 自动源是自己在设 target_delta 的,若最后一拍还偏着(比如人
                 * 走出画面前车正在转弯),那个角度会被倒车段带进去。
                 * 遥控源不需要这一步 —— 人停车前本来就会回正。
                 * start_openloop_reverse() 内部会重设转向使能与目标,顺序在后。 */
                kart_steer_set_target_delta(0.0f);
                kart_follow_reset();
#endif

                if(kart_playback_start_openloop_reverse())
                {
                    kart_debug_uart_set_event(KART_EVENT_S3_REVERSE_START, KART_EVENT_LEVEL_INFO);
                    subject3_stage = S3_PHASE2_REVERSE;
                }
                else
                {
                    kart_debug_uart_set_event(KART_EVENT_S3_ABORT, KART_EVENT_LEVEL_ERROR);
                    mission_stop_all();
                    subject3_stage = S3_FAULT;
                }
            }
            break;

        case S3_PHASE2_REVERSE:
            /* 第二阶段:倒车原路返回发车区。实际打角/速度由主循环
             * kart_playback_poll() 的倒车分支执行(菜单 S3 OLMode 选索引方式:
             * 0=里程查表 1=最近点+横向位置闭环),
             * deadman(遥控失联/低挡)在 poll 入口判。返回发车区(剩余里程<阈值)
             * 或查到起点自动 stop → is_running 变 0,车停稳,返回完成。 */
            if(!kart_playback_is_running())
            {
                if(kart_playback_get_result() != KART_PLAYBACK_RESULT_COMPLETED)
                {
                    kart_debug_uart_set_event(KART_EVENT_S3_ABORT, KART_EVENT_LEVEL_ERROR);
                    subject3_stage = S3_FAULT;
                    break;
                }
                /* 已停在发车区。规则要求这里还要按语音口令做灯光/鸣笛,
                 * 所以不直接收车,先进 S3_SIGNAL。
                 * 进信号阶段要把语音串口抢过来(与 VOFA 日志共用 UART10),
                 * 顺序必须"先停日志、再切波特率"(理由见 mission_enter 科目二段)。
                 * 代价:这一段没有 VOFA 日志。车已经停稳,没有控制过程要看。 */
                mission_stop_all();         /* 车/转向彻底断输出,信号阶段只动灯和喇叭 */
#if BOARD_VOICE_SHARES_AUX_UART
                kart_debug_uart_set_enabled(0);
#endif
                kart_voice_uart_acquire();
                subject3_signal_ticks = 0;
                subject3_signal_done  = 0;
                subject3_stage = S3_SIGNAL;
            }
            break;

        case S3_SIGNAL:
            /* 第三阶段:已回发车区,按语音口令做灯光/鸣笛。
             * 【不用 kart_voice_dispatch()】它会把门洞类(0x15~0x1E)和动作类
             * (0x1F~0x26)派发成 playback / motion —— 那会让已经完赛停稳的车
             * 重新开动。这里只放行灯光(0x04~0x0B)和鸣笛(0x0C~0x14),
             * 其余口令一律丢弃。 */
            kart_voice_poll();
            {
                kart_voice_cmd_t cmd;

                /* 鸣笛是长动作:忙时不取新命令,保证串行(与 dispatch 同规则)。 */
                if(!kart_horn_is_busy() && kart_voice_get_cmd(&cmd))
                {
                    if(cmd.cmd >= KART_VOICE_CMD_LEFT_LIGHT
                       && cmd.cmd <= KART_VOICE_CMD_WIPER)
                    {
                        kart_light_set_command((kart_light_command_t)
                            (KART_LIGHT_CMD_LEFT_TURN
                             + (cmd.cmd - KART_VOICE_CMD_LEFT_LIGHT)));
                        subject3_signal_done = 1;
                    }
                    else if(cmd.cmd >= KART_VOICE_CMD_HORN_1S
                            && cmd.cmd <= KART_VOICE_CMD_HORN_ALARM)
                    {
                        kart_horn_start((uint8)(cmd.cmd - KART_VOICE_CMD_HORN_1S + 1));
                        subject3_signal_done = 1;
                    }
                    /* else:门洞/动作类在本阶段无意义,直接丢弃(车不能再动)。 */
                }
            }

            /* 超时收车:不算故障,免得灯一直亮/一直等着。
             * 正常流程是裁判问完就手动 LEFT 退出,这道闸只防没人管。 */
            if(subject3_signal_ticks < 0xFFFFU) { subject3_signal_ticks++; }
            if(subject3_signal_ticks >= KART_S3_SIGNAL_TIMEOUT_TICKS)
            {
                kart_debug_uart_set_event(KART_EVENT_S3_DONE, KART_EVENT_LEVEL_INFO);
                kart_mission_set_mode(MISSION_REMOTE);
                return;
            }
            break;

        case S3_FINISHED:
            /* 全流程结束:保持无输出。停机已在进 S3_SIGNAL 时做过。 */
            break;

        case S3_FAULT:
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
    if((mode == MISSION_SUBJECT_1 || mode == MISSION_SUBJECT_2 || mode == MISSION_SUBJECT_3)
       && !kart_imu_is_ready())
    {
        mode = MISSION_FAULT;
    }

    if(mode == mission_mode)
    {
        return;                         /* 同模式不重复切 */
    }

    /* 离开科目三先关 4D7 RX，避免退出后的菜单继续承受无意义的串口中断。 */
#if (KART_S3_FOLLOW_SRC == KART_S3_FOLLOW_SRC_PLINK)
    if(MISSION_SUBJECT_3 == mission_mode)
    {
        kart_person_link_set_rx_enabled(0U);
    }
#endif

    /* 统一 exit:任何模式退出都执行完整停机(清运动+蜂鸣器+队列)。 */
    mission_stop_all();

    /* 退出科目二:把共用串口还给 VOFA 日志(切回 460800)再开闸。
     * 放在 mission_enter 之前,避免与新模式的 enter 抢同一外设。
     * 顺序与进入时相反:先切波特率、后开闸 —— 开闸后立刻可能发字节,
     * 此时外设必须已经是 460800,否则第一批帧以 115200 发出会是乱码。 */
    /* 科目三也可能持有语音串口:它的 S3_SIGNAL 阶段(回发车区后按口令做灯光/鸣笛)
     * 同样 acquire 过。判据用"阶段已到 S3_SIGNAL 及以后"而不是无条件释放 ——
     * 在跟随/倒车阶段就退出的话根本没 acquire,那时 release 会把日志波特率
     * 切成还没被改过的样子(实际是同一个 460800,行为无害但语义是错的),
     * 更要紧的是 debug_uart_set_enabled(1) 会把本来关着的日志闸打开。 */
    if((MISSION_SUBJECT_2 == mission_mode)
       || (MISSION_SUBJECT_3 == mission_mode && subject3_stage >= S3_SIGNAL))
    {
        kart_light_set_command(KART_LIGHT_CMD_OFF);   /* 灯板图案归零,屏幕交回模式号显示 */
        kart_voice_uart_release();
#if BOARD_VOICE_SHARES_AUX_UART
        kart_debug_uart_set_enabled(1);
#endif
    }

    /* 调度器最坏值按模式分段。漏拍数和最大分发耗时都是历史极值,不清的话
     * 跟随段的真实负载会被之前空闲段的峰值盖住,日志看不出是哪一段卡的。
     * 唯一读者是 VOFA 日志通道,清零不参与任何控制判据。 */
    g_sched_max_exec_us   = 0;
    g_sched_overrun_count = 0;

    mission_mode = mode;
    mission_enter(mode);
}

uint16 kart_mission_get_s3_vision_age(void)
{
    return s3_vision_age_ticks;
}

uint32 kart_mission_get_s3_vision_seq(void)
{
    return s3_vision_seq;
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
        case MISSION_SUBJECT_3:
            subject3_loop();
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

kart_subject3_stage_t kart_mission_get_subject3_stage(void)
{
    return subject3_stage;
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
