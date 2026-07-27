#ifndef KART_PLAYBACK_H_
#define KART_PLAYBACK_H_

#include "zf_common_headfile.h"

/*
 * 路径复现模块（Pure Pursuit 纯跟踪）
 * ------------------------------------------------------------------
 * 读 kart_record 录下的路径点（相对起点坐标），用 Pure Pursuit 算法
 * 跟踪：从当前位置沿路径前视一段距离找目标点 → 目标点方位角作为航向
 * 外环目标 → 串级转向自动打角；速度取录制点速度喂速度环。
 * ------------------------------------------------------------------
 * 坐标系：录制与播放都以各自起点车体坐标系表示，x向右、y向前。
 * playback_start 保存本次起点位置和航向，并把当前世界坐标旋转到该局部坐标系。
 * ------------------------------------------------------------------
 * 调用位置：
 *   kart_playback_start()  —— VOFA 命令 b1(先 z 清零、摆正车)
 *   kart_playback_poll()   —— 主循环(在 steer_abs_update 之后、
 *                             steer_ctrl_update 之前，好覆写目标航向)
 *   kart_playback_stop()   —— b0 或跑完自动停
 */

/* ===== 前视距离(自适应)=====
 * 手动控速下速度全程变化,固定前视在任何单值都不对(低速切内、高速偏移)。
 * 故按速度线性自适应:LD = clamp(BASE + GAIN*|v|, MIN, MAX)。
 * v 单位脉冲/5ms,LD 单位 m。GAIN 让高速时前视自动拉长,减小偏移;
 * 低速回落到 BASE,弯道不切内。上车按走线微调这三个值。 */
#define KART_PLAYBACK_LD_BASE       (0.35f)     /* 前视基础(低速下限量级) */
#define KART_PLAYBACK_LD_GAIN       (0.020f)    /* 每单位速度增加的前视(m per 脉冲/5ms) */
#define KART_PLAYBACK_LD_MIN        (0.30f)     /* 前视下限:防低速点太近抖动 */
#define KART_PLAYBACK_LD_MAX        (1.20f)     /* 前视上限:防高速瞄太远切内过弯 */
#define KART_PLAYBACK_FINISH_DIST   (0.15f)     /* 到终点判定距离(m):提前15cm停车避免过冲 */

/* ===== 速度剖面(低速录制 → 提速复现)=====
 * 【为什么需要】原先复现速度 = 录制速度(逐点回放),于是"录多快就复多快"。
 *   但高速录制轮胎打滑、路径录歪;低速录制路径好,速度剖面却被压平(全程慢),
 *   直接乘倍率则直道弯道同比放大 → 弯道冲出去。
 * 【做法】速度不再取自录制,而是在 playback_start 时按【路径几何】现场算:
 *   ① 曲率限速:v_curve = sqrt(a_lat / |kappa|),kappa 由 wp[].yaw 沿弧长中心差分得到;
 *   ② 反向传播:v[i] = min(v[i], sqrt(v[i+1]² + 2*a_brake*ds)) —— 把弯心低速往前传,
 *      实现"入弯前提前减速"。这是倍率/斜坡都做不到的(刹车必须前瞻);
 *   ③ 直道由 PB Vmax 钳位,PB Scale 再整体缩放,PB Vmin 兜底防爬行。
 *   出弯加速不在此处限:已由 kart_control 的目标速度斜坡(ramp_step)在时间域管住。
 * 录制速度仅用于判断"该点是前进段还是倒车段",不再决定快慢。
 * 倒车段不进剖面(开环回放打角,提速只会放大里程漂移),仍按录制速度走。 */
#define KART_PLAYBACK_ALAT_DEFAULT  (4.0f)      /* 弯道横向加速度上限(m/s²),越小过弯越慢越稳 */
#define KART_PLAYBACK_ABRAKE        (2.5f)      /* 反向传播减速度(m/s²),越小刹车提前量越大 */
#define KART_PLAYBACK_KAPPA_WIN     (3)         /* 曲率中心差分半窗(点):太小会被 2°量化噪声打乱 */
/* 脉冲/5ms ↔ m/s 换算:0.00036816 m/脉冲 ÷ 0.005 s = 0.0736 m/s per (脉冲/5ms)。
 * 剖面内部用 m/s 算(a_lat/a_brake 才有物理意义),存表前换回脉冲/5ms。 */
#define KART_PLAYBACK_V_TO_MS       (0.0736322f)
#define KART_PLAYBACK_MS_TO_V       (13.5811f)
#define KART_PLAYBACK_SPEED_MAX     (60.0f)     /* 复现速度上限钳位(脉冲/5ms):跟随录制速度,防毛刺冲出。须与 kart_remote.h 的 KART_REMOTE_MAX_SPEED 同步,否则录得快复现被钳慢 */

/* ===== 方案B:正向复现中的倒车段分段处理阈值 =====
 * 录制轨迹含倒车段(v<0)时,Pure Pursuit 航向环对车尾是正反馈会打圈发散。
 * 故进度点速度为负时:关航向外环,只开转角内环,开环回放该点录制打角 steer_buf[nearest]。
 * 进度点速度为正时:维持原 Pure Pursuit。带死区防前进↔倒车边界使能位反复翻转。 */
#define KART_PLAYBACK_REV_SPEED_EPS (2.0f)      /* 倒车段判定死区(脉冲/5ms):|v|<此值维持上一拍模式 */

/* ===== 倒车段航向闭环纠偏 =====
 * 纯开环回放录制打角不保持航向(地面阻力/左右轮不对称累积漂移,实测倒车漂 11°)。
 * 故倒车段在"录制打角"基准上叠加一个航向 P 纠偏:参考 = 录制点航向 wp[nearest].yaw,
 * 实测 = IMU yaw,误差经 P 增益转成打角修正量。倒车阿克曼动力学与前进相反,
 * 故纠偏符号取负(SIGN=-1);若上车发现越纠越斜(正反馈发散),改成 +1。 */
#define KART_PLAYBACK_REV_HEAD_KP   (20.0f)     /* 倒车航向纠偏 P 增益(编码器计数/度) */
#define KART_PLAYBACK_REV_HEAD_SIGN (-1.0f)     /* 倒车纠偏符号:-1=反向动力学负反馈;越纠越斜则改+1 */
#define KART_PLAYBACK_REV_CORR_MAX  (400.0f)    /* 纠偏量钳位(计数):防坏参考点猛打方向 */

/* ===== 科目四开环反向复现参数 ===== */
#define KART_PLAYBACK_OL_SPEED      (-12.0f)    /* 开环倒车固定速度(脉冲/5ms,负=倒退),先测小求稳 */
#define KART_PLAYBACK_OL_FINISH     (0.10f)     /* 剩余里程<此值(m)判返回发车区,停车 */

/* ===== 科目四开环倒车的航向 P 纠偏(2026-07-27 新增)=====
 * 现象:纯开环回放打角倒车,前两个桩正常,第三个桩起开始飘。
 * 原因:开环没有任何航向反馈,单拍的小误差(内环死区约 63 计数、左右轮不对称、
 *       地面侧滑、里程标量 dist_sum 把打滑也算成前进)逐点累积,越走越偏。
 * 做法:与方案B倒车段同构 —— 在"录制打角 steer[k]"基准上叠加航向 P 纠偏:
 *       参考航向 = 录制起点航向 + wp[k].yaw(该点录制时的航向,转回世界系)
 *       误差     = wrap180(参考 - IMU 实测 yaw)
 *       修正量   = SIGN * KP * 误差,钳位后加到打角上。
 * 注意:参考航向与打角共用同一个索引 k。若真正的病根是里程/索引漂移,
 *       参考航向也会跟着错,纠偏会持续往一边使劲 —— 那就把 EN 关掉再跑一趟,
 *       对比 CH13(参考航向) 与 CH9(实测 yaw) 就能分清是索引漂还是航向漂。
 * 调参:先只开 EN 跑,越纠越斜(正反馈发散)就把 SIGN 改 +1;
 *       纠得太慢降不下误差就加大 KP;打角来回抖就加大 DB 或减小 KP。 */
#define KART_PLAYBACK_OL_HEAD_EN    (1)         /* 1=开航向P纠偏 0=退回纯开环(对比用) */
#define KART_PLAYBACK_OL_HEAD_KP    (20.0f)     /* 纠偏 P 增益(编码器计数/度),与方案B同量级 */
#define KART_PLAYBACK_OL_HEAD_SIGN  (-1.0f)     /* 纠偏符号:-1=倒车反向动力学负反馈;越纠越斜改 +1 */
#define KART_PLAYBACK_OL_CORR_MAX   (400.0f)    /* 纠偏量钳位(计数):防坏参考点猛打方向 */
#define KART_PLAYBACK_OL_HEAD_DB    (1.5f)      /* 误差死区(度):内环有约63计数死区,小误差别抖 */

void   kart_playback_init(void);
uint8  kart_playback_start(void);       /* 1=成功启动，0=路径无效 */
void   kart_playback_stop(void);
void   kart_playback_poll(void);
uint8  kart_playback_is_running(void);
uint16 kart_playback_get_index(void);
float  kart_playback_get_target_yaw(void);

/* 科目四开环反向复现接口:车头不掉转,直接挂倒挡按里程回放录制打角原路倒回。
 * 不走 Pure Pursuit/航向外环/IMU 重建,只开转角内环 + 速度环负速。1=成功,0=路径无效。 */
uint8  kart_playback_start_openloop_reverse(void);

/* 诊断读出（VOFA 日志用，不改控制）：本次播放起点车体系下的
 * 当前投影位置 cur 与当前瞄准点 aim。整段跑完后 aim 序列即录制
 * 路径，cur 轨迹叠上去可区分漂移(H1/H2)还是绕点公转(H3)。 */
float  kart_playback_get_cur_x(void);
float  kart_playback_get_cur_y(void);
float  kart_playback_get_aim_x(void);
float  kart_playback_get_aim_y(void);

/* 诊断:读某点的剖面速度(脉冲/5ms)。菜单/VOFA 观测减速点落位用。
 * 剖面未生成(未启动/正在开环倒车)或下标越界返回 0。 */
float  kart_playback_get_profile_v(uint16 i);
/* 剖面是否已生成(1=本次复现用剖面速度,0=退回录制速度)。 */
uint8  kart_playback_profile_valid(void);

#endif
