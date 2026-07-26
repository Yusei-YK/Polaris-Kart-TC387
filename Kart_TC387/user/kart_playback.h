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

#endif
