#ifndef KART_MOTION_H_
#define KART_MOTION_H_

#include "zf_common_headfile.h"

/*
 * 科目二语音运动控制 —— 执行层状态机
 * ------------------------------------------------------------------
 * 由 kart_voice_dispatch() 按语音命令码 0x1F~0x26 路由到 kart_motion_start()。
 * 只用【编码器累计路程 kart_odom_get_dist()】+【IMU 航向 kart_imu_get_yaw()】,
 * 不用 x/y 位置积分(用户要求:惯导已标定,直接用路程+航向,不做二维定位)。
 *
 * 实际 PWM 仍由主循环的 kart_steer_ctrl_update()/速度环产生,本模块只设
 * 目标转角/航向/速度和使能(与 playback/remote 同架构,5ms 延迟无感)。
 *
 * 转圈/转弯完成判据用【累计 yaw】(每拍 wrap180 增量累加),不受 ±180 回绕影响;
 * 直行/蛇形用【路程】判完成。
 *
 * 急停:kart_motion_update() 顶部判 deadman(遥控失联 或 三段低挡)→ 立即停,
 *      与 kart_playback 一致(遥控当安全绳)。
 * 串行:一条动作跑完(is_busy=0)前,dispatch 不取新命令。
 * ------------------------------------------------------------------
 * 所有参数先按"小测"给,现场按规则放大(全宏可调)。
 */

/* 统一速度(脉冲/5ms),先小测 7 */
#define KART_MOTION_SPEED           (7.0f)

/* 直线路程阈值(米):现场规则只需超过规定值(如≥5m),前行先按 3m 试 */
#define KART_MOTION_FWD_DIST        (3.0f)
#define KART_MOTION_BACK_DIST       (3.0f)

/* 蛇形:总行进路程 / 每半摆翻转一次的路程 / 打角幅度(编码器计数)。
 * 轨迹左右摆动幅度靠 DELTA 与 HALF_DIST 决定,规则要求摆动>2m,先小测再放大。 */
#define KART_MOTION_SNAKE_DIST      (3.0f)
#define KART_MOTION_SNAKE_HALF_DIST (0.8f)
#define KART_MOTION_SNAKE_DELTA     (300.0f)

/* 转圈:固定打角(计数),半径 3~10m 由打角大小决定(角越大半径越小),先小测。
 * 完成判据 = 累计 yaw 达到一整圈 360°。 */
#define KART_MOTION_CIRCLE_DELTA    (300.0f)
#define KART_MOTION_CIRCLE_ANGLE    (360.0f)

/* 左右转:先直行 approach(规则要求>2m)再原地转向,累计 yaw 到目标角回正停。
 * 规则未给特定角度,尽量大,取 90°(方向转正即停)。 */
#define KART_MOTION_TURN_APPROACH   (2.5f)
#define KART_MOTION_TURN_DELTA      (400.0f)
#define KART_MOTION_TURN_ANGLE      (90.0f)

/* --- 对外接口 --- */

/* 上电 init(与 voice/horn 同批做一次):复位状态机,不碰电机。 */
void  kart_motion_init(void);

/* 科目二 loop 每拍调:推进状态机 + deadman 急停。idle 时空操作。 */
void  kart_motion_update(void);

/* 由 voice dispatch 调:按语音命令码(0x1F~0x26)启动对应动作。
 * 正在跑动作(busy)时返回 0 不打断;成功启动返回 1;非运动命令返回 0。 */
uint8 kart_motion_start(uint8 voice_cmd);

/* 是否正在执行动作(dispatch 判串行用)。 */
uint8 kart_motion_is_busy(void);

/* 立即停止当前动作:速度清零关速度环、转向回中关内外环。 */
void  kart_motion_stop(void);

#endif
