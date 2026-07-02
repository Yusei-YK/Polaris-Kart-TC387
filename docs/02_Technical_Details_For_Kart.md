# 面向卡丁迁移的 TopSpeed 技术细节

本文档是第二遍阅读后的技术笔记。它解释 TopSpeed 的关键模块如何工作，以及卡丁快跑科目一应该怎样借鉴。

## 1. PID 控制器

PID 是用“当前值和目标值的差”来算输出的控制器。

最简单的例子：

```text
目标速度 = 0.5 m/s
实际速度 = 0.3 m/s
误差 = 0.2 m/s
PID 输出 = 增大电机 PWM
```

TopSpeed 的 PID 结构体主要包含：

- `Kp`：比例系数。误差越大，输出越大。
- `Ki`：积分系数。误差长期存在时，慢慢增加输出。
- `Kd`：微分系数。误差变化太快时，提前抑制。
- `err`：当前误差。
- `err_sum`：误差累加，用于积分项。
- `err_last`：上一次误差，用于微分项。
- `output`：PID 最终输出。
- `out_max`：输出限幅。
- `i_max`：积分限幅，防止积分越积越大。

TopSpeed 的调用方式是：

```text
init_PID_ctrler 初始化参数
PID_ctrler_update 输入误差
读取 pid.output
必要时 PID_ctrler_reset 清零历史误差
```

卡丁可以复用这个思想，但建议重写一个更干净的 `kart_pid`：

- 明确支持 `Ki = 0`，避免积分限幅逻辑异常。
- 积分项要有上限。
- 输出饱和时可考虑停止继续积分，防止“松开后过冲”。
- 每个闭环单独一个 PID，不要几个功能共用同一个 PID 变量。

卡丁需要的 PID 至少有：

- 左后轮速度 PID。
- 右后轮速度 PID。
- 前轮角度 PID。
- yaw 航向 PID。

## 2. GPS 几何函数

TopSpeed 的 GPS 模块不只是读经纬度，它把经纬度转换成以起点为原点的平面坐标。

为什么要这样做：

- 经纬度单位不是米，直接算距离和角度不直观。
- 比赛场地尺度很小，可以近似当成平面。
- 转成 `x/y` 米坐标后，就能用普通几何公式。

重要函数思想：

### `get_distance`

计算两点距离。

```text
dx = x2 - x1
dy = y2 - y1
distance = sqrt(dx*dx + dy*dy)
```

卡丁路径记录也要用距离，但第一阶段距离主要来自左右轮编码器平均值，而不是 GPS。

### `get_angle`

计算从点 A 指向点 B 的方位角。

```text
angle = atan2(y2 - y1, x2 - x1)
```

输出再转成角度制。

### `get_relative_angle`

计算两个角之间的最短差值，并归一化到 `-180~180`。

例如：

```text
当前 yaw = 179 度
目标 yaw = -179 度
直接相减 = -358 度
真实最短误差 = 2 度
```

卡丁 yaw 闭环必须使用这种“角度环绕”处理，否则在 180/-180 附近会突然大转向。

### `draw_straight_line`

根据两个点生成一条直线。TopSpeed 常用直线来表示路径、判定线、刹车线。

### `point_to_straight_line_distance`

计算当前点到某条直线的有符号距离。有符号的意思是：车在直线左侧还是右侧可以用正负号区分。

TopSpeed 直线循迹的典型思路是：

```text
角度误差 = 车头 yaw 和路径方向的差
横向误差 = 车到路径直线的距离
舵机输出 = 角度 PID + 横向距离 PID
```

卡丁第一阶段的主线不是 GPS 直线循迹，但 GPS 备份模式可以复用这套几何函数。

## 3. IMU yaw 处理

IMU yaw 是车辆绕竖直方向转过的角度，也就是车头朝向。

TopSpeed 技术报告里提到：yaw 没有可靠的低频校正源。地磁容易受电机和环境干扰，所以 TopSpeed 主要依赖陀螺仪积分，在一个科目 30 秒左右的时间内漂移可以接受。

卡丁科目一第一阶段也应这样处理：

- 上电后静止校准陀螺仪零偏。
- 发车前调用 yaw reset，把当前车头方向记为 0 度。
- 控制中始终把 yaw 误差归一化到 `-180~180`。
- 不依赖磁力计。
- 每次测试只跑短时间，先验证闭环，不追求长时间无漂移。

卡丁应封装成：

```text
kart_imu_init
kart_imu_update_5ms
kart_imu_reset_yaw
kart_imu_get_yaw_deg
```

`zxs_imu963RA.c` 里已有 yaw 积分和姿态解算代码，可读作参考，但它含有阻塞式校准和全局变量，后续需要整理成卡丁自己的 IMU wrapper。

## 4. Mission 状态机

TopSpeed 不把完整比赛流程写在 `while(TRUE)` 里，而是用 Mission 调度。

核心概念：

- `Subject`：当前运行哪个科目。
- `Mission_CALLBACK()`：周期调用，根据 `Subject` 分发到对应科目 loop。
- `Mission_main()`：处理延迟事件。
- 每个 Subject 内部再有自己的状态机。

状态机适合比赛流程，因为比赛流程天然是分阶段的：

```text
等待开始
校准
发车
绕桩
接近倒库
倒车入库
停车
故障
```

卡丁建议使用：

```c
typedef enum {
    KART_IDLE,
    KART_IMU_CALIB,
    KART_STEER_CENTER,
    KART_WAIT_START,
    KART_FORWARD_REPLAY,
    KART_SLOW_FOR_PARK,
    KART_REVERSE_ALIGN,
    KART_REVERSE_PARK,
    KART_STOP,
    KART_FAULT
} kart_state_t;
```

初学者要理解：状态机不是为了复杂，而是为了避免“代码跑到哪一步不知道”。每个状态只做一件事，并且有明确的切换条件。

## 5. Power 输出安全策略

TopSpeed 的一个好设计是：任务代码不直接写 PWM。

任务层只写：

```text
Power_now.Motor_Duty
Power_now.Servo_Duty
```

最后由 `power_sync()` 做：

- 限幅。
- 遥控接管。
- 滤波。
- 安全停机。
- 真正输出到底层硬件。

卡丁必须保留这个思想，但要重写成三路有刷电机：

```c
typedef struct {
    float speed_target_mps;
    float steer_target_deg;
    int16_t left_pwm;
    int16_t right_pwm;
    int16_t steer_pwm;
    uint8_t enable;
} kart_power_t;
```

最后由 `kart_power_sync()` 输出：

- 转向电机 DIR+PWM。
- 左后轮 DIR+PWM。
- 右后轮 DIR+PWM。
- 急停和驱动使能检查。
- nFAULT 检查。
- 电流/电压保护。
- 输出限幅。

注意：TopSpeed 的 `send_motor_duty()` 是无刷驱动板串口协议，卡丁不能用。

## 6. Flash 参数保存

TopSpeed 的 Flash 思路是：把一整套参数做成结构体，然后整页读写。

优点：

- 参数集中。
- 保存和读取简单。
- 菜单调参后可以直接写入 Flash。

风险：

- 结构体大小要注意 4 字节对齐。
- 写 Flash 前要擦除。
- 如果结构体版本变化，旧 Flash 数据可能不兼容。

卡丁可以先不急着写 Flash，第一阶段参数可以写成宏或全局变量。等电机、编码器、IMU、闭环跑通后，再做：

- `kart_param_t`：PID 参数、限幅、方向符号。
- `kart_path_point_t`：路径点表。
- `kart_flash_param.c`：保存和读取。

路径点建议：

```c
typedef struct {
    float distance_m;
    float yaw_deg;
    float speed_mps;
    uint8_t event;
} kart_path_point_t;
```

## 7. 日志系统

TopSpeed 技术报告强调日志非常重要。高速室外车不是肉眼能看清每个控制周期发生了什么，必须把数据记下来。

日志应记录：

- 当前状态机状态。
- yaw 当前值和目标值。
- 前轮角当前值和目标值。
- 左右轮速度当前值和目标值。
- 三路 PWM。
- 编码器累计距离。
- 急停、遥控、故障标志。

卡丁第一阶段可以用串口打印，格式先简单：

```text
t,state,dist,yaw,yaw_ref,steer,steer_ref,vl,vr,pwm_l,pwm_r,pwm_s,fault
```

后续再做 FIFO 和高速日志，避免打印阻塞控制周期。

## 8. TopSpeed 科目二策略 13 的技术核心

策略 13 是 TopSpeed 后期绕桩策略之一。它对卡丁不适合直接搬，但有几个重要思想。

### 8.1 拟合并投影锥桶

作者先对锥桶点做总最小二乘拟合直线，再把锥桶投影到这条线上。这样能减少人工打点误差，让锥桶排列更“整齐”。

卡丁如果后续做 GPS 备份绕桩，可以借鉴这个思路。

### 8.2 预瞄点

策略 13 不直接追锥桶本身，而是追两个锥桶之间偏移出来的预瞄点。这样车不会朝障碍物撞过去，而是朝安全路径走。

卡丁路径复刻也可以理解为“按距离查目标 yaw 和目标速度”，目标点不是物理锥桶，而是路径表中的参考状态。

### 8.3 判定线切换

TopSpeed 使用判定线判断何时切换到下一个锥桶。好处是切换条件与空间位置绑定，而不是只靠时间。

卡丁科目一也应尽量用：

- 编码器累计距离。
- yaw 达标时间。
- 前轮角达标时间。
- 事件点。

少用“delay 几秒后进入下一状态”。

### 8.4 父状态机加子状态机

策略 13 外层状态包括：

```text
Fast_Across
Crossing_Barrier
Turning
Rush
Finished
```

绕桩内部再用当前锥桶编号和子状态推动流程。

卡丁也应这样分层：外层管科目流程，内层管路径回放、倒库对正、转向闭环等。

## 9. 当前 SmartCar 硬件适配细节

### 三路电机

交接文档要求第一版：

```text
转向: P21_2 DIR, ATOM0_CH1_P21_3 PWM
左后: P02_4 DIR, ATOM0_CH5_P02_5 PWM
右后: P02_6 DIR, ATOM0_CH7_P02_7 PWM
```

`board_pins.h` 里已有这些 GPIO/PWM 枚举，但命名还是 `BOARD_MOTOR1_PWMx`、`BOARD_MOTOR2_PWMx`，语义不清。建议新建 `kart_board_pins_v1.h` 做清晰别名，不直接改 `board_pins.h`。

### 编码器

左后轮：

```text
TIM2_ENCODER
TIM2_ENCODER_CH1_P33_7
TIM2_ENCODER_CH2_P33_6
```

右后轮：

```text
TIM5_ENCODER
TIM5_ENCODER_CH1_P10_3
TIM5_ENCODER_CH2_P10_1
```

风险：当前 `zf_driver_encoder.c` 里 `encoder_quad_init()` 有 `zf_assert(encoder_n <= TIM4_ENCODER)`，但后面又写了 TIM5 的初始化分支。也就是说 TIM5 枚举和底层代码存在矛盾。后续不能盲目认为右轮正交一定可用，需要实车或编译验证。如果断言生效，要考虑：

- 改为 `encoder_dir_init()`，但这会降低正交能力。
- 换硬件到 TIM3/TIM4 支持脚。
- 在不改库的前提下做应用层兼容方案。

### 前轮绝对值编码器

第一版硬件使用：

```text
SPI_3
SPI3_SCLK_P22_3
SPI3_MOSI_P22_0
SPI3_MISO_P22_1
SPI3_CS13_P23_1 / GPIO P23_1
```

逐飞默认 `zf_device_absolute_encoder` 不是这个引脚，应新写 `kart_abs_encoder`，使用 SPI3 和手动 CS 读 12 bit 原始值。

## 10. 卡丁第一阶段控制主线

最终目标主线：

```text
编码器累计距离 distance_now
IMU yaw_now
路径表查 yaw_ref 和 speed_ref
yaw_ref - yaw_now -> yaw PID -> steer_angle_target
steer_angle_target - steer_angle_now -> steer PID -> steer_pwm
speed_ref - wheel_speed -> 左右轮 PID -> left/right pwm
```

第一阶段不要同时打开所有闭环。推荐顺序：

1. 三路电机开环 5% PWM 测试。
2. 编码器方向测试。
3. 前轮绝对角读数测试。
4. IMU yaw reset 测试。
5. 前轮角度闭环。
6. 左右后轮速度闭环。
7. yaw 到前轮角目标的外环。
8. 路径记录和回放。
9. 科目一状态机。
