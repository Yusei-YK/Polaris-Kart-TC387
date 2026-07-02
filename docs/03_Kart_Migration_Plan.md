# 卡丁快跑科目一迁移计划

本文档给出后续落代码顺序。原则是先做能验证的底层，再做闭环，最后做比赛状态机。

## 1. 总目标

在 `G:\CODE\Smart car\SmartCar\6.6_V1_TC264` 上写出一套低速稳定完赛的卡丁科目一代码。

主线控制：

```text
编码器累计路程 + IMU yaw + 前轮角度闭环 + 左右后轮速度闭环 + Mission 状态机
```

短期成功标准：

- 上电自检能打印关键传感器。
- 三路有刷电机能单独安全控制。
- 左右后轮编码器方向正确。
- SPI3 前轮绝对值编码器能读到 `0~4095`。
- IMU yaw 可归零，短时间稳定。
- 能记录短路径。
- 能悬空回放路径。
- 能低速地面直行。
- 能低速完成科目一基本流程。

## 2. 工程边界

### 只写应用层

后续新文件建议放在：

```text
G:\CODE\Smart car\SmartCar\6.6_V1_TC264\code
```

因为当前工程说明要求用户代码直接放在 `code` 下，不再创建子文件夹。

### 不动框架

不修改：

- `libraries`
- 逐飞底层库
- 英飞凌 iLLD
- `.cproject` / `.project`，除非 ADS 需要刷新工程纳入新文件
- TopSpeed 参考工程逻辑

允许的最小改动：

- `user/cpu0_main.c`：加入 `kart_app_init()` 和 `kart_app_loop()`。
- `user/isr.c`：在合适 PIT ISR 中调用 `kart_app_1ms_isr()` 或 `kart_app_5ms_isr()`。
- 这些入口改动必须很小，具体控制逻辑放在 `code/kart_*.c`。

## 3. 建议文件列表

由于 `code` 目录要求平铺，建议用文件名前缀表达模块边界：

```text
kart_board_pins_v1.h
kart_types.h
kart_pid.c
kart_pid.h
kart_motor.c
kart_motor.h
kart_encoder.c
kart_encoder.h
kart_abs_encoder.c
kart_abs_encoder.h
kart_imu.c
kart_imu.h
kart_chassis.c
kart_chassis.h
kart_path.c
kart_path.h
kart_mission.c
kart_mission.h
kart_subject1.c
kart_subject1.h
kart_debug.c
kart_debug.h
kart_app.c
kart_app.h
```

第一批不要一次写完全部。先写最小自检链路。

## 4. 里程碑

### M0 文档和边界确认

输出：

- `docs/01_TopSpeed_Project_Map.md`
- `docs/02_Technical_Details_For_Kart.md`
- `docs/03_Kart_Migration_Plan.md`
- `docs/04_Indoor_Test_Checklist.md`
- `docs/05_Second_PCB_Revision_Checklist.md`

通过标准：

- 文档明确 TopSpeed 哪些可复用、哪些必须重写。
- 文档明确 SmartCar 目标落点和不改框架边界。

### M1 板级引脚别名和上电自检

新增：

- `kart_board_pins_v1.h`
- `kart_app.c/h`
- `kart_debug.c/h`

任务：

- 用清晰语义别名映射 `board_pins.h`。
- 打印或显示板级版本、编译时间、传感器初始状态。
- 先不输出电机 PWM。

通过标准：

- 上电后串口能看到自检信息。
- 所有电机默认停止。

### M2 三路电机底层

新增：

- `kart_motor.c/h`

接口：

```c
void kart_motor_init(void);
void kart_motor_set_steer(int16_t pwm);
void kart_motor_set_left(int16_t pwm);
void kart_motor_set_right(int16_t pwm);
void kart_motor_stop_all(void);
```

原则：

- signed PWM：正负号代表方向。
- 方向反转只改 sign 宏。
- 上层不得直接调用 `pwm_set_duty()` 或 `gpio_set_level()` 控电机。

通过标准：

- 悬空状态下三路电机能分别 5% 正反转。
- 急停或 disable 后三路全部停止。

### M3 左右后轮编码器

新增：

- `kart_encoder.c/h`

任务：

- 初始化左后 TIM2。
- 尝试初始化右后 TIM5。
- 读取 delta。
- 清零计数。
- 做方向 sign 宏。

风险：

当前 `encoder_quad_init()` 对 TIM5 有 `zf_assert(encoder_n <= TIM4_ENCODER)`，后续要重点编译和实测。如果断言阻止 TIM5，不能改库硬闯，要先给出硬件或软件替代方案。

通过标准：

- 手推左后轮，左编码器 delta 正负合理。
- 手推右后轮，右编码器 delta 正负合理。
- 车向前推时左右累计距离同号。

### M4 前轮绝对值编码器

新增：

- `kart_abs_encoder.c/h`

任务：

- 使用 SPI3。
- 手动控制 `P23_1` CS。
- 读 12 bit 原始值 `0~4095`。
- 提供角度换算接口。
- 暂时只做读数，不急着做中心标定。

通过标准：

- 转动前轮，raw 值连续变化。
- 不转动时 raw 值抖动小。
- 串口输出 raw 和 deg。

### M5 IMU yaw

新增：

- `kart_imu.c/h`

任务：

- 封装 IMU 初始化。
- 支持 yaw reset。
- 输出 `-180~180` yaw。
- 不依赖磁力计。

通过标准：

- 静止 30 秒 yaw 漂移可接受。
- 原地转动车身，yaw 方向正确。
- reset 后当前方向接近 0 度。

### M6 底盘闭环

新增：

- `kart_pid.c/h`
- `kart_chassis.c/h`

闭环拆分：

- 前轮角度闭环：目标前轮角 -> 转向电机 PWM。
- 左右轮速度闭环：目标速度 -> 左右轮 PWM。
- yaw 外环：目标 yaw -> 目标前轮角。

通过标准：

- 悬空速度闭环能稳定到低速目标。
- 转向角闭环能回到中心。
- 地面低速直行不会明显跑偏。

### M7 路径记录和回放

新增：

- `kart_path.c/h`

路径点：

```c
typedef struct {
    float distance_m;
    float yaw_deg;
    float speed_mps;
    uint8_t event;
} kart_path_point_t;
```

通过标准：

- 推车或低速开车能记录距离-yaw。
- 悬空回放时能看到 yaw_ref、speed_ref 和输出变化。

### M8 科目一状态机

新增：

- `kart_mission.c/h`
- `kart_subject1.c/h`

状态：

```text
KART_IDLE
KART_IMU_CALIB
KART_STEER_CENTER
KART_WAIT_START
KART_FORWARD_REPLAY
KART_SLOW_FOR_PARK
KART_REVERSE_ALIGN
KART_REVERSE_PARK
KART_STOP
KART_FAULT
```

通过标准：

- 室内低速模拟锥桶能按事件切换状态。
- 任一故障进入 `KART_FAULT` 并停机。

## 5. 导航模式保留接口

按交接文档保留三个模式名，但第一阶段只实现惯导回放：

```text
NAV_INERTIAL_REPLAY
NAV_GPS_BACKUP
NAV_FUSION_LIGHT
```

第一版策略：

- `NAV_INERTIAL_REPLAY`：主用。
- `NAV_GPS_BACKUP`：先留接口，不依赖它完赛。
- `NAV_FUSION_LIGHT`：后续再做。

## 6. 风险清单

### TIM5 正交编码器

风险：库头文件有 TIM5 枚举，源文件也有 TIM5 初始化分支，但 `encoder_quad_init()` 开头断言只允许到 TIM4。

处理：先编译验证，再决定是否换接 TIM3/TIM4、使用 `encoder_dir_init()`，或写不改库的封装方案。

### SPI3 绝对值编码器

风险：逐飞默认绝对值编码器驱动硬编码 SPI0，不适配第一版板。

处理：应用层新写 `kart_abs_encoder`，不要改 `zf_device_absolute_encoder` 默认宏。

### 三路有刷电机方向

风险：方向接反会导致闭环正反馈，车会越控越偏。

处理：所有方向只通过底层 sign 宏修正，上层永远使用“正值代表车辆定义的正方向”。

### 缺少急停/驱动使能/故障反馈

风险：调试三路电机时危险。

处理：第一版如果硬件还没接急停，也要在软件里默认 disable，必须通过测试模式显式 enable。

### 一次改太多

风险：编译错误和硬件问题混在一起，无法定位。

处理：每个 M 里程碑独立验证，不把闭环和比赛策略一起写。

## 7. 下一步推荐

下一步只做 M1：

- `kart_board_pins_v1.h`
- `kart_app.c/h`
- `kart_debug.c/h`
- 最小改 `cpu0_main.c`

不要在 M1 输出任何电机 PWM。先确认工程能编译、上电能打印、所有执行器保持停止。
