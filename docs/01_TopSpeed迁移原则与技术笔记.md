# TopSpeed 迁移原则与技术笔记

> 历史参考，不是当前任务入口。当前状态以 `docs/会话交接主文档.md` 和当前代码为准。

本文档记录当前有效的 TopSpeed 迁移原则和技术笔记。旧文档里指向 `6.6_V1_TC264/`、`code/` 的落点已作废，当前落点统一为 `user/` 和 `code/`。

## 当前边界

- TopSpeed 参考工程：`G:\CODE\top speed\NEUQ_TopSpeed_CrossCountry_TC377-main`
- SmartCar 主力工程：`G:\CODE\Smart car\SmartCar`
- 当前用户代码：`user/` 和 `code/`
- 编译依赖：`TC387_Library-master/` 和工程内逐飞库，不能删除，不能 ignore。
- 不改写 TopSpeed 原工程，不改写历史提交。

迁移原则：

1. TopSpeed 是参考源，不是可整包复制的目标。
2. 保留可验证的工程思想：周期控制、统一动力输出、PID、Mission/Subject 状态机、日志。
3. 硬件相关全部按卡丁重写：三路有刷电机、左右后轮编码器、前轮绝对编码器、TC387 引脚。
4. 每次只打开一个闭环或一个硬件链路，先能停、能看、能解释，再提高复杂度。

## TopSpeed 可复用思想

### 5ms 周期控制

TopSpeed 的主循环不是主要控制节拍，真正的周期控制放在中断里。卡丁当前已经沿用这个思路：

- `user/cpu0_main.c` 初始化后打开 `CCU60_CH0` 5ms 周期中断。
- `user/isr.c` 的 `cc60_pit_ch0_isr()` 调用 `kart_imu_update()` 和 `kart_control_speed_update()`。
- 主循环负责低频轮询：转向绝对编码器刷新、后轮控制权仲裁、`power_sync()`、VOFA 调试输出。

### 统一动力输出

TopSpeed 的有效设计是任务层不直接写硬件输出，而是写统一的 Power 模型，再由 `power_sync()` 做限幅、安全和实际输出。

卡丁当前对应实现：

- `kart_power.c/.h`：三路有刷电机 DIR+PWM 输出、限幅、停止、遥控接管。
- `kart_remote.h` 只有声明，没有独立 `kart_remote.c`；遥控实现写在 `kart_power.c` 内，这是当前工程事实，不是缺文件。

### PID 算子

可复用的是 PID 算法思想，不是 TopSpeed 的整套控制对象。当前卡丁已新增 `kart_pid.c/.h`，后续闭环应保持每个环独立 PID：

- 后轮平均速度环：已实现。
- 左右轮独立速度环：待做。
- 前轮角度环：待做。
- yaw 航向外环：待做。

### Mission / Subject 状态机

TopSpeed 的 Mission/Subject 分层适合比赛流程，但卡丁科目一策略必须重写。推荐状态仍是：

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

原则：状态切换尽量依赖距离、yaw、角度、事件和故障标志，少用固定 `delay`。

### 日志和在线调参

TopSpeed 的日志思想必须保留。当前卡丁实现为 `kart_debug_uart.c/.h`：

- VOFA+ JustFloat。
- 当前 7 通道：目标速度、实测速度、输出 duty、yaw、RX 字节计数、转向 raw、转向中位偏差。
- ASCII 在线命令：`p/i/d/t/e` 调 Kp/Ki/Kd/目标速度/使能。

## 必须重写的部分

- 动力输出：TopSpeed 是舵机 + 无刷驱动板协议；卡丁是三路有刷电机 `DIR + PWM`。
- 转向控制：TopSpeed 舵机是 PWM 直接控角度，无前轮角度传感器闭环；卡丁是直流转向电机 + 绝对编码器，必须自写角度环。
- 科目一策略：TopSpeed 科目一不是卡丁绕桩倒库，不能直接搬。
- 后轮速度闭环：卡丁左右后轮来自编码器反馈，且当前仍只有后轮平均速度单环。
- 绝对编码器驱动：逐飞默认绝对编码器驱动不能直接套当前 SPI4/P22.0/P22.1/P22.3/P23.1 接线。

## 当前代码事实

| 文件 | 当前作用 | 状态 |
|---|---|---|
| `board_pins.h` | TC387 当前引脚事实来源 | 有效 |
| `cpu0_main.c` | 初始化和主循环调度 | 有效，主循环已刷新 `kart_steer_abs_update()` |
| `isr.c` | 5ms 控制节拍和 UART3 SBUS 回调 | 有效 |
| `kart_power.c/.h` | 三路动力输出、遥控实现 | 有效 |
| `kart_encoder.c/.h` | 左右后轮编码器读取 | 左后正常，右后待硬件排查 |
| `kart_steer_abs.c/.h` | SPI4 转向绝对编码器 | 代码协议正确，待硬件飞线/修板验证 |
| `kart_pid.c/.h` | PID 算子 | 已纳管 |
| `kart_control.c/.h` | 后轮平均速度环 | 已实现，默认关闭，等 `e1` |
| `kart_imu.c/.h` | Madgwick 6DOF yaw 解算 | 已实现，init 有静止标定 |
| `kart_calc.c/.h` | 几何与姿态辅助函数 | 已实现 |
| `kart_debug_uart.c/.h` | VOFA+ 输出和在线改参 | 已扩到 7 通道 |

## 第一阶段学习和调试顺序

1. 看 `cpu0_main.c` 和 `isr.c`，确认程序入口和 5ms 控制节拍。
2. 看 `board_pins.h`，确认硬件定义。
3. 只测三路电机开环，且悬空、低 PWM。
4. 测左右编码器方向和右后编码器是否有信号。
5. 测前轮绝对编码器 raw 是否稳定。
6. 测 IMU yaw reset 和短时漂移。
7. 开后轮速度环。
8. 写并测试前轮角度环。
9. 写 yaw 外环。
10. 路径记录、回放和科目一状态机。

## 技术风险

- `e0` 关闭速度环后电机停不住的问题未定位，通电前必须先查。
- 转向硬件已经发生打死烧毁事故，闭环前必须加限幅和停机验证。
- 右后轮编码器/驱动存在硬件疑点，不能把平均速度卡在 19~22 误判成 PID 参数问题。
- 第二版板转向角传感器 MOSI/CS 仍需修正，详见 `03_硬件排查与第二版PCB.md`。
