# TopSpeed 工程地图和卡丁迁移接收笔记

本文档是第一遍阅读后的结构化笔记。目标不是照抄 TopSpeed，而是把它的工程组织、控制节拍、数据流和调试方法迁移到卡丁快跑科目一。

## 0. 当前边界

- TopSpeed 参考工程：`G:\CODE\top speed\NEUQ_TopSpeed_CrossCountry_TC377-main`
- SmartCar 目标工程：`G:\CODE\Smart car\SmartCar`
- 当前建议落点：`G:\CODE\Smart car\SmartCar\6.6_V1_TC264`
- 新应用代码落点：`G:\CODE\Smart car\SmartCar\6.6_V1_TC264\code`
- 不修改：TopSpeed 原工程逻辑、SmartCar 的 `libraries`、逐飞底层库、英飞凌底层库、ADS 工程框架。

SmartCar 目标工程自己的说明要求用户代码直接放在 `code` 文件夹，不再创建代码子文件夹。因此后续即使迁移提示词里写了 `bsp/control/nav/mission` 这类分层目录，也应在当前 TC264 工程中用平铺文件名表达分层，例如 `kart_motor.c`、`kart_chassis.c`、`kart_subject1.c`。

## 1. 已阅读资料

- `kart_code_rewrite_handoff.md`
- `kart_hardware_handoff (1).md`
- `codex_kart_migration_prompt (2).md`
- TopSpeed `README.md`
- TopSpeed `TopSpeed_diary.md`
- TopSpeed 技术报告 `东北大学秦皇岛分校_TopSpeed_极速越野.docx`
- TopSpeed 关键源码：`user/cpu0_main.c`、`user/isr.c`、`TopSpeed_include.h`、`TopSpeed_Typedef.h`、PID、Power、IMU、GPS、GPS_INS、Mission、Subject_1、Subject_2、Menu、Route、Flash、Diary。
- SmartCar 目标工程：`6.6_V1_TC264` 的 `user`、`code`、`libraries/zf_driver`、`libraries/zf_device`、绝对编码器和 IMU 示例。

编码注意：TopSpeed 和 SmartCar 里存在混合编码。有些文件用 UTF-8 打开正确，有些注释要用系统默认/GB2312 才可读。后续遇到乱码时不能固定一种编码，要按文件内容重开判断。

## 2. TopSpeed 工程目录地图

### 根目录

- `.project`、`.cproject`：ADS/Eclipse AURIX 工程配置。
- `README.md`：工程说明。
- `TopSpeed_diary.md`：作者迭代日志，非常重要。能看出模块为什么被重写、策略为什么演进。
- `东北大学秦皇岛分校_TopSpeed_极速越野.docx/pdf`：技术报告，说明六层软件架构、硬件结构、传感器处理和调试经验。
- `Lcf_Tasking_Tricore_Tc.lsl`：链接脚本，属于工程框架，不迁移到 TC264。

### `user`

这是程序入口和中断入口。

- `cpu0_main.c`：主初始化和主循环。TopSpeed 的 CPU0 负责 Flash、GPS、菜单、按键、动力输出、任务调度等。
- `cpu1_main.c`：IMU、GPS_INS 等较重周期任务。
- `cpu2_main.c`：ESP、语音、麦克风等扩展任务。
- `isr.c`：中断回调集中处。串口、PIT、摄像头、DMA、外部中断都在这里进入用户逻辑。
- `isr_config.h`：中断优先级配置。

卡丁 TC264 只有双核，科目一第一版不需要照搬三核结构。建议先用 CPU0 单核跑通 1ms/5ms 控制，再考虑把 IMU 放到 CPU1。

### `code/TopSpeed_Driver`

这里不是逐飞底层驱动，而是 TopSpeed 自己写的“车辆驱动层”。

- `TopSpeed_PID.c/h`：PID 控制器。
- `TopSpeed_Power.c/h`：统一动力输出。任务层写 `Power_now`，最后由 `power_sync()` 输出。
- `TopSpeed_Com.c/h`：串口通信、驱动板通信、上位机通信。
- `TopSpeed_Flash.c/h`：Flash 参数和点位保存。
- `TopSpeed_Key.c/h`：按键和旋钮。
- `TopSpeed_Filter.c/h`：滤波器。

卡丁最重要的是学习“统一动力输出”和“底层封装”，但 `TopSpeed_Power.c` 不能直接复制，因为它控制的是越野车无刷驱动板和舵机，不是三路 DIR+PWM 有刷电机。

### `code/TopSpeed_App`

应用层负责把原始传感器数据变成决策层能用的量。

- `TopSpeed_IMU.c/h`：IMU 初始化、校准、yaw/姿态解算、yaw 归一化。
- `TopSpeed_GPS.c/h`：经纬度转本地坐标、点位采集、几何函数。
- `TopSpeed_GPS_INS.c/h`：GPS 和惯导组合估计。
- `TopSpeed_CTRV.c/h`：运动模型估计。
- 视觉、语音等模块：卡丁科目一第一阶段不迁移。

卡丁第一阶段不依赖 GPS 完赛，应先使用编码器距离和 IMU yaw。GPS 只保留为备份模式接口。

### `code/TopSpeed_Decision`

决策层负责“现在该干什么”和“目标量是多少”。

- `TopSpeed_Mission.c/h`：任务调度总入口。根据当前科目调用对应状态机。
- `TopSpeed_Subject_1.c`：越野科目一往返策略。
- `TopSpeed_Subject_2.c`：越野科目二绕桩策略，策略 13/14 对预瞄点和判定线很有参考价值。
- `TopSpeed_Menu.c/h`：树形菜单、参数调整、点位采集、任务启动。

卡丁科目一要重写自己的 `Kart_Mission` 和 `Kart_Subject1`。TopSpeed 的状态机组织方式可借鉴，路线和硬件输出不可照搬。

### `code/TopSpeed_Debug`

调试层是 TopSpeed 稳定的重要原因。

- `TopSpeed_Diary.c/h`：日志模块，使用 UART 和 FIFO 输出运行数据。
- `TopSpeed_Route.c/h`：路径记录。
- `TopSpeed_RC.c/h`：遥控接管和安全。
- 其他上位机、显示、绘图辅助模块。

卡丁第一版也必须有日志和室内自检，否则闭环参数很难调。

### `libraries`

逐飞库和英飞凌 iLLD 库。迁移时只能调用，不应修改。

## 3. SmartCar 目标工程地图

### `6.6_V1_TC264`

这是当前最适合作为卡丁代码目标的 TC264 工程。

- `user/cpu0_main.c`：当前是空模板，已经有 `clock_init()`、`debug_init()`、`cpu_wait_event_ready()` 和主循环框架。
- `user/cpu1_main.c`：当前基本空。
- `user/isr.c`：PIT、UART、摄像头、外部中断等 ISR 入口已经存在。
- `user/isr_config.h`：中断优先级。
- `code/board_pins.h`：当前板级引脚定义，包含电机、编码器、SPI3 绝对值编码器、IMU、GPS、无线、屏幕等。
- `code/zxs_imu963RA.c/h`：已有 IMU 姿态解算参考。
- `libraries/zf_driver`：GPIO、PWM、SPI、UART、PIT、ENCODER、FLASH 等外设库。
- `libraries/zf_device`：IMU、GNSS、IPS200、绝对值编码器等设备库。

### `Abs_Encoder_SPI_264Demo`

这是 TC264 绝对值编码器 SPI 示例。可用于理解读数流程，但它使用的库和当前 `6.6_V1_TC264` 不完全一致，不能直接搬。

### `imu963RA`

IMU 示例或资料目录，可作为调试参考。

### `Seekfree_TC264_demo_6.3`

较原始的逐飞 TC264 模板。当前建议优先使用 `6.6_V1_TC264`，因为它已经有 `board_pins.h` 和板级映射。

## 4. TopSpeed 程序启动流程

TopSpeed 的主流程在 `user/cpu0_main.c`。

简化后的流程是：

```text
clock_init
debug_init
Beep_init
printf/log 初始化
PIT 0.1ms 初始化，用于串口 FIFO 搬运
system_start
遥控、屏幕、Flash、GPS、菜单、按键、动力初始化
cpu_wait_event_ready
sys_inited = 1
PIT 1ms 初始化
while(TRUE)
    GPS 串口数据解析
    菜单刷新
    Mission_main 延迟事件处理
```

初学者要注意：主循环不是主要控制逻辑。真正的周期控制放在 PIT 中断里，这样每 1ms 或 5ms 稳定执行一次，不会被屏幕刷新、串口解析等慢任务拖乱节拍。

## 5. TopSpeed 周期任务

### 0.1ms

用于日志和 printf 串口 FIFO reload。含义是把软件缓冲区里的字节一点点搬进硬件串口 FIFO，避免一次 printf 阻塞主控制。

卡丁第一版可以先不用 0.1ms 高频日志，先用低频串口打印或 5ms/10ms 日志。

### 1ms

TopSpeed CPU0 的 1ms 中断中主要做：

- `Mission_timer_increase()`
- 按键扫描
- 蜂鸣器轮询
- 每 5ms 调用一次任务控制和动力输出
- 每 100ms 设置屏幕刷新标志

卡丁可以保留这个节拍思想。

### 5ms

TopSpeed 每 5ms 做：

- `Mission_CALLBACK()`
- `RC_sync()`
- `power_sync()`

5ms 等于 200Hz，是车辆控制常用频率。卡丁的转向角闭环、左右轮速度闭环、yaw 外环都可以先按 5ms 运行。

### 100ms

屏幕刷新标志。屏幕刷新慢，不能放在高频控制里做大量绘制。

## 6. TopSpeed 数据流

```text
传感器原始数据
    -> App 层处理
        -> GPS_Handle / IMU_Handle / Speed_encoder / GIE_Handle
            -> Decision 层状态机
                -> Power_now 目标输出
                    -> power_sync 统一限幅、安全、遥控接管
                        -> 执行器
```

卡丁应改成：

```text
编码器 / IMU / 前轮绝对角编码器
    -> kart_encoder / kart_imu / kart_abs_encoder
        -> kart_chassis
            -> kart_mission / kart_subject1
                -> kart_power_t
                    -> kart_power_sync
                        -> 三路 DIR+PWM 有刷电机
```

## 7. 可复用内容

### 可以直接学习或部分移植

- PID 控制器的结构和调用方式。
- yaw 角归一化到 `-180~180` 的思想。
- GPS 本地坐标、距离、角度、直线、投影等几何函数。
- Mission 总调度和 Subject 子状态机的分层方式。
- Flash 用结构体整页保存参数的方式。
- Diary 日志格式和 FIFO 异步输出思想。
- Route 路径记录思想。
- Menu 的“页面结构体 + 回调函数”思想。
- `Power_now -> power_sync` 的统一输出思想。

### 必须重写

- 动力输出。TopSpeed 是舵机 + 无刷驱动板串口协议，卡丁是三路有刷电机 DIR+PWM。
- Kart Subject1 状态机。TopSpeed 科目一是越野 GPS 往返，不是卡丁绕桩倒库。
- 转向闭环。卡丁需要前轮角度绝对编码器反馈，而不是普通舵机开环 PWM。
- 左右后轮速度闭环。TopSpeed 的速度来自无刷霍尔/驱动板，卡丁来自左右后轮正交编码器。
- 绝对值编码器驱动。SmartCar 默认绝对值编码器库使用 SPI0/P00.8，第一版卡丁板是 SPI3/P23.1。

## 8. 第一阶段卡丁学习顺序

1. 先理解 `cpu0_main.c`：程序从哪里开始。
2. 再理解 `isr.c`：为什么控制逻辑放在中断节拍里。
3. 学 PID：误差、比例、积分、微分、输出限幅。
4. 学 PWM 和 GPIO：为什么有刷电机需要 DIR+PWM。
5. 学编码器：为什么要先确认正反方向。
6. 学 IMU yaw：为什么 yaw 会漂，为什么每次发车要重置。
7. 学状态机：为什么不要把比赛流程写成一堆 delay。
8. 学日志：为什么能看数据比凭感觉调车可靠。

## 9. 当前最重要的工程判断

卡丁第一版不要追求完整高级导航。先做一个“低速、可停、可观测”的闭环平台：

```text
三路电机可单独控制
左右编码器方向正确
前轮绝对角读数稳定
IMU yaw 可重置
5ms 周期闭环稳定
路径可记录和回放
最后再接科目一状态机
```

这比直接搬 TopSpeed 的复杂 GPS 策略更可靠。
