# SmartCar
<img width="184" height="187" alt="image" src="https://github.com/user-attachments/assets/aecc110b-6373-4010-a015-ecd86fe12567" />
# TC264 卡丁快跑组非视觉方案工程跟进文档

> 本文档用于记录本工程的比赛规则理解、代码结构规划、编程日志、已完成内容和未完成内容。后续 AI 或人工继续开发时，请优先阅读并更新本文档。

## 1. 项目背景

- 项目名称：TC264 逐飞库“卡丁快跑组”非视觉方案。
- 主控平台：英飞凌 TC264。
- 基础库：逐飞科技 TC264 开源库。
- 开发环境：AURIX Development Studio，使用 VS Code 辅助编辑。
- 工程目录：`Seekfree_TC264_demo_6.3/`。
- 用户代码目录：`Seekfree_TC264_demo_6.3/code/`。
- 文件编码：本工程文件按 GBK 编码处理。
- 总体方案：不使用摄像头视觉，主要依靠电机编码器、舵机角度编码器、IMU、TOF、GNSS/RTK、无线串口、遥控器和固定路线表实现比赛任务。

## 2. 比赛规则与任务理解

> 以下内容基于当前对“卡丁快跑组”任务的工程化理解整理。若后续拿到正式规则手册，需要同步修订本节。

### 2.1 总体约束

1. 不使用摄像头视觉作为控制输入。
2. 车辆需要通过非视觉传感器完成路线、动作和交互任务。
3. 所有任务必须使用非阻塞状态机实现，主循环不能被长时间 `delay` 卡死。
4. 所有硬件底层 API 必须通过应用层 wrapper 封装，业务代码不直接依赖逐飞底层库。
5. 不使用 `malloc`，不使用操作系统，不使用复杂动态资源管理。
6. 出现急停、TOF 过近、路线段超时、关键传感器异常时，必须立即停车并进入错误状态。

### 2.2 科目1：自动驾驶

目标：不用视觉，通过固定路线表、编码器距离、IMU 航向角、舵机角度反馈和 TOF 完成自动行驶任务。

当前理解的关键动作：

- 固定路线前进。
- 绕锥桶或类似障碍路线。
- 按指定距离转向。
- 使用 IMU 判断转弯角度。
- 使用 TOF 进行近距离防撞。
- 倒车入库：
  - 接近车库入口；
  - 前进摆角；
  - 固定舵角倒车；
  - 回正倒车；
  - 根据后向 TOF 距离停车。

### 2.3 科目2：人车交互

目标：先用无线串口或遥控器字符串模拟语音识别命令，后续可替换为真实语音模块。

计划支持命令：

- `LIGHT_LEFT`
- `LIGHT_RIGHT`
- `LIGHT_HIGH`
- `LIGHT_LOW`
- `LIGHT_FOG`
- `LIGHT_DOUBLE_FLASH`
- `LIGHT_INNER`
- `WIPER`
- `HORN_1S`
- `HORN_2S`
- `HORN_3S`
- `HORN_2_TIMES`
- `HORN_3_TIMES`
- `HORN_4_TIMES`
- `HORN_ALARM`
- `FORWARD_5M`
- `BACKWARD_2M`
- `SNAKE_FORWARD`
- `SNAKE_BACKWARD`
- `TURN_LEFT_90`
- `TURN_RIGHT_90`
- `CIRCLE_CW`
- `CIRCLE_CCW`

要求：灯光、蜂鸣器、雨刷、动作命令都必须写成非阻塞状态机。

### 2.4 科目3：路径记录与路径返回

第一阶段：人工遥控小车行驶并记录路径点。

路径点内容：

- `time_ms`
- `distance_m`
- `yaw_deg`
- `speed_mps`
- `steer_deg`
- `tof_front_m`
- `tof_rear_m`
- `gps_lat`
- `gps_lon`
- `gps_valid`

第二阶段：自动返回。

要求：

- 倒序读取路径点。
- 不简单倒放 PWM。
- 使用编码器距离误差控制速度。
- 使用 IMU 航向误差控制舵角。
- TOF 距离过近时减速或急停。
- GNSS/RTK 只作为低频位置修正。

## 3. 硬件平台与传感器规划

### 3.1 主控

- TC264。
- 逐飞 TC264 开源库。
- ADS 工程已能编译通过。

### 3.2 测距模块

优先级：DL1B > DL1A。

- `zf_device_dl1b.h`：TOF 测距模块 DL1B，100Hz，最大距离约 1.4m。
- `zf_device_dl1a.h`：TOF 测距模块 DL1A，33Hz，最大距离约 1.2m。

用途：

- 倒车入库停车判断。
- 防撞。
- 近距离检测。
- 前向、后向、侧向距离输入。

### 3.3 IMU / 陀螺仪

优先级：IMU963RA > IMU660RA > ICM20602 > MPU6050。

- `zf_device_imu963ra.h`：九轴 IMU，支持地磁计采集。
- `zf_device_imu660ra.h`：六轴 IMU。
- `zf_device_icm20602.h`：六轴陀螺仪，支持 SPI。
- `zf_device_mpu6050.h`：六轴陀螺仪，传统 IIC。

用途：

- 航向角估计。
- 角速度采集。
- 转弯角度判断。
- 路径返回时航向误差控制。

### 3.4 屏幕

只用于调试显示，不参与闭环控制。

- `zf_device_ips114.h`
- `zf_device_ips200.h`
- `zf_device_tft180.h`
- `zf_device_oled.h`

### 3.5 无线通讯

优先级：wireless_uart / wifi_uart。

- `zf_device_wireless_uart.h`
- `zf_device_wifi_uart.h`
- `zf_device_wifi_spi.h`
- `zf_device_ble6a20.h`
- `zf_device_bluetooth_ch9141.h`

用途：

- 调试输出。
- 命令输入。
- 模拟语音识别命令。
- 选择科目。

### 3.6 远程遥控

- `zf_device_uart_receiver.h`

用途：

- 科目3第一阶段手动驾驶路径记录。
- 科目选择。
- 急停或模式切换。

### 3.7 其他传感器

- `zf_device_absolute_encoder.h`：360 度绝对式角度编码器，分辨率约 0.088 度。
- `zf_device_menc15a.h`：15 位高精度磁编码器。
- `zf_device_gnss.h`：GPS/RTK 定位模块。
- `zf_device_key.h`：按键。
- `zf_device_virtual_oscilloscope.h`：虚拟示波器。

用途：

- 舵机实际角度反馈。
- 路径记录和返回。
- 低频位置修正。
- 调试和状态显示。

## 4. 工程结构分析

### 4.1 主入口

- CPU0 主入口：`user/cpu0_main.c`。
- CPU1 主入口：`user/cpu1_main.c`。
- 第一阶段只使用 CPU0，CPU1 暂不参与业务逻辑。

### 4.2 用户代码目录

- 应用层代码统一放在 `code/`。
- 不在 `code/` 下再建立子文件夹。
- 新增 `.c/.h` 后，需要在 ADS 中刷新工程并重新编译，确认 `code/` 源文件被自动加入构建。

### 4.3 逐飞库目录

- `libraries/zf_common/`
- `libraries/zf_driver/`
- `libraries/zf_device/`
- `libraries/zf_components/`
- `libraries/infineon_libraries/`

原则：不修改 `libraries/` 下的逐飞库底层文件。

### 4.4 总头文件

- 总头文件：`libraries/zf_common/zf_common_headfile.h`。
- 当前已包含常用驱动、设备和组件头文件。
- 不把我们自己的应用层头文件加进这个总头文件，避免污染逐飞库。

## 5. 代码结构规划

### 5.1 基础总控模块

计划新增：

- `app_config.h`
- `app_types.h`
- `app.h`
- `app.c`
- `app_time.h`
- `app_time.c`

职责：

- 全局配置。
- 通用结构体和枚举。
- 应用层初始化入口。
- 非阻塞时间管理。

### 5.2 传感器模块

计划新增：

- `sensor_manager.h`
- `sensor_manager.c`
- `imu_manager.h`
- `imu_manager.c`
- `tof_manager.h`
- `tof_manager.c`
- `encoder_manager.h`
- `encoder_manager.c`

职责：

- 所有传感器统一初始化和更新。
- 对外输出统一 `SensorData_t`。
- 底层 API 全部封装，不让业务层直接调用逐飞库函数。

### 5.3 控制模块

计划新增：

- `steering_control.h`
- `steering_control.c`
- `chassis_control.h`
- `chassis_control.c`
- `odometry.h`
- `odometry.c`
- `safety.h`
- `safety.c`

职责：

- 舵机控制。
- 电机速度闭环。
- 里程计估计。
- 安全保护。

### 5.4 路线与任务模块

计划新增：

- `route_executor.h`
- `route_executor.c`
- `mission_manager.h`
- `mission_manager.c`
- `mission_auto.h`
- `mission_auto.c`
- `mission_voice.h`
- `mission_voice.c`
- `path_record.h`
- `path_record.c`
- `path_return.h`
- `path_return.c`

职责：

- 路线表执行。
- 科目1自动驾驶。
- 科目2人车交互。
- 科目3路径记录和路径返回。

### 5.5 调试通信模块

计划新增：

- `debug_comm.h`
- `debug_comm.c`

职责：

- 封装 wireless_uart / wifi_uart / printf。
- 提供统一调试输出和命令输入接口。

## 6. 总体运行流程

目标主循环结构：

```c
System_Init();
App_Init();

while(1)
{
    SensorManager_Update();
    Odom_Update();
    MissionManager_Update();
    RouteExecutor_Update();
    ChassisControl_Update();
    SteeringControl_Update();
    Safety_Update();
}
```

在逐飞工程中，将对应放入 `user/cpu0_main.c` 的 `core0_main()` 中。

注意：

- `clock_init()` 和 `debug_init()` 仍保留在 `cpu0_main.c`。
- `System_Init()` 不重复初始化时钟和调试串口。
- 所有 `Update()` 必须非阻塞。
- 任何硬件初始化可以在 `App_Init()` 中执行，但主循环中不能重复初始化硬件。

## 7. 总体状态机规划

### 7.1 MissionManager 状态机

```text
INIT
  |
  v
WAIT_START
  |
  |-- 科目1 --> MISSION_AUTO ---------> FINISH
  |
  |-- 科目2 --> MISSION_VOICE --------> FINISH
  |
  |-- 科目3记录 --> MISSION_RECORD_PATH
  |
  |-- 科目3返回 --> MISSION_RETURN_PATH -> FINISH
  |
  +-- 故障 --> ERROR
```

状态说明：

- `INIT`：初始化完成后的内部过渡。
- `WAIT_START`：等待按键、遥控器或无线串口选择科目。
- `MISSION_AUTO`：执行科目1固定路线表。
- `MISSION_VOICE`：执行科目2模拟语音命令。
- `MISSION_RECORD_PATH`：手动驾驶并记录路径点。
- `MISSION_RETURN_PATH`：倒序跟踪路径点返回。
- `FINISH`：完成后停车。
- `ERROR`：故障急停并锁定。

### 7.2 RouteExecutor 状态机

```text
ROUTE_SEG_IDLE
  -> ROUTE_SEG_ENTER
  -> ROUTE_SEG_RUNNING
  -> ROUTE_SEG_DONE
  -> 下一段 ROUTE_SEG_ENTER
```

支持路线类型：

- `ROUTE_STOP`
- `ROUTE_FORWARD_DISTANCE`
- `ROUTE_BACKWARD_DISTANCE`
- `ROUTE_STEER_DISTANCE`
- `ROUTE_TURN_YAW`
- `ROUTE_SNAKE`
- `ROUTE_WAIT`
- `ROUTE_TOF_STOP`

每段路线进入时记录：

- 起始距离。
- 起始 yaw。
- 起始时间。
- 当前段索引。
- 当前目标参数。

### 7.3 Safety 状态机

```text
SAFETY_OK
  -> SAFETY_WARNING
  -> SAFETY_FAULT
```

保护策略：

- `SAFETY_OK`：正常。
- `SAFETY_WARNING`：降速或报警。
- `SAFETY_FAULT`：立即停车，MissionManager 进入 `ERROR`。

## 8. 已确认可复用的逐飞 API

后续全部通过 wrapper 封装。

### 8.1 IMU963RA

- `imu963ra_init()`
- `imu963ra_get_acc()`
- `imu963ra_get_gyro()`
- `imu963ra_get_mag()`
- 全局变量：`imu963ra_acc_x/y/z`、`imu963ra_gyro_x/y/z`、`imu963ra_mag_x/y/z`

### 8.2 IMU660RA

- `imu660ra_init()`
- `imu660ra_get_acc()`
- `imu660ra_get_gyro()`

### 8.3 DL1B / DL1A

- `dl1b_init()`
- `dl1b_get_distance()`
- `dl1b_distance_mm`
- `dl1a_init()`
- `dl1a_get_distance()`
- `dl1a_distance_mm`

### 8.4 编码器

- `encoder_quad_init()`
- `encoder_dir_init()`
- `encoder_get_count()`
- `encoder_clear_count()`

### 8.5 PWM

- `pwm_init()`
- `pwm_set_duty()`
- `PWM_DUTY_MAX`

### 8.6 无线串口和遥控

- `wireless_uart_init()`
- `wireless_uart_read_buffer()`
- `wireless_uart_send_string()`
- `uart_receiver_init()`
- 全局 `uart_receiver`

### 8.7 按键

- `key_init()`
- `key_scanner()`
- `key_get_state()`
- `key_clear_state()`

### 8.8 编码器与 GNSS

- `absolute_encoder_init()`
- `absolute_encoder_get_location()`
- `menc15a_init()`
- `menc15a_get_absolute_data()`
- `menc15a_get_speed_data()`
- `gnss_init()`
- `gnss_data_parse()`
- 全局 `gnss`

## 9. 当前已经完成的部分

截至 2026-06-04：

1. 已将 TC264 逐飞模板工程加入仓库。
2. 已在 AURIX Development Studio 中导入工程。
3. 已确认模板工程可以成功编译。
4. 已确认项目文件按 GBK 编码处理。
5. 已确认用户应用层代码应放在 `Seekfree_TC264_demo_6.3/code/`。
6. 已分析主入口：`user/cpu0_main.c`。
7. 已确认第一阶段不修改 `libraries/` 下逐飞库底层文件。
8. 已完成非视觉方案总体架构规划。
9. 已规划应用层模块列表和状态机结构。
10. 已创建本文档，用于后续持续跟进。

## 10. 当前还没有完成的部分

1. 尚未生成应用层 `.c/.h` 代码骨架。
2. 尚未修改 `user/cpu0_main.c` 接入应用层主循环。
3. 尚未验证新增 `code/` 下 `.c` 文件能否被 ADS 自动编译。
4. 尚未启用 IMU963RA 实测。
5. 尚未启用 DL1B / DL1A 实测。
6. 尚未启用电机编码器读取。
7. 尚未配置电机 PWM 和方向控制。
8. 尚未配置舵机 PWM 和舵机角度反馈。
9. 尚未实现速度 PID。
10. 尚未实现舵机闭环。
11. 尚未实现里程计融合。
12. 尚未实现路线表执行器。
13. 尚未实现科目1路线表。
14. 尚未实现科目2命令状态机。
15. 尚未实现科目3路径记录和返回。
16. 尚未实现完整安全保护模块。
17. 尚未进行上板联调。

## 11. 编程日志

### 2026-06-04

- 用户说明：工程所有文件使用 GBK 编码。
- 用户已安装英飞凌 AURIX Development Studio。
- 已指导用户导入 `Seekfree_TC264_demo_6.3` 工程。
- 用户反馈：模板工程编译通过。
- 已分析工程结构：
  - `user/cpu0_main.c` 为 CPU0 主入口；
  - `user/cpu1_main.c` 暂不使用；
  - `code/` 是推荐用户代码目录；
  - `libraries/` 不应修改。
- 已规划“卡丁快跑组”非视觉方案模块结构。
- 已创建本跟进文档。

## 12. 后续开发顺序

建议严格按以下顺序推进，每一步都先保证编译通过。

### 阶段1：最小骨架

1. 创建 `app_config.h`。
2. 创建 `app_types.h`。
3. 创建各模块 `.h/.c` 空实现。
4. 修改 `cpu0_main.c` 接入主循环。
5. ADS Clean + Build。

### 阶段2：时间与调度

1. 实现 `app_time`。
2. 所有模块 `Update()` 加入周期控制。
3. 确认没有长阻塞 delay。

### 阶段3：调试输入输出

1. 封装 wireless_uart / wifi_uart。
2. 支持基本命令：`status`、`start`、`stop`、`reset`。
3. 支持打印传感器状态。

### 阶段4：传感器验证

1. 验证 IMU963RA。
2. 验证 TOF DL1B。
3. 验证电机编码器。
4. 验证舵机角度编码器。
5. 可选验证 GNSS。

### 阶段5：执行器安全控制

1. 配置电机 PWM。
2. 配置舵机 PWM。
3. 实现限幅。
4. 实现急停。
5. 空载验证输出。

### 阶段6：闭环控制

1. 实现速度 PID。
2. 实现舵机角度闭环或开环映射。
3. 实现里程计。
4. 小速度实车验证。

### 阶段7：路线与科目

1. 实现 `route_executor`。
2. 实现科目1固定路线表。
3. 实现科目2命令动作状态机。
4. 实现科目3路径记录。
5. 实现科目3路径返回。

### 阶段8：安全与调参

1. 完整安全保护。
2. 路线参数调试。
3. 速度、舵角、TOF 阈值调试。
4. 比赛场地实测。

## 13. AI 跟进注意事项

1. 后续与用户交流全程使用中文。
2. 工程内文件按 GBK 编码处理。
3. 不修改逐飞库底层文件，除非用户明确授权。
4. 新增应用层代码优先放在 `code/`。
5. 不要编造不确定的逐飞库 API。
6. 不确定 API 时先写 wrapper，并用 TODO 标明需要用户替换或确认。
7. 不使用 `malloc`。
8. 不使用操作系统。
9. 不在主循环中使用长 `delay`。
10. 所有任务使用非阻塞状态机。
11. 编译错误优先级高于功能完整度。
12. 每次修改代码或完成硬件验证后，更新本文档的“编程日志”“已完成部分”“未完成部分”。
13. 若后续拿到正式比赛规则，应优先更新“比赛规则与任务理解”章节。

## 14. 当前下一步

建议下一步生成最小可编译代码骨架：

1. `app_config.h`
2. `app_types.h`
3. `app.h / app.c`
4. `sensor_manager.h / sensor_manager.c`
5. `imu_manager.h / imu_manager.c`
6. `tof_manager.h / tof_manager.c`
7. `encoder_manager.h / encoder_manager.c`
8. `odometry.h / odometry.c`
9. `route_executor.h / route_executor.c`
10. `mission_manager.h / mission_manager.c`
11. `chassis_control.h / chassis_control.c`
12. `steering_control.h / steering_control.c`
13. `safety.h / safety.c`

然后只轻改 `user/cpu0_main.c`，接入主循环并验证 ADS 编译。

## 15. 多传感器滤波与航向角融合规划

### 15.1 规划背景

去年方案曾使用 GNSS 打点导航，并使用 IMU 航向角与 GNSS 航向角进行卡尔曼滤波，最终得到融合航向角。本工程继续沿用“GNSS 打点 + 融合航向角”的非视觉导航核心。

逐飞库提供的 GNSS、IMU、TOF、编码器等原始数据不能直接用于控制，因此本工程将“滤波、异常剔除、航向角融合”作为控制前的核心基础层。任务层、路线层、控制层原则上只能使用融合后的车辆状态，不能直接使用原始传感器值。

### 15.2 新增滤波与融合模块文件列表

全部新增到 `Seekfree_TC264_demo_6.3/code/`，不修改逐飞库底层文件。

#### 15.2.1 通用滤波与角度工具

- `filter_common.h`
- `filter_common.c`

功能：

- 一阶低通滤波；
- 滑动平均滤波；
- 三点中值滤波；
- 角度限制到 `-180 ~ 180`；
- 角度限制到 `0 ~ 360`；
- 角度差计算；
- 通用限幅 `Clamp`。

计划接口：

```c
float Filter_LowPass(float input, float last, float alpha);
float Filter_Median3(float a, float b, float c);
float Angle_Wrap180(float angle);
float Angle_Wrap360(float angle);
float Angle_Diff(float target, float current);
float Clamp(float value, float min, float max);
```

#### 15.2.2 IMU 滤波模块

- `imu_filter.h`
- `imu_filter.c`

功能：

- 支持 IMU963RA、IMU660RA、ICM20602、MPU6050 的原始数据输入；
- 上电静止校准 `gyro_z_bias`；
- 对 `gyro_z` 做一阶低通；
- 根据 `gyro_z` 和 `dt` 积分得到 `imu_yaw_deg`；
- 支持 `IMU_ResetYaw(float yaw_deg)`；
- 输出 `imu_yaw_deg`、`gyro_z_dps`、`gyro_z_bias`、`imu_valid`。

注意：

- 所有陀螺仪单位统一为 `degree/s`；
- yaw 积分必须使用真实 `dt`；
- 静止校准期间不允许进入导航；
- IMU 数据异常时设置 `imu_valid = false`。

#### 15.2.3 GNSS 滤波模块

- `gnss_filter.h`
- `gnss_filter.c`

功能：

- 接收 GNSS 原始经纬度、速度、航向角、定位状态；
- 将经纬度转换为以起点为原点的局部坐标 `x_m / y_m`；
- 对 `x_m / y_m` 做一阶低通；
- 判断 GNSS 是否跳点；
- 判断 GNSS 模块输出航向角是否可信；
- 根据连续两个有效 GNSS 点计算 `course_calc_deg`；
- 输出位置、速度、GNSS 有效状态、航向角有效状态。

计划接口：

```c
void GNSS_Filter_Init(void);
void GNSS_Filter_SetOrigin(double lat, double lon);
void GNSS_Filter_Update(const GNSS_RawData_t *raw_data);
void GNSS_Filter_GetPosition(float *x_m, float *y_m);
float GNSS_Filter_GetHeading(void);
bool GNSS_Filter_IsHeadingValid(void);
```

#### 15.2.4 航向角融合模块

- `heading_fusion.h`
- `heading_fusion.c`

功能：

- 使用卡尔曼滤波融合 IMU `gyro_z` 与 GNSS heading；
- 状态量：
  - `x[0] = yaw_deg`
  - `x[1] = gyro_bias_dps`
- GNSS heading 有效时执行观测更新；
- GNSS heading 无效时只执行预测；
- 角度误差全部使用 `Angle_Diff()`，避免 `359° -> 0°` 跳变；
- 测量噪声 `R` 支持动态调整。

计划接口：

```c
void HeadingFusion_Init(float init_yaw_deg);
void HeadingFusion_Predict(float gyro_z_dps, float dt);
void HeadingFusion_UpdateGNSS(float gnss_heading_deg, float gnss_speed_mps, bool gnss_heading_valid);
float HeadingFusion_GetYaw(void);
float HeadingFusion_GetGyroBias(void);
void HeadingFusion_Reset(float yaw_deg);
```

#### 15.2.5 编码器滤波模块

- `encoder_filter.h`
- `encoder_filter.c`

功能：

- 读取电机编码器原始脉冲；
- 计算速度 `m/s` 和累计距离 `m`；
- 对速度做一阶低通或滑动平均；
- 判断编码器异常，例如长时间有 PWM 但速度接近 0；
- 输出 `encoder_speed_mps`、`encoder_distance_m`、`encoder_valid`。

#### 15.2.6 TOF 滤波模块

- `tof_filter.h`
- `tof_filter.c`

功能：

- 对 DL1A / DL1B 原始距离做滤波；
- 使用三点中值滤波 + 一阶低通；
- 过滤突变值；
- 输出 `front_tof_m`、`rear_tof_m`、`side_tof_m`；
- 支持倒车入库停车判断；
- 支持近距离防撞判断。

#### 15.2.7 GNSS 打点导航模块

- `waypoint_nav.h`
- `waypoint_nav.c`

功能：

- 基于 GNSS 打点进行路径点导航；
- 路径点结构体 `Waypoint_t` 包括：
  - `x_m`
  - `y_m`
  - `target_speed_mps`
  - `arrive_radius_m`
- 根据当前 `x/y` 和目标点计算 `target_yaw_deg`；
- 根据 `fused_yaw_deg` 计算 `yaw_error_deg`；
- `yaw_error_deg` 进入舵机控制；
- 到达目标点半径后切换下一个点；
- 支持路线完成判断。

#### 15.2.8 传感器融合管理模块

- `sensor_fusion_manager.h`
- `sensor_fusion_manager.c`

功能：

- 统一调用 `imu_filter`、`gnss_filter`、`encoder_filter`、`tof_filter`、`heading_fusion`；
- 统一输出融合后的车辆状态：
  - `fused_yaw_deg`
  - `x_m`
  - `y_m`
  - `speed_mps`
  - `distance_m`
  - `gnss_valid`
  - `imu_valid`
  - `heading_valid`
  - `tof_front_m`
  - `tof_rear_m`
- 任务层、导航层、控制层只能读取融合后的状态，不允许直接使用原始传感器值。

### 15.3 主循环数据流图

建议主循环调整为：

```c
while(1)
{
    SensorRaw_Update();             // 读取原始传感器
    SensorFusionManager_Update();   // 滤波和融合
    MissionManager_Update();        // 任务状态机
    WaypointNav_Update();           // GNSS打点导航
    ChassisControl_Update();        // 速度控制
    SteeringControl_Update();       // 舵机控制
    Safety_Update();                // 安全保护
}
```

总体数据流：

```text
逐飞库原始数据层
  |
  |-- IMU 原始 acc/gyro ---------------------> imu_filter
  |                                             |-- gyro_z_dps
  |                                             |-- imu_yaw_deg
  |                                             |-- imu_valid
  |
  |-- GNSS 原始 lat/lon/speed/course/fix ----> gnss_filter
  |                                             |-- x_m / y_m
  |                                             |-- course_deg
  |                                             |-- course_calc_deg
  |                                             |-- gnss_heading_valid
  |
  |-- 编码器原始脉冲 ------------------------> encoder_filter
  |                                             |-- speed_mps
  |                                             |-- distance_m
  |                                             |-- encoder_valid
  |
  |-- TOF 原始距离 --------------------------> tof_filter
                                                |-- front_tof_m
                                                |-- rear_tof_m
                                                |-- side_tof_m
                                                |-- tof_valid

imu_filter.gyro_z_dps + gnss_filter.heading
  |
  v
heading_fusion
  |
  |-- fused_yaw_deg
  |-- gyro_bias_dps
  |-- heading_valid
  v
sensor_fusion_manager
  |
  |-- fused_yaw_deg
  |-- x_m / y_m
  |-- speed_mps
  |-- distance_m
  |-- tof_front_m / tof_rear_m
  |-- 各类 valid 标志
  v
任务层 / 导航层 / 控制层 / 安全层
```

控制原则：

- 航向控制只能使用 `heading_fusion` 输出的 `fused_yaw_deg`；
- 导航目标航向由 `waypoint_nav` 根据 GNSS 局部坐标计算；
- 舵机控制输入为 `yaw_error_deg = Angle_Diff(target_yaw_deg, fused_yaw_deg)`；
- 安全模块使用融合后的 TOF、速度、GNSS/IMU/编码器有效性标志。

### 15.4 heading_fusion 卡尔曼滤波设计

#### 15.4.1 状态定义

```text
x[0] = yaw_deg        // 融合航向角，单位 degree
x[1] = gyro_bias_dps  // 陀螺仪 Z 轴零偏，单位 degree/s
```

协方差矩阵：

```text
P = [ P00  P01
      P10  P11 ]
```

过程噪声：

```text
Q = [ HEADING_KF_Q_YAW   0
      0                  HEADING_KF_Q_BIAS ]
```

观测噪声：

```text
R = HEADING_KF_R_GNSS_BASE * dynamic_scale
```

#### 15.4.2 预测模型

输入：

- `gyro_z_dps`
- `dt`

预测：

```text
yaw = yaw + (gyro_z_dps - gyro_bias_dps) * dt
gyro_bias = gyro_bias
```

角度处理：

```text
yaw = Angle_Wrap180(yaw)
```

线性化状态转移矩阵：

```text
F = [ 1   -dt
      0    1  ]
```

协方差预测：

```text
P = F * P * F^T + Q
```

展开实现时可使用 2x2 手写矩阵，避免动态分配。

#### 15.4.3 GNSS 观测更新

观测量：

```text
z = gnss_heading_deg
```

观测矩阵：

```text
H = [1  0]
```

残差必须用角度差：

```text
y = Angle_Diff(z, yaw)
```

创新协方差：

```text
S = P00 + R
```

卡尔曼增益：

```text
K0 = P00 / S
K1 = P10 / S
```

状态更新：

```text
yaw       = yaw + K0 * y
gyro_bias = gyro_bias + K1 * y
yaw       = Angle_Wrap180(yaw)
```

协方差更新：

```text
P = (I - K * H) * P
```

为降低数值误差，后续实现可采用 Joseph 形式；第一版可先用简单 2x2 更新，并注意保持 `P01/P10` 对称。

#### 15.4.4 GNSS 无效时的处理

当 `gnss_heading_valid == false` 时：

- 只执行 `HeadingFusion_Predict()`；
- 不执行 `HeadingFusion_UpdateGNSS()`；
- `fused_yaw_deg` 会短时间跟随 IMU 积分；
- 若 GNSS 长时间无效，安全层可降速或报警；
- 若 IMU 也无效，`heading_valid = false`，任务层不得继续导航。

#### 15.4.5 R 动态调整原则

`R` 越大，表示越不信任 GNSS 航向；`R` 越小，表示越信任 GNSS 航向。

建议规则：

```text
R = HEADING_KF_R_GNSS_BASE

如果 gnss_heading_valid == false：不更新
如果 speed_mps 较低：R *= 4 ~ 10
如果 speed_mps 较高且直行稳定：R *= 0.5 ~ 1
如果 GNSS 质量差：R *= 2 ~ 5
如果 GNSS 质量好：R *= 1
如果本次 course 跳变接近阈值：R *= 2 ~ 4
```

第一版实现可只根据速度和有效性调整，后续再接入 GNSS 质量指标。

### 15.5 GNSS 航向角有效性判断逻辑

GNSS heading 有两个来源：

1. GNSS 模块直接输出的 `course_deg`；
2. 连续两个有效 GNSS 点计算得到的 `course_calc_deg = atan2(dy, dx)`。

二者都不能直接相信，必须先进行有效性判断。

#### 15.5.1 基础有效条件

本次 GNSS 数据必须满足：

```text
gnss_fix_valid == true
speed_mps > GNSS_MIN_HEADING_SPEED
没有发生 GNSS 跳点
```

其中速度阈值建议初值：

```text
GNSS_MIN_HEADING_SPEED = 0.5f m/s
```

原因：GNSS 在低速或静止时，模块输出航向角通常漂移严重，不适合用于航向观测更新。

#### 15.5.2 连续点计算航向有效条件

若使用连续点计算航向，必须额外满足：

```text
两次有效 GNSS 点距离 > GNSS_MIN_HEADING_DISTANCE
```

建议初值：

```text
GNSS_MIN_HEADING_DISTANCE = 0.2f m
```

计算：

```text
dx = current_x_m - last_valid_x_m
dy = current_y_m - last_valid_y_m
distance = sqrt(dx * dx + dy * dy)
course_calc_deg = atan2(dy, dx) * 180 / PI
```

然后统一 wrap 到 `-180 ~ 180` 或 `0 ~ 360`。

#### 15.5.3 GNSS 跳点判断

若当前点相对上一有效点距离过大，判定为跳点：

```text
if distance_from_last_valid > GNSS_MAX_JUMP_DISTANCE:
    gnss_jump_detected = true
    本次位置不用于更新滤波位置
    本次 heading 不可信
```

建议初值：

```text
GNSS_MAX_JUMP_DISTANCE = 3.0f m
```

该阈值需要结合 GNSS 更新率和车速调试。例如车辆最大速度越高，阈值可适当增大。

#### 15.5.4 航向突变判断

若本次 GNSS 航向相对上次有效 GNSS 航向变化过大，也认为本次 GNSS heading 不可信：

```text
course_delta = fabs(Angle_Diff(current_course, last_valid_course))
if course_delta > GNSS_MAX_COURSE_CHANGE:
    heading_valid = false
```

`GNSS_MAX_COURSE_CHANGE` 第一版可先放在 `app_config.h` 中，例如 `60.0f`，后续根据车速和赛道转弯半径调参。

#### 15.5.5 模块 course 与连续点 course 的选择

建议第一版策略：

1. 若 `course_deg` 有效，且与 `course_calc_deg` 差值不大，则优先使用模块 `course_deg`；
2. 若模块 `course_deg` 与连续点 `course_calc_deg` 差异过大，则增大 R 或判定 GNSS heading 暂不可信；
3. 若模块 `course_deg` 不可信但连续点距离足够且未跳点，可使用 `course_calc_deg`；
4. 若两者都不可信，heading_fusion 只执行 IMU 预测。

判断示例：

```text
course_agree = fabs(Angle_Diff(course_deg, course_calc_deg)) < GNSS_COURSE_AGREE_MAX
```

第一版建议：

```text
GNSS_COURSE_AGREE_MAX = 45.0f
```

### 15.6 app_config.h 新增调参项

后续在 `app_config.h` 中集中放置：

```c
#define IMU_GYRO_LPF_ALPHA             (0.25f)
#define GNSS_POS_LPF_ALPHA             (0.35f)
#define ENCODER_SPEED_LPF_ALPHA        (0.30f)
#define TOF_LPF_ALPHA                  (0.35f)

#define GNSS_MIN_HEADING_SPEED         (0.50f)
#define GNSS_MIN_HEADING_DISTANCE      (0.20f)
#define GNSS_MAX_JUMP_DISTANCE         (3.00f)
#define GNSS_MAX_COURSE_CHANGE         (60.0f)
#define GNSS_COURSE_AGREE_MAX          (45.0f)

#define HEADING_KF_Q_YAW               (0.05f)
#define HEADING_KF_Q_BIAS              (0.001f)
#define HEADING_KF_R_GNSS_BASE         (4.0f)

#define WAYPOINT_ARRIVE_RADIUS         (0.50f)
#define NAV_YAW_KP                     (0.80f)
#define NAV_YAW_KD                     (0.05f)
#define MAX_STEER_DEG                  (30.0f)
```

以上数值均为初始建议值，必须通过实车调试修正。

### 15.7 当前规划变更记录

#### 2026-06-04 补充

- 明确本工程导航核心为“GNSS 打点 + 融合航向角”。
- 明确最终控制只能使用 `heading_fusion` 输出的 `fused_yaw_deg`。
- 新增滤波与融合模块规划：
  - `filter_common`
  - `imu_filter`
  - `gnss_filter`
  - `heading_fusion`
  - `encoder_filter`
  - `tof_filter`
  - `waypoint_nav`
  - `sensor_fusion_manager`
- 明确 GNSS 低速航向不可信，必须进行速度、距离、跳点、航向突变等有效性判断。
- 明确所有滤波模块必须非阻塞、标准 C、无 malloc、无 delay。

## 16. 卡丁快跑组比赛规则理解更新

### 16.1 规则来源

用户补充的规则来源为《第21届智能车竞赛卡丁快跑组比赛科目细则》，页面显示首次发布于 2025-11-11，并于 2026-06-03 修改。

参考链接：

- https://blog.csdn.net/zhuoqingjoking97298/article/details/154697989

> 后续若获取到组委会正式 PDF 或最新通知，应以正式规则为准，并同步更新本文档。

### 16.2 组别本质理解

卡丁快跑组不是传统“循迹跑圈”比赛，而是室外卡丁车综合任务组，重点考察：

1. 自动运行指定路线的能力；
2. 接收或理解人的命令并完成动作的能力；
3. 路径记录与自动返回能力；
4. 室外定位、运动控制、路径回放、人车交互、倒车入库和安全稳定性。

工程能力关键词：

```text
定位能力
运动控制能力
路径记录能力
路径回放能力
人车交互能力
倒车入库能力
安全稳定性
```

因此本工程不能只追求速度，更要重视定位、融合、鲁棒性和状态机稳定性。

### 16.3 科目1：自动驾驶

#### 16.3.1 场地与任务

科目1在室外操场进行，场地包括：

```text
发车区
锥桶绕行区
倒车入库区
```

车模需要从发车区自动出发，按照给定方向依次绕行所有锥桶，最后在倒车入库区完成倒车入库。

倒车入库停车区规则重点：

- 门口和库底四个角落都有锥桶标识；
- 门口宽度约 `1.5 m`；
- 车库长度约 `2 m`。

#### 16.3.2 工程理解

科目1重点不是简单直行，而是：

```text
出发定位
绕锥桶
转向控制
进入停车区
倒车入库
准确停车
```

非视觉方案建议：

- 使用 GNSS 打点确定大概路线和锥桶区位置；
- 使用编码器控制局部距离；
- 使用融合航向角控制车身方向；
- 使用 TOF 辅助近距离防撞和倒车入库停车；
- 倒车入库最后阶段不能只依赖普通 GNSS，应主要依靠 `编码器 + IMU/融合航向 + TOF`。

#### 16.3.3 对代码的要求

- 科目1路线应参数化，放在 `mission_auto.c` 或后续专门路线表中；
- 路线点、距离、舵角、速度、TOF 阈值不能写死在控制函数中；
- 倒车入库应拆分为多个非阻塞路线段：
  - 接近车库入口；
  - 前进摆角；
  - 固定舵角倒车；
  - 回正倒车；
  - 后向 TOF 停车。

### 16.4 科目2：战场救护 / 人车交互

科目2要求通过自然语言控制车模完成任务。规则说明参赛队员坐在驾驶座位上，正对车模上的硅麦说出抽取任务，车模需要识别语音并完成对应动作。

语音转文字可通过：

- WiFi 调用云端 AI；
- 离线语音芯片 + 英飞凌硅麦；
- 工程早期可先使用无线串口或遥控器字符串模拟语音命令。

#### 16.4.1 发车区灯光任务

规则包含八种灯光或雨刷命令：

```text
打开左转向灯
打开右转向灯
打开远光灯
打开近光灯
打开雾灯
打开双闪灯
打开车内照明灯
打开雨刷器
```

工程实现：

- 可用 LED、RGB 灯、屏幕图案或 GPIO 输出模拟灯光；
- 雨刷器可用灯标识，也可用舵机模拟摆动；
- 所有灯光和雨刷动作必须使用非阻塞状态机。

#### 16.4.2 停车区鸣笛任务

规则包含鸣笛命令：

```text
鸣笛1秒
鸣笛2秒
鸣笛3秒
鸣笛2声
鸣笛3声
鸣笛4声
长短鸣笛
急促鸣笛
警报鸣笛
```

规则重点：

- 喇叭必须使用 `2W` 以上无源扬声器，保证视频录制能听清。

鸣笛动作理解：

```text
鸣笛1/2/3秒：只响一次，持续对应时间
鸣笛2/3/4声：每声1秒，中间间隔1秒
长短鸣笛：鸣笛1秒，再鸣笛3秒，中间间隔1秒
急促鸣笛：响0.5秒，停0.5秒，次数5次以上
警报鸣笛：500Hz和1000Hz双频交替，每个频率持续1秒，交替5次以上
```

工程要求：

- 必须做成非阻塞蜂鸣器/扬声器状态机；
- 不能使用连续 `delay_ms()`；
- 鸣笛过程中主循环仍要执行安全保护和传感器更新。

#### 16.4.3 行进区通过门洞任务

从发车区到动作区的命令：

```text
通过门洞1左侧
通过门洞1
通过门洞2
通过门洞3
通过门洞3右侧
```

从动作区返回停车区的命令：

```text
门洞1右侧返回
门洞1返回
门洞2返回
门洞3返回
门洞3左侧返回
```

工程理解：

- 科目2不只是原地灯光和鸣笛，也包含指定路径行驶；
- 非视觉方案可实现为：

```text
语音/串口命令 -> 选择对应路线表 -> 编码器/融合航向/GNSS 执行固定路径
```

#### 16.4.4 动作区车模运动任务

动作命令包括：

```text
前行10米
后退10米
蛇形前进10米
蛇形后退10米
逆时针转一圈
顺时针转一圈
左转
右转
```

规则补充理解：

- 前行、后退、蛇形前进、蛇形后退，根据现场场地大小，只需要超过现场规定距离即可，例如大于 `5 m`；
- 蛇形运动要求轨迹左右摆动超过 `2 m`；
- 转一圈的转弯半径可以在 `3 m ~ 10 m`；
- 左转、右转要求车模行进超过 `2 m` 后完成转向并转正停止。

工程实现建议：

```text
前进/后退：编码器控距离
蛇形：舵机周期摆动 + 编码器控距离
转圈：固定舵角 + 融合航向判断完成360°
左转/右转：先前进超过2米，再转向，最后回正停车
```

### 16.5 科目3：如影随形 / 穿越迷宫

科目3分两个阶段，并且规则理解上最关键。

#### 16.5.1 第一阶段

规则描述：

- 车模从发车区出发；
- 跟随参赛队员按照规定迷宫路径绕行；
- 到达停车区；
- 分赛区比赛时，可能允许队员驾驶车模或遥控车模完成迷宫绕行；
- 国赛要求车模跟随队员完成迷宫绕行。

工程理解：

```text
分赛区 / 练习阶段：
可以优先采用遥控或人工驾驶记录路径。

国赛严格规则：
可能必须具备视觉跟随引导员能力。
```

#### 16.5.2 第二阶段

规则描述：

- 车模根据第一阶段路径；
- 从停车区自动出发；
- 绕行回到发车区。

工程核心：

```text
第一阶段记录路径
第二阶段自动返回
```

成绩为第一阶段和第二阶段时间之和。

#### 16.5.3 科目3重要限制

规则提到：参赛队员引导车模运行时，禁止携带电子设备进行引导和控制车模，并要求使用机器视觉跟随引导员；允许在引导员衣服上贴适当图案，方便车模识别跟踪。

这对当前非视觉方案有重要影响：

```text
如果只面向分赛区允许遥控/驾驶的阶段：
GNSS打点 + IMU + 编码器记录路径是可行路线。

如果目标是国赛严格规则：
完全不使用视觉可能无法满足“跟随引导员”的要求。
```

因此当前工程策略应明确为：

- 第一目标：完成分赛区/练习可用的非视觉路径记录与返回；
- 保留扩展接口：如果后续目标转向国赛，应增加简单视觉跟随方案，例如识别衣服图案、色块、AprilTag 或特定颜色标志。

### 16.6 场地规则与参数化要求

规则说明省赛或国赛具体场地尺寸会由组委会赛前公布，网页中的锥桶间距和场地尺寸更多是练习参考。

因此代码必须支持参数化：

```text
路线点可以现场修改
GNSS点可以现场重新打点
距离、半径、阈值统一放配置
速度、舵角、TOF阈值可快速调参
```

工程要求：

- `app_config.h` 存放核心参数；
- `mission_auto.c` 或路线表文件存放科目路线；
- `waypoint_nav.c` 支持 GNSS 点表导航；
- 后续可增加串口命令修改参数或选择路线。

### 16.7 对当前技术路线的影响

当前“卡丁快跑组非视觉方案”的重点调整为：

```text
科目1：GNSS/编码器/融合航向跑路线，TOF辅助倒车入库
科目2：语音或串口命令触发固定动作和固定路线
科目3：第一阶段记录路径，第二阶段自动返回
```

但是必须明确风险：

```text
科目3国赛可能要求视觉跟随引导员，纯 GNSS 打点方案可能只适合分赛区或练习阶段。
```

因此代码架构要做到：

- 当前不依赖视觉；
- 任务层保留视觉跟随数据接口；
- 路径记录与路径返回模块不与具体遥控方式强绑定；
- 如果以后加入视觉，引导员跟随只作为 `path_record` 的一种输入来源。

### 16.8 规划变更记录

#### 2026-06-04 补充

- 根据用户提供的比赛规则说明，重新补充卡丁快跑组三个科目的正式理解。
- 明确科目1包括室外自动驾驶、绕锥桶和倒车入库。
- 明确科目2包括灯光、鸣笛、门洞路径和动作区运动任务。
- 明确科目3是路径记录与自动返回，并指出国赛可能要求视觉跟随引导员。
- 明确当前纯非视觉方案更适合分赛区、练习阶段或允许遥控/驾驶记录路径的场景。
- 明确后续如果目标国赛，应保留视觉跟随扩展接口。
