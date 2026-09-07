# Polaris Kart · TC387 卡丁快跑

<img width="184" height="187" alt="Polaris Kart" src="https://github.com/user-attachments/assets/aecc110b-6373-4010-a015-ecd86fe12567" />

哈尔滨理工大学 北极星队(Team Polaris, HRBUST)· 卡丁快跑组
第二十一届全国大学生智能汽车竞赛 · 全国二等奖

基于 Infineon AURIX TC387 的卡丁车工程。惯导轨迹录制与复刻、引导员跟随、
视觉锥桶识别、人车联动的灯光与语音、踏板人工驾驶,四核分工,控制环 5 ms,
遥测 50 Hz。

## 先看这里

**实车完赛的固件是标签 `race-final-2026`。** 想复现赛场上的车,检出这个标签,
不要用分支尖端。赛后又加了 CH32 踏板盒驾驶和一轮注释订正,那些改动没有在
实车上跑过、也还没编译验证过,现在都在 `feature/pedal-ecu` 分支上。
各版本之间的差别记在 `CHANGELOG.md`。

## 工程概览

| 项 | 内容 |
|---|---|
| 主控 | Infineon AURIX TC387(TriCore,四核) |
| 工具链 | AURIX Development Studio + TASKING |
| Eclipse 工程名 | `Kart_TC387`(写在 `.project` 里,不是文件夹名) |
| 跟踪文件 | 829 个:本队 171,厂商库 658 |
| 提交 | 116 条,2026-06-04 至 2026-09-07 |
| 控制周期 | 速度环、转向内环 5 ms;任务与运动学 10 ms |
| 遥测 | VOFA+ JustFloat,460800,43 通道,50 Hz |
| 参数 | 存片上 flash,菜单里在线改,掉电不丢 |
| 动态内存 | 不用。全工程 `malloc`/`calloc`/`free` 零处使用 |

## 目录结构

```text
Polaris-Kart-TC387/
├── code/                  业务代码,按职责分层
│   ├── Kart_Config/       板级引脚、标定常量(引脚事实的唯一来源)
│   ├── Kart_Driver/       外设驱动封装:电机、编码器、相机、flash、鸣笛
│   ├── Kart_App/          应用层:速度环、转向环、里程推算、IMU、灯光语音
│   ├── Kart_Algo/         算法:PID、几何计算、图像预处理、视觉跟踪
│   ├── Kart_Decision/     决策:任务状态机、录制回放、跟随、菜单参数
│   ├── Kart_Debug/        调试:VOFA、硬件自测、轨迹绘制、开机动画
│   └── Kart_TPL/          第三方器件驱动:点阵屏、TLD7002 灯板
├── user/                  四核 main、中断向量表、多核调度
├── libraries/             厂商代码,不改动
├── docs/                  文档、测试清单、硬件资料
├── tools/                 上位机脚本:日志解析、轨迹仿真、素材生成
└── .cproject .project ... AURIX Studio 工程文件
```

## 运行模式

顶层模式机在 `code/Kart_Decision/kart_mission.c`,七个模式:

| 模式 | 做什么 |
|---|---|
| `MISSION_IDLE` | 安全待机。车轮、转向、蜂鸣器全部无输出 |
| `MISSION_SUBJECT_1` | 科目一绕桩:START 键触发,前向复现录好的路线 |
| `MISSION_SUBJECT_2` | 科目二人车交互:听语音口令,一条一条派发动作 |
| `MISSION_SUBJECT_3` | 科目三如影随形:跟随录轨 → 开环反向复现 → 盲盒固定动作 |
| `MISSION_REMOTE` | SBUS 遥控接管,调试和手动开车用 |
| `MISSION_FAULT` | 故障锁止 |
| `MISSION_PEDAL` | 踏板驾驶:人踩油门刹车,方向盘走机械连杆,转向电机不使能 |

模式切换统一走 `kart_mission_set_mode()`:先 exit 旧模式统一停机,再 enter 新模式,
保证互切不留残余输出。入口是 IPS200 菜单,调试口的 VOFA `m` 命令是备用。

科目三第一阶段的控制源由 `S3_FOLLOW_SRC` 编译期选:0 遥控、1 本地视觉认引导板、
2 收 TC4D7 送来的人体检测帧。出厂档走 1:`board_pins.h` 里 `PERSON_LINK_ENABLE` 是 0,
`kart_mission.h` 的 `#if` 就落到视觉分支。要换成 4D7 认人只改那一个宏,这里自动跟上。
三条并存的原因是现场哪条不灵都能改一个宏重烧退回去,第二阶段的开环倒车复现三者共用。

## 功能模块与源码对应

| 功能 | 源码 |
|---|---|
| 后轮速度闭环 | `code/Kart_App/kart_control.c` |
| 转向内环(角度)+ 外环(航向) | `code/Kart_App/kart_steer_ctrl.c` |
| 里程与位姿推算 | `code/Kart_App/kart_odom.c` |
| IMU 姿态解算 | `code/Kart_App/kart_imu.c` |
| 轨迹录制 / 复刻回放 | `code/Kart_Decision/kart_record.c` `kart_playback.c` |
| 科目任务状态机 | `code/Kart_Decision/kart_mission.c` |
| 引导员跟随 | `code/Kart_Decision/kart_follow.c` |
| 运动学与几何 | `code/Kart_Decision/kart_motion.c` `code/Kart_Algo/kart_calc.c` |
| 视觉识别与目标跟踪 | `code/Kart_Algo/kart_vision.c` `kart_vtrack.c` `kart_preprocess.c` |
| 人车联动 | `code/Kart_App/kart_person_link.c` |
| 踏板盒串口接入 | `code/Kart_App/kart_pedal.c` |
| 灯光 / 语音 / 鸣笛 | `code/Kart_App/kart_light.c` `kart_voice.c` `code/Kart_Driver/kart_horn.c` |
| 遥控 SBUS 接管 | `code/Kart_App/kart_remote.c` |
| 菜单、参数、flash 存取 | `code/Kart_Decision/kart_menu.c` `kart_params.c` `code/Kart_Driver/kart_flash.c` |
| VOFA 遥测与在线调参 | `code/Kart_Debug/kart_debug_uart.c` |
| 多核调度与视觉投递 | `user/kart_multicore.c` |

## 数据流

```text
传感器原始数据
  编码器 kart_encoder · IMU kart_imu · 转向绝对编码器 kart_steer_abs
  摄像头 kart_camera · 遥控 SBUS kart_remote · 踏板盒 kart_pedal
    -> 驱动层封装(code/Kart_Driver/)
底盘状态估计    kart_imu 姿态 + kart_odom 位置与里程
    ->
任务状态机      kart_mission / kart_record / kart_playback / kart_follow
    -> 目标速度 + 目标前轮角
闭环            kart_control 后轮速度 + kart_steer_ctrl 转向内环与航向外环
    ->
统一输出        kart_power 限幅、停机、遥控接管
    ->
三路 DIR + PWM
```

## 四核分工

| 核 | 分工 | 说明 |
|---|---|---|
| CPU0 | 主循环、5 ms 控制环、菜单状态、全部中断 | `isr_config.h` 里每一条 `*_INT_SERVICE` 都是 `IfxSrc_Tos_cpu0`(只有 `EXTI_CH2_CH6` 给了 DMA),中断全在这个核上。点阵屏由它的 1 ms PIT(`cc61_pit_ch0_isr`)扫 |
| CPU1 | 惯导核:IMU 姿态解算与里程推算 | 命令通道只为惯导而建:`KART_MC_CMD_IMU_UPDATE` 和 `KART_MC_CMD_ODOM_UPDATE` 两条,`core1_service()` 只处理这两件事 |
| CPU2 | IPS200 屏幕绘制 | 软件 SPI,刷一屏要推 61.8 万 bit ≈ 355 ms(按位周期推算,非实测),留在主核会挤掉 71 个控制拍 |
| CPU3 | 视觉识别(异步) | 一帧约 360 ms(推算,非实测),投递即返回,主核不等结果 |

多核调度与通道协议在 `user/kart_multicore.c`,各核入口在 `user/cpu1_main.c` ~
`cpu3_main.c`。

## 设计约束

这几条是工程一直守着的规矩,改代码时别破:

1. 主控制流程非阻塞。流程靠距离、航向、事件推进,不靠长 `delay` 硬等。
2. 硬件访问一律经 `code/Kart_Driver/` 封装,业务代码不直接碰 PWM/GPIO/SPI/UART。
3. 不用动态内存,不上操作系统。
4. 急停、关键传感器异常、路段超时,一律先停车,由 `kart_power` 统一执行。
5. 闭环调参必须有 VOFA 通道或日志支撑,不靠肉眼和手感猜。
6. 注释和文档以代码为准。对不上的时候改注释,不是改代码来迁就注释。

## 编译与烧录

1. 安装 AURIX Development Studio。安装与使用说明见
   `docs/硬件资料/AURIX_Studio使用说明书_逐飞V1.9.pdf`。
2. `File → Import → Existing Projects into Workspace`,根目录选本仓库。
   工程名是 `Kart_TC387`。
3. 选构建配置:`Debug` 带调试信息,`release build` 是比赛用的。
4. 烧录与调试直接用已入库的两份配置:`Kart_TC387 Debug.launch`、
   `Kart_TC387 release build.launch`,clone 下来就能跑,不用自己新建。
5. 上位机用 VOFA+,接调试串口,协议 JustFloat,波特率 460800,
   **通道数必须设成 43**,少一个通道整帧会错位。标定步骤见根目录
   `VOFA_标定操作.md`。

## 二次开发注意

- 引脚以 `code/Kart_Config/board_pins.h` 为准。文档里的引脚表都是历史记录,
  对不上时信头文件。
- 参数在菜单里改,自动存 flash。不要改参数槽位的名字、单位、量程和顺序,
  存进 flash 的是下标;顺序一动,旧车上存的参数全部错位。
- 菜单里的 `Load Default` 会把在线标定好的值一次冲掉,正常调试不要按。
- `libraries/` 是厂商代码,不改。要升级就整包替换,别做局部补丁。
- 改工程名用根目录的 `AURIX修改工程名称.bat`(教程是同目录的 `AURIX修改工程名称教程.txt`)。它最后会
  调用 `删除临时文件.bat`;改完名记得把两份 `.launch` 的文件名也跟着改。
- 本仓库自己写的源文件统一 CRLF,`libraries/` 保持上游的 LF,由
  `.gitattributes` 锁定,不要在 IDE 里全局转行尾。

## 文档

`docs/README.md` 是文档索引。先看这三份:

| 文档 | 内容 |
|---|---|
| `开发日志.md`(根目录) | 开发过程、当天遇到的问题和结论 |
| `docs/03_硬件排查与第二版PCB.md` | 硬件硬结论、v1/v2 引脚、走线错误与飞线修复 |
| `docs/04_Indoor_Test_Checklist.md` | 上车前的室内测试清单 |

待办和已知缺口在 `docs/TODO.md`,封箱前的检查项在根目录 `国赛封箱清单.txt`。

## 版权与使用声明

- 根目录 `LICENSE` 是 MIT,覆盖 `code/` `user/` `docs/` `tools/` 里本队自己写的部分。
- `libraries/` 是逐飞开源库与 Infineon iLLD / SFR,遵循各自上游许可。逐飞按
  GPL 3.0 发布,`user/cpu1_main.c` ~ `cpu3_main.c` 也保留了逐飞的 GPL 声明头。
  MIT 不覆盖这些文件。
- `docs/硬件资料/` 的三份 PDF 是厂商公开手册,版权归原作者,放在这里只为方便查阅。
- 参赛、学习、二次开发自便。商用请自行核对上游许可。

## 分支与标签

`main` 是主线。`feature/pedal-ecu` 是赛后的踏板驾驶与文档整理,尚未并入。
关键节点用标签标记:

| 标签 | 指向 | 是什么 |
|---|---|---|
| `race-final-2026` | `579a77a` | 国赛完赛版本(全国二等奖),实车跑的就是这一份 |
| `flash-s3-20260823` | `d050cdd` | 国赛前最后一次科目三烧录版本 |
| `freeze-20260726-light-voice` | `b035a35` | 灯光与语音功能冻结点 |
| `snapshot-pre-multicore-20260722` | `5f9e182` | 拆多核之前的快照 |
| `archive/teammate-main-20260722` | `fdc1214` | 队友侧分支归档,未并入主线 |

历史上做过一次目录上提,把编译工程从 `Kart_TC387/` 子目录移到了仓库根。如果你手上
是旧布局的副本,拉取后需要在 AURIX Studio 里重新 import 一次。

## 致谢

- 哈尔滨工程大学济海追风8队,比赛期间无私帮了我们很多。真心感谢。
- 东北大学秦皇岛分校 TopSpeed 队的
  [开源工程](https://github.com/Ryan-5853/NEUQ_TopSpeed_CrossCountry_TC377),
  以及东北大学秦皇岛分校公开的 IMU 姿态解算实现
  (知乎 @Morever,<https://zhuanlan.zhihu.com/p/656101554>)。
  真心感谢无私开源。
- 逐飞科技的 TC387 开源库、AURIX Studio 使用说明,以及 TLD7002 点阵屏例程
  (<https://gitee.com/seekfree/TLD7002_LED_Dot_Matrix.git>,当年固定在提交
  `335abb7`,GPL-3.0)。
- Infineon 的 iLLD 与 SFR 头文件。
