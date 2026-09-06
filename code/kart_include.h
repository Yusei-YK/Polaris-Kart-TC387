#ifndef KART_INCLUDE_H_
#define KART_INCLUDE_H_

/*
 * 工程统一聚合头 —— 兼分层索引
 * ------------------------------------------------------------------
 * 【用法】新增模块的 .c 直接 kart_include 本文件一个就行,不用逐个挑头文件。
 *   现有 .c 保持原样、不做批量替换 —— 它们的 kart_include 链已经验证可用,
 *   为了"整齐"去动三十个文件不值得(改一次就多一次编不过的风险)。
 *
 * 【分组 = 物理目录】2026-08 起下面每个分组对应 code/ 下一个真实目录,
 *   看注释就知道文件该放哪。加新文件时先想清楚它属于哪层,
 *   放进对应目录,不要往 user/ 或 code/ 根目录随手扔。
 *
 *   code/Kart_Config    标定与板级:物理量、符号、引脚。全工程数值的唯一出处。
 *   code/Kart_Driver    直接操作硬件:PWM / 编码器 / SPI 读角 / 蜂鸣器 / Flash。
 *   code/Kart_Algo      纯计算,不碰硬件,可在 PC 上单测:几何、PID。
 *   code/Kart_App       状态估计与闭环:姿态、里程、速度环、转向串级环、
 *                       遥控接管、语音解析、灯板图案、外部视觉链路。
 *   code/Kart_Decision  决定"现在该干什么":科目状态机、菜单、参数表、
 *                       运动原语、路径录制与复现。
 *   code/Kart_Debug     只读观测与自检:VOFA 日志、硬件自测、轨迹可视化。
 *   code/Kart_TPL       第三方/器件驱动,尽量不改,方便跟上游对齐。
 *   user/               只留 cpu*_main / isr / isr_config / kart_multicore
 *                       —— 启动、中断向量、核间调度,不放业务逻辑。
 *
 * 【依赖方向】只许上层依赖下层,不许反向。
 *   Kart_Decision → Kart_App → Kart_Algo/Kart_Driver → Kart_Config
 *   例:kart_control 可以用 kart_pid,kart_pid 绝不能反过来引用 kart_control。
 *   违反这条会绕出循环依赖,也会让 Kart_Algo 没法单独在 PC 上测。
 *
 * 【头文件里放什么】宏定义、类型定义、函数声明。不放函数实现,
 *   不放变量定义(要跨文件共享就在 .h 里 extern、在 .c 里定义一次)。
 *
 * 【为什么这里不写目录前缀】code/* 每个目录都进了编译器 -I 搜索路径
 *   (.cproject 的 Kart_Debug 配置),所以 kart_include 只写文件名。挪目录时改那份
 *   -I 列表,不用回来改几十条 kart_include。
 * ------------------------------------------------------------------
 */

/* ---------------- 逐飞库 ---------------- */
#include "zf_common_headfile.h"

/* ---------------- Kart_Config ---------------- */
#include "kart_calib.h"         /* 整车标定量:几何/极性/尺度/零位/符号/遥控端点 */
#include "board_pins.h"         /* 引脚映射与外设资源分配 */

/* ---------------- Kart_Driver ---------------- */
#include "kart_power.h"         /* 三路 PWM 统一出口 + slew 限幅 */
#include "kart_encoder.h"       /* 后轮增量编码器 */
#include "kart_steer_abs.h"     /* 转向 SPI 绝对编码器 */
#include "kart_horn.h"          /* 蜂鸣器节拍机 */
#include "kart_flash.h"         /* DFlash 路径槽位持久化 */
#include "kart_camera.h"        /* SCC8660 彩色摄像头 + UART1 分时 + 采集诊断 */

/* ---------------- Kart_Algo ---------------- */
#include "kart_calc.h"          /* 几何/角度/四元数 */
#include "kart_pid.h"           /* 位置式 PID */
#include "kart_vision.h"        /* 黄色引导板检测(颜色分割 Detector) */
#include "kart_vtrack.h"        /* 视觉跟踪(LK 光流 Tracker + FB 一致性 + 中值 scale) */

/* ---------------- Kart_App ---------------- */
#include "kart_imu.h"           /* Madgwick 6DOF 航向解算 */
#include "kart_odom.h"          /* 航位推算 */
#include "kart_control.h"       /* 后轮速度环 */
#include "kart_steer_ctrl.h"    /* 转向串级环(航向外环 + 转角内环) */
#include "kart_remote.h"        /* SBUS 遥控解析 + 接管(兼当安全绳) */
#include "kart_pedal.h"         /* CH32 油门/刹车踏板盒 → 速度环(人坐车上开) */
#include "kart_voice.h"         /* 语音帧解析与命令队列 */
#include "kart_light.h"         /* 灯板点阵图案 */
#include "kart_person_link.h"   /* TC4D7 人体视觉链路（收 25 字节帧 → 合成 kart_vtrack）*/

/* ---------------- Kart_Decision ---------------- */
#include "kart_mission.h"       /* 科目状态机 */
#include "kart_menu.h"          /* 屏幕菜单 */
#include "kart_params.h"        /* 菜单在线可调参数表 */
#include "kart_motion.h"        /* 科目二运动原语 + GOTO 摆位 */
#include "kart_record.h"        /* 路径录制 */
#include "kart_playback.h"      /* 路径复现(Pure Pursuit / 开环倒回) */

/* ---------------- Kart_Debug ---------------- */
#include "kart_debug_uart.h"    /* VOFA 日志 + 串口命令 */
#include "kart_hw_test.h"       /* 上电硬件自测 */
#include "kart_traj_view.h"     /* 录制轨迹屏上可视化(默认 ENABLE=0 不出代码) */
#include "kart_wifi.h"           /* WiFi SPI 图传 + 逐飞助手上位机通道 */
#include "kart_bench.h"          /* Kart_TC387 实时性能基准测试矩阵 B1-B12 */

/* ---------------- Kart_TPL ---------------- */
#include "zf_device_dot_matrix_screen.h" /* 点阵屏 */
#include "zf_device_tld7002.h"           /* 灯板 TLD7002 逐飞层封装 */

#endif
