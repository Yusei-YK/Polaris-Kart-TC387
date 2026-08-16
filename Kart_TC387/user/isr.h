/*********************************************************************************************************************
* Kart_TC387 Opensourec Library 即（Kart_TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 Kart_TC387 开源库的一部分
*
* Kart_TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/


#ifndef _isr_h
#define _isr_h
#include "zf_common_headfile.h"

/* 5ms PIT 节拍计数器:cc60_pit_ch0_isr 每拍 +1,主循环协作式调度靠它对齐周期。
 * volatile:中断改、主循环读,禁编译器缓存。uint32 差分天然处理回绕。 */
extern volatile uint32 g_tick_5ms;

/* 协作式调度器运行时监测(cpu0_main.c 定义,VOFA 读):
 *   last_exec_us  —— 上一轮任务分发耗时(微秒)
 *   max_exec_us   —— 历史最大分发耗时(微秒),观察最坏情况
 *   overrun_count —— 漏周期累计(主循环一次跨过 >1 个 tick 即累加,不补跑) */
extern volatile uint32 g_sched_last_exec_us;
extern volatile uint32 g_sched_max_exec_us;
extern volatile uint32 g_sched_overrun_count;

/* 点阵屏 SYNC(P15.8=ERU_CH5)下降沿累计:诊断用。主循环每秒读一次清零 → SYNC_Hz,帧率=SYNC_Hz/14。
 * 判断:计数为 0 → SYNC 无边沿(整形链/芯片没吐 SYNC);低/抖 → 刷新率不足或丢中断。 */
extern volatile uint32 g_dot_sync_edges;










#endif

