#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_
#include "zf_common_headfile.h"
#include "kart_calib.h"     /* 整车标定量(极性/尺度/转向零位/遥控端点)集中在那里 */

/* 本文件只放【引脚映射与外设资源分配】。
 * 实测标定量(电机/编码器极性、脉冲当量、转向零位与硬限位)已搬到 kart_calib.h,
 * 换齿轮/换编码器/换轮子只改那一个文件。 */

#define KART_STEER_DIR_PIN              (P21_2)
#define KART_STEER_PWM_PIN              (ATOM0_CH1_P21_3)
/* 2026-07-17 单轮开环诊断(L800/R800)实测:软件左通道原接 P02_5 组却驱动物理右轮,
 * 软件右通道原接 P02_7 组却驱动物理左轮 —— PWM/DIR 引脚组左右接反,是双 PI 跑飞的根因。
 * 方向(两侧均正转)、编码器(I9=左/I10=右)都对,故只把左右后轮引脚整组对调。 */
#define KART_LEFT_REAR_DIR_PIN          (P02_6)
#define KART_LEFT_REAR_PWM_PIN          (ATOM0_CH7_P02_7)
#define KART_RIGHT_REAR_DIR_PIN         (P02_4)
#define KART_RIGHT_REAR_PWM_PIN         (ATOM0_CH5_P02_5)

/* 电机极性 STEER/KART_LEFT/KART_RIGHT_MOTOR_SIGN → kart_calib.h 第二节 */

#define KART_LEFT_ENCODER_INDEX         (TIM2_ENCODER)
#define KART_LEFT_ENCODER_CH1           (TIM2_ENCODER_CH1_P33_7)
#define KART_LEFT_ENCODER_CH2           (TIM2_ENCODER_CH2_P33_6)
#define KART_LEFT_ENCODER_A_GPIO        (P33_7)
#define KART_LEFT_ENCODER_B_GPIO        (P33_6)
#define KART_RIGHT_ENCODER_INDEX        (TIM5_ENCODER)
#define KART_RIGHT_ENCODER_CH1          (TIM5_ENCODER_CH1_P10_3)
#define KART_RIGHT_ENCODER_CH2          (TIM5_ENCODER_CH2_P10_1)
#define KART_RIGHT_ENCODER_A_GPIO       (P10_3)
#define KART_RIGHT_ENCODER_B_GPIO       (P10_1)

/* 编码器极性 KART_LEFT/KART_RIGHT_ENCODER_SIGN → kart_calib.h 第二节
 * 脉冲当量 KART_LEFT/KART_RIGHT_ENC_PULSE_TO_M → kart_calib.h 第三节 */

#define KART_STEER_ABS_SPI_INDEX        (SPI_4)
#define KART_STEER_ABS_SPI_MODE         (SPI_MODE0)
#define KART_STEER_ABS_SPI_BAUD         (1000000)
#define KART_STEER_ABS_SPI_SCK_PIN      (SPI4_SCLK_P22_3)
#define KART_STEER_ABS_SPI_MOSI_PIN     (SPI4_MOSI_P22_0)
#define KART_STEER_ABS_SPI_MISO_PIN     (SPI4_MISO_P22_1)
#define KART_STEER_ABS_SPI_HW_CS_PIN    (SPI_CS_NULL)
#define KART_STEER_ABS_CS_GPIO_PIN      (P23_1)
#define KART_STEER_ABS_RAW_SHIFT        (4)

/* 转向零位与硬限位 KART_STEER_ABS_CENTER_RAW / LEFT_LIMIT_RAW / RIGHT_LIMIT_RAW
 * → kart_calib.h 第四节(软限位、外环上限、满舵半径都由那三个值派生) */

/* ---------------- IMU660RA(六轴,SPI_0)----------------
 * 最新网表与 zf_device_imu660ra.h 默认引脚一致。
 * 科目一只用陀螺+加速度做 6DOF yaw，不用磁力计。 */
#define KART_IMU660RA_SPI_INDEX         (SPI_0)
#define KART_IMU660RA_SPI_SCK_PIN       (SPI0_SCLK_P20_11)
#define KART_IMU660RA_SPI_MOSI_PIN      (SPI0_MOSI_P20_14)
#define KART_IMU660RA_SPI_MISO_PIN      (SPI0_MISO_P20_12)
#define KART_IMU660RA_SPI_CS_PIN        (P20_13)

/* v2 主板:无线模块数据脚使用 UART1(P33.12/P33.13)。
 * 2026-07-19 最新网表坐实:无线串口.1→P33.12(TX)、.3→P33.13(RX)、.4→P10.2(RTS)、.8→P11.6(RST)。
 * 注意:P33.12/P33.13 与摄像头 UART1 资源冲突,已确认无线与摄像头不同时用。
 */
#define BOARD_WIRELESS_UART_INDEX       (UART_1)
#define BOARD_WIRELESS_UART_TX_PIN      (UART1_TX_P33_12)
#define BOARD_WIRELESS_UART_RX_PIN      (UART1_RX_P33_13)
#define BOARD_WIRELESS_UART_BAUD        (115200)
#define BOARD_WIRELESS_UART_BAUD_FAST   (460800)
#define BOARD_WIRELESS_RTS_PIN          (P10_2)     /* 无线模块 RTS(暂未用),v2 网表不变 */
#define BOARD_WIRELESS_RST_PIN          (P11_6)     /* 无线模块复位(暂未用),v2 网表不变 */

/* VOFA 日志串口。
 * 2026-07-24 由 UART2(P14.2/P14.3)改到 UART10(P13.0/P13.1)对接无线模块。
 *
 * 2026-07-27 增加 UART0(P14.0/P14.1)选项:那天没带无线模块,只能用 USB-TTL 直插
 * 抓 VOFA。P13.0/P13.1 是无线模块排针位,不方便接 TTL 线,故改用空闲的 ASCLIN0。
 *
 * 2026-07-28 改回 0:带了无线模块,插 P13.0/P13.1 排针无线抓 VOFA,不用拖线。
 *   LOG_ON_UART0 = 1 → UART_0 / P14.0(TX) / P14.1(RX)  (USB-TTL 直插备用)
 *   LOG_ON_UART0 = 0 → UART_10 / P13.0 / P13.1(无线模块)  ← 当前
 *
 * 前提与已知风险:
 *   ① P14.0/P14.1 不在《尽量不要使用的引脚.txt》禁用表内(表里是 P14.2~P14.6);
 *   ② 全工程 ASCLIN0 无其他用户,uart0_rx_isr(isr.c)只做丢弃兜底,不冲突;
 *   ③ 【历史结论】cpu0_main.c 注释记着 2026-07-26 给 TLD7002 挪线时"UART0 实测
 *      收发不通"(ERR=1/RX=0/回环 0)。但那次是灯板 2M 半双工用法,与本处 460800
 *      单向 TX 不同,未必同因。若实测仍不通,把 LOG_ON_UART0 改回 0。
 *
 * 切到 UART0 后日志与语音不再共用外设 → BOARD_VOICE_SHARES_AUX_UART 自动变 0,
 * 进出科目二不再切波特率/停日志(语音仍独占 UART_10)。 */
#define LOG_ON_UART0               (0)

#if LOG_ON_UART0
#define BOARD_AUX_UART_INDEX            (UART_0)
#define BOARD_AUX_UART_TX_PIN          (UART0_TX_P14_0)
#define BOARD_AUX_UART_RX_PIN          (UART0_RX_P14_1)
#else
#define BOARD_AUX_UART_INDEX            (UART_10)
#define BOARD_AUX_UART_TX_PIN          (UART10_TX_P13_0)
#define BOARD_AUX_UART_RX_PIN          (UART10_RX_P13_1)
#endif
#define BOARD_AUX_UART_BAUD            (115200)
#define BOARD_AUX_UART_BAUD_FAST       (460800)

/* ---------------- 语音模块(科目二,2026-07-26 定案)----------------
 * 背景:语音原接 P33.12/13,那是 UART_1=ASCLIN1,与 TLD7002 灯板飞线
 * (P11.12/P11.10 @2M)是同一个硬件外设,谁后 init 谁改波特率 → 灯板灭。
 * 库里 UART_1 只映射到 ASCLIN1,软件无法共存,所以必须换外设。
 *
 * 为什么落在 P13.0/P13.1(UART_10=ASCLIN10):
 *   ① 全工程 ASCLIN10 只有 VOFA 日志一个用户,而比赛不接无线模块 → 该外设本就空闲;
 *   ② 语音模块直接插原无线排针即可,不用飞线;
 *   ③ 不动 ASCLIN1(灯板)、不动 ASCLIN3(SBUS 遥控)。遥控是唯一人工接管兜底,
 *      不能为让位语音而拔掉,所以放弃了 P15.6/15.7 方案。
 *
 * 使用前提(硬件):科二运行前拔掉无线模块,把语音模块插上,
 *   语音 TX → MCU P13.1(RX),语音 RX → MCU P13.0(TX)。
 * 使用前提(软件):UART_10 平时是 460800 跑日志,进科目二要重配到 115200 并停日志,
 *   见 kart_mission.c 的 mission_enter/mission_exit(MISSION_SUBJECT_2)。
 *
 * VOICE_ON_AUX_UART=0 可一键退回旧接法(P33.12/13),但那样灯板与语音仍不能共存。 */
#define VOICE_ON_AUX_UART          (1)

#if VOICE_ON_AUX_UART
#define BOARD_VOICE_UART_INDEX          (UART_10)
#define BOARD_VOICE_UART_TX_PIN         (UART10_TX_P13_0)
#define BOARD_VOICE_UART_RX_PIN         (UART10_RX_P13_1)
#else
#define BOARD_VOICE_UART_INDEX          (BOARD_WIRELESS_UART_INDEX)
#define BOARD_VOICE_UART_TX_PIN         (BOARD_WIRELESS_UART_TX_PIN)
#define BOARD_VOICE_UART_RX_PIN         (BOARD_WIRELESS_UART_RX_PIN)
#endif
#define BOARD_VOICE_UART_BAUD           (115200)    /* 语音模块固定 115200 8N1,不可改 */

/* 语音与日志是否共用同一外设:为 1 时进科二必须切波特率 + 停日志。
 * 2026-07-27:日志可切到 UART0,故不能再写死等于 VOICE_ON_AUX_UART ——
 * 两者都落在 UART_10 时才算共用。日志在 UART0 时科二无需停日志/切波特率。 */
#define BOARD_VOICE_SHARES_AUX_UART     (VOICE_ON_AUX_UART && !LOG_ON_UART0)

/* ---------------- TC4D7 人体视觉链路（2026-08-12）----------------
 * 387 跑不了人体检测模型（ARC PPU 向量内建，TriCore 编不过；即使改成标量也是
 * 百毫秒/帧），所以视觉打到 TC4D7，387 只通过串口拿结果。帧格式与设计取舍
 * 全在 code/Kart_App/kart_person_link.h 顶部。单向链路：387 不往 4D7 发东西。
 *
 * 硬件：4D7 侧 XH2.54-4P（1=VCC5V 2=GND 3=TX 4=RX），TX/RX 串 100R，
 *   经 SN74LVC2G34DCKR（VCC3V3 供电）缓冲 + 10K 上拉到 3.3V → 标准 3.3V TTL，
 *   与 Kart_TC387 的 ASCLIN 电平直接兼容，不需电平转换。
 *   【必须共地】P1 的 pin2 接到 387 的 GND。不共地时两边参考电平浮动，
 *   现象是 byte_count 乱涨但 crc_err 几乎等于 frame_ok（全是垃圾字节）。
 *   接线方向：4D7 TX → 387 RX，4D7 RX → 387 TX（本模块不发，TX 可悬空）。
 *
 * 为何不新开一个 UART：387 上没真正空闲的口。UART_0(P14.0/P14.1) 是 boot 相关脚
 *   （见下面 P14.x 警告），UART_3 是 SBUS 遥控 —— 那是唯一的人工接管安全绳，
 *   不能为让位拔。所以复用，两个选项：
 *
 *   PORT = LIGHT（默认） UART_1  P11.12(TX)/P11.10(RX)  灯板 TLD7002 飞线位
 *     视觉阶段拔灯板、插 4D7；跟随调完再插回灯板（把 ENABLE 改 0 就行）。
 *     两者物理上不同时存在，且本宏一开就把点阵屏整条链路（tld7002_init /
 *     tld7002_set_duty / CCU61_CH0 1ms 扇描 / uart1_rx_isr 的 tld7002_callback）全部静默，
 *     不可能出现两个主抢 ASCLIN1 波特率（2M 与 115200）的情况。
 *
 *   PORT = VOFA          UART_10 P13.0(TX)/P13.1(RX)  无线模块排针位
 *     比赛不让接无线模块，这个坐子比赛时本来就空。但调试时 VOFA 日志与
 *     语音模块也在这个口，真冲突 → 选它时必须同时 LOG_ON_UART0=1
 *     （日志改走 USB-TTL 直插），否则下面的 #error 会拦住。 */
/* 总开关。为何它在 board_pins.h 而不在 kart_person_link.h：
 * 下面 DOT_MATRIX_MUTED 的推导要用到它，而点阵屏/灯板驱动（Kart_TPL）
 * 只 kart_include board_pins.h，不能反过来 kart_include Kart_App 的头。
 * 放在这里也符合分层：“哪个外设归谁”本来就是 Kart_Config 的职责。
 *
 * 0 = 完全不出代码、不动 UART 归属，工程行为与没这个模块一模一样。
 * 1 = 接管 PERSON_LINK_PORT 选定的 UART；选 LIGHT 时点阵屏自动静默。
 * 默认 0：硬件没接 4D7 时开着毫无意义，而且会白白丢掉点阵屏与菜单回显。 */
/* 【视觉源一键切换】只改下面这一个值：
 *   0 = Kart_TC387 本地 SCC8660 视觉（S3_FOLLOW_SRC_VISION）
 *   1 = TC4D7 / PLINK 视觉（S3_FOLLOW_SRC_PLINK）
 * kart_mission.h 会自动联动控制源，不需要再改第二个宏。 */
#define PERSON_LINK_ENABLE         (0)

#define PERSON_LINK_PORT_LIGHT     (0)
#define PERSON_LINK_PORT_VOFA      (1)

#ifndef PERSON_LINK_PORT
#define PERSON_LINK_PORT           (PERSON_LINK_PORT_LIGHT)
#endif

#if (PERSON_LINK_PORT == PERSON_LINK_PORT_LIGHT)
/* 引脚与 TLD7002_UART_RX / TLD7002_UART_HLSIL 同一对（zf_device_tld7002.h）。
 * 注意那边名字叫 RX/HLSIL 是从灯板芯片角度命名的，对 MCU 而言
 * P11.12 是 TX、P11.10 是 RX，这里按 MCU 视角写。 */
#define BOARD_PERSON_LINK_UART_INDEX    (UART_1)
#define BOARD_PERSON_LINK_TX_PIN        (UART1_TX_P11_12)
#define BOARD_PERSON_LINK_RX_PIN        (UART1_RX_P11_10)
#else
#define BOARD_PERSON_LINK_UART_INDEX    (UART_10)
#define BOARD_PERSON_LINK_TX_PIN        (UART10_TX_P13_0)
#define BOARD_PERSON_LINK_RX_PIN        (UART10_RX_P13_1)
#endif

/* 点阵屏/灯板是否该全链路静默。它不是“手动开关”，而是从上面两个宏推出来的：
 * 只有“链路开了 且 选的是灯板口”时才要静默。推导而不手写的理由：
 * 这个条件要在 6 处用（cpu0_main / isr.c / dot_matrix_screen.c / kart_multicore.c 等），
 * 手写就是 6 个可能忘改的地方，而忘改的后果是两个主抢同一个 ASCLIN1：
 * 现象是一会儿收不到帧、一会儿灯板不亮，很难查。 */
#define DOT_MATRIX_MUTED           (PERSON_LINK_ENABLE && (PERSON_LINK_PORT == PERSON_LINK_PORT_LIGHT))

/* 语音模块是否该全链路静默。同样是推导而不是手写。
 * 为何需要它：语音模块就插在无线排针(P13.0/P13.1 = UART_10)上，
 * 而 4D7 选 VOFA 口时插的是同一个坐子 —— 硬件上就不可能共存，
 * 软件再去初始化它只会把链路搞死。具体两个坑：
 *   ① kart_voice_init() 在 cpu0_main 里比 kart_person_link_init() 晚，
 *      它的 uart_init() 末尾是 uart_rx_interrupt(n, 0) —— 把链路刚开的
 *      RX 中断又关掉了。现象是一个字节也收不到(CH39 恒 0)，
 *      而波特率、引脚、接线全是对的，极难查。
 *   ② kart_voice_poll() 的 uart_query_byte() 与链路 RX 中断抢同一个
 *      1 字节 FIFO，谁先取走另一方就永远收不到 → CRC 错漮天飞。
 * 代价：这个配置下科目二与科目三信号阶段的语音不工作。
 * 这不是回避，是把硬件事实写进代码：坐子被 4D7 占着，语音本来就没插。 */
#define VOICE_MUTED                (PERSON_LINK_ENABLE && (PERSON_LINK_PORT == PERSON_LINK_PORT_VOFA))

#if (PERSON_LINK_ENABLE && (PERSON_LINK_PORT == PERSON_LINK_PORT_VOFA) && !LOG_ON_UART0)
#error "kart_person_link 选了 VOFA 口(UART_10) 但日志也在 UART_10：把 LOG_ON_UART0 改 1，或把 PORT 改回 LIGHT"
#endif

/* ---------------- GPS(UART_3,交接文档 3.5)---------------- */
/* 主板有 GPS,科目一先跑纯惯导,GPS 仅作辅助/以后融合用。
 * 注意方向:GPS_TX→MCU_RX=P15.7,MCU_TX→GPS_RX=P15.6,所以 UART3 的 TX 是 P15_6、RX 是 P15_7。*/
#define BOARD_GPS_UART_INDEX            (UART_3)
#define BOARD_GPS_UART_TX_PIN          (UART3_TX_P15_6)
#define BOARD_GPS_UART_RX_PIN          (UART3_RX_P15_7)
#define BOARD_GPS_UART_BAUD            (115200)

/* ---------------- IPS200 屏幕(当前为软件 SPI)----------------
 * 2026-08-12 以 zf_device_ips200.h + 主板网表复核：屏幕已不走 SPI_2/P15.2~5。
 * SCK/MOSI 是普通 GPIO 软件翻转，因此能与无线模块的硬件 SPI_2 同时使用。 */
#define BOARD_LCD_USE_SOFT_SPI         (1)
#define BOARD_LCD_SPI_SCK_PIN          (P02_8)
#define BOARD_LCD_SPI_MOSI_PIN         (P20_3)
#define BOARD_LCD_RST_PIN              (P15_1)
#define BOARD_LCD_DC_PIN               (P15_0)
#define BOARD_LCD_CS_PIN               (P00_12)
#define BOARD_LCD_BL_PIN               (P33_9)

/* ================= 无线模块(SPI 版,图传用)2026-08-10 =================
 * 【为什么要接它】现场"认不到板子"的判断需要看【整帧图像】和阈值中间量,
 * 车上 2 寸屏看不了 —— 屏太小、且刷一屏几十毫秒不能进控制窗口。
 * 图传是后面所有视觉调参的前提设施,不是锦上添花。
 *
 * 【网表复核引脚(2026-08-12,Netlist_Schematic1_1_2026-08-12.tel)】
 *   排针 1 = P15.4    排针 3 = P15.3    排针 5 = P15.5
 *   排针 4 = P15.2    排针 6 = P15.8    排针 7 = GND
 *   排针 8 = GND      排针 9 = +5V      排针 10 = P33.5(RST)
 *   排针 2 = NC。与 WIFI6B21-SPI V2.0 手册的 1~10 脚定义逐项一致。
 *
 * 【资源约束,必须知道】
 *
 * 冲突一(安全关键,库默认踩的坑):
 *   zf_device_wifi_spi.h 出厂默认是 SPI_4 / P22.3(SCK) / P22.0(MOSI) / P22.1(MISO)
 *   / P22.2(CS) / P23.1(RST)。而本车 SPI_4 + P22.3/P22.0/P22.1 + P23.1(CS)
 *   正是【转向绝对值编码器】(见上面 STEER_ABS_*)。
 *   直接调库里的 wifi_spi_init() 会重配这条 SPI 并且把 P23.1 当复位脚拉动 ——
 *   转向角读数当场失效。转向角是软限位和回中的唯一依据,丢了就是失控。
 *   所以本工程【必须】覆盖这些宏,已在 zf_device_wifi_spi.h 就地改成下面这组值,
 *   并在那里留了原值注释。改库文件是有意为之:库函数里是硬编码宏,没有形参可传。
 *
 * IPS200 不冲突(2026-08-12 网表复核):
 *   无线模块走硬件 SPI_2 的 P15.2/3/4/5；IPS200 当前实际配置是软件 SPI，
 *   SCK=P02.8、MOSI=P20.3、CS=P00.12、BL=P33.9。上面的 BOARD_LCD_* 已同步
 *   为这组当前值。屏幕与图传可以同时启用。
 *   另外 P15.8 = 点阵屏 SYNC(DOT_MATRIX_SCREEN_SYNC_PIN, ERU_CH5_REQ1_P15_8),
 *   无线模块要它当 INT。【注意：这里原先写的"SYNC 已 exti_disable、让出来无代价"
 *   是错的】dot_matrix_screen_init() 末尾 (zf_device_dot_matrix_screen.c:732) 仍然会
 *   exti_init(SYNC, EXTI_TRIGGER_FALLING) 把 P15.8 配成下降沿中断；那句
 *   exti_disable 在 dot_matrix_screen_test_rows_static() 里面，被
 *   DOT_ROWS_TEST(=0) 卡着，正常开机流程从来没执行过。
 *   后果：插上无线模块后它把 P15.8 当 INT 频繁翻转，每个下降沿都进
 *   exti_ch1_ch5_isr；模块拔掉时该脚被下拉恒低、一个沿也不产生。
 *   这就是 2026-08-11 实测到的"拔了才有 VOFA 日志"。现已在
 *   cpu0_main.c 里 dot init 之后、kart_wifi_init() 之前补上 exti_disable，
 *   仅 WIFI_ENABLE=1 时生效（=0 时保留 SYNC 边沿诊断计数）。
 *   P33.5 除无线模块 RST 外无其他用途。
 *
 * WIFI_ENABLE 只控制图传本身；不再联动关闭 IPS200 菜单。 */
#ifndef WIFI_ENABLE
/* 2026-08-12 停用 WIFI6B21-SPI；图传改走下载器 UART_0(P14.0/P14.1)。 */
#define WIFI_ENABLE                (0)
#endif

/* 【引脚数值不写在这里,写在 zf_device_wifi_spi.h】
 * 反直觉,但只有这一个位置可行:库函数 wifi_spi_init() 用的是硬编码宏、没有引脚形参,
 * 必须在那个头文件里覆盖;而那个头文件被 zf_common_headfile.h:111 先 kart_include,
 * 本文件又 kart_include zf_common_headfile.h —— 在那里反向 kart_include 本文件会构成头文件环,
 * 让本文件在 SPI2_SCLK_P15_3 等枚举还没定义时被展开,报一堆对不上原因的错。
 * 所以分工是:数值在 zf_device_wifi_spi.h(带出厂默认值注释),取舍依据在这里。
 * 两处一致性由 kart_wifi.c 顶部的编译期断言强制 —— .c 里才能安全比较枚举常量,
 * 预处理器把 SPI_2/SPI_4 这类标识符当 0,写 #if 比较会恒真、永远误报。
 *
 * 当前分配(与 zf_device_wifi_spi.h 一致):
 *   SPI_2 @10MHz  SCK=P15.3  MOSI=P15.5  MISO=P15.4  CS=P15.2  INT=P15.8  RST=P33.5 */

/* ---------------- 按键板(2026-07-28 新板,按键板.tel 网表)----------------
 * 板上三个操作件,H1 是 2.54-2×6P 排线到主板:
 *   SW1  EC11 旋转编码器  A=P11.2  B=P11.3  按下(D)=P20.6   (C/E 脚接 GND)
 *   SW2  五向开关         UP=P33.11 DOWN=P20.0 KART_LEFT=P21.6 RIGHT=P21.7 MID=P33.4
 *   SW3  轻触开关         START=P20.7                        (3/4 脚接 GND)
 *
 * 电平:三个件的公共端全接 GND,按下/导通把信号脚拉低 → 按下读 0。
 * 上拉在主板侧(按键板上的 R1/4.7k 是 LED1 限流,不是按键上拉),故软件用
 * GPI_FLOATING_IN,不叠片内上拉。若日后换成不带上拉的主板,把 kart_menu_init /
 * kart_mission_init 里这几个脚改 GPI_PULL_UP 即可,逻辑不用动。
 *
 * 与旧板的差异(换板后同步改的地方):
 *   DOWN   P20.6 → P20.0    P20.6 让给旋钮按下
 *   MID    P20.7 → P33.4    P20.7 让给独立 START 键
 *   KART_LEFT   P33.4 → P21.6    旧板 P21.6 与 UP 短路,故当年拿 P33.4 顶 KART_LEFT,新板已修
 *   START  仍是 P20.7,但不再与菜单 MID 共用一个脚 ——
 *          就绪/科目三界面不必再屏蔽菜单 MID,kart_menu 与 kart_mission 也不会再对同一脚
 *          做两种 gpio_init(旧板 kart_menu 配 FLOATING、kart_mission 配 PULL_UP,谁后 init 谁生效)。
 *
 * 引脚占用已核对:P11.2/11.3 空闲(灯板占 P11.10/11.12,无线 RST 占 P11.6);
 * P20.0 空闲(isr.c 里 ERU_CH6_REQ9_P20_0 是注释掉的示例);P21.6/21.7 空闲
 * (转向占 P21.2/21.3);均不在《尽量不要使用的引脚.txt》(P14.2~14.6/P10.5/P10.6)内。 */
#define BOARD_KEY_UP_PIN               (P33_11)
#define BOARD_KEY_DOWN_PIN             (P20_0)
#define BOARD_KEY_LEFT_PIN             (P21_6)
#define BOARD_KEY_RIGHT_PIN            (P21_7)
#define BOARD_KEY_MID_PIN              (P33_4)

#define BOARD_ENC_A_PIN                (P11_2)
#define BOARD_ENC_B_PIN                (P11_3)
#define BOARD_ENC_SW_PIN               (P20_6)      /* 旋钮按下,当第二个 MID 用 */

/* 发车键(独立轻触开关)。上拉输入,按下接地读 0,科目一/科目三等此键触发。
 * 若实测按下读 1,把 kart_mission 里的边沿判据取反即可。 */
#define BOARD_START_KEY_PIN            (P20_7)

/* ---------------- 蜂鸣器 / ADC 检测(交接文档 3.6 / 5)---------------- */
#define BOARD_BEEP_PIN                 (P33_10)     /* 蜂鸣器输出 */
#define BOARD_MIC_ADC_CH               (ADC0_CH0_A0)    /* 硅麦采集(发车声控) */
#define BOARD_VBAT_ADC_CH              (ADC1_CH3_A11)   /* 电池电压检测,分压比待硬件标注 */

/* ---------------- 灯板 TLD7002 ---------------- */
/* 引脚定义不在这里 —— 见 zf_device_dot_matrix_screen.h(行译码 A0/A1/A2/EN、SYNC)
 * 与 zf_device_tld7002.h(TLD7002_UART_* = ASCLIN1 @2M,P11.12/P11.10)。
 * 注意:第一版曾把灯板挂 UART0/P14.0/P14.1,已废弃且不可复用 ——
 * P14.x 属 boot 相关引脚,占用会导致 MCU 下载不进去(见项目根 不建议使用的引脚.txt)。 */

#define KART_MAIN_LOOP_PERIOD_MS        (5)

#endif
