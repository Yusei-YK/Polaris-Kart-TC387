/*********************************************************************************************************************
 * 文件名称  kart_wifi
 * 所属分层  Kart_Debug（只读观测层：不参与任何控制决策）
 * 功能说明  WiFi(SPI) 图传 + 逐飞助手上位机通道：整帧 RGB565 图像 / 叠加框点 / 示波器波形
 *
 * 【为什么必须有这个模块 —— 它是视觉调参的前提设施，不是附加功能】
 *   现场排查"认不到板子"，需要回答的是"这一帧里板子是什么颜色、判别式给了多少分、
 *   被哪道门限拒了"。这三件事都要看【整帧】，而车上 2 寸 IPS200 屏：
 *     1) 分辨率装不下 160×120 还要叠框叠字；
 *     2) 刷一屏逐行阻塞 SPI 几十毫秒，绝对不能进 5ms 控制窗口，
 *        所以只能在 IDLE 页看 —— 而"认不到板子"恰恰发生在车在跑的时候。
 *   把图发到电脑上看，是唯一能在车动着的时候看到真实输入的办法。
 *   在图传通之前做任何阈值调整，都是盲调。
 *
 * 【屏幕与图传可以同时使用】
 *   2026-08-12 网表复核：无线模块独占硬件 SPI_2 的 P15.2/3/4/5；当前 IPS200
 *   已改走软件 SPI P02.8/P20.3，CS=P00.12、BL=P33.9，不再占 P15.x。
 *   早期基于 P15.x 屏幕引脚的互斥判断已经网表证伪并撤销。
 *
 * 【带宽实算：为什么必须限帧，不能每帧都发】
 *   160×120 RGB565 = 38400 字节/帧，逐飞助手协议再加十几字节头。
 *   摄像头 30FPS 全发 = 1.15 MB/s。这个数在两个环节都过不去：
 *     SPI 侧：10MHz 理论 1.25MB/s，已经顶到 92% 占用率，且 wifi_spi_send_buffer
 *             是【阻塞】的，单帧 38400 字节按 4088 一包要发 10 包，
 *             每包之间还要等模块 INT 空闲 —— 实测耗时必然远大于理论 3.1ms。
 *     WiFi 侧：模块转发 TCP，实测吞吐通常 300-600 KB/s 量级，1.15MB/s 直接堵死。
 *   结论：图传帧率必须限。默认 WIFI_IMG_PERIOD_MS = 200（5FPS），
 *   调颜色阈值看 5FPS 完全够（人眼判断颜色不需要流畅），
 *   而且 5FPS × 38400 = 192 KB/s，两侧都留了 2 倍以上余量。
 *
 * 【最关键的一条：图传绝不能拖慢控制环】
 *   wifi_spi_send_buffer() 阻塞，单帧几十毫秒量级 —— 这个耗时进 5ms 拍必然漏拍，
 *   而漏拍会让速度环/转向环的积分项按错误的 dt 累积，是会让车跑飞的。
 *   所以本模块的发送【只允许】在主循环的空转段调用（kart_wifi_background_poll），
 *   与 kart_debug_uart_background_poll 同一个位置，且内部：
 *     1) 每次只发一小块（WIFI_CHUNK_BYTES），发完就返回，靠反复被调推进；
 *     2) 用状态机记住发到第几块，不在函数里循环等；
 *     3) 车在跑(非 IDLE)时只发示波器波形,不发图像。
 *        【但这道闸不在本模块里】判断写在 cpu0_main.c 请求图像那一段:
 *        if(MISSION_IDLE == kart_mission_get_mode()) 才累 50ms 计数、
 *        才调 kart_wifi_request_image()。本模块的 background_poll 只认
 *        "有没有待发的帧",谁请求它就发。把 poll 搬到别处或自己加请求点时,
 *        这道闸会一起丢掉,必须重新加。
 *   这样最坏情况下单次调用只阻塞一小块的时间，而不是整帧。
 *   【严禁】把 wifi_send_image() 直接放进 kart_task_5ms/10ms。
 *
 * 【为什么示波器通道比图像更重要】
 *   图像回答"看到了什么"，示波器回答"算出了什么"。真正难查的问题是
 *   "图看着挺好，为什么 bearing 是错的" —— 那需要 bearing / scale_r /
 *   置信度 / 存活点数在【同一条时间轴】上对齐看。逐飞助手的示波器通道
 *   正好干这个，而且它数据量极小（每包几十字节），可以高频发。
 *   所以示波器默认 20ms 一包（50Hz），图像 200ms 一帧。
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-08-10        Kart                首版：图传通道 + 示波器，未接硬件验证
 ********************************************************************************************************************/

#ifndef KART_WIFI_H_
#define KART_WIFI_H_
#include "zf_common_headfile.h"
#include "board_pins.h"

/*=========================== 总开关 ===========================*/
/* WIFI_ENABLE 定义在 board_pins.h（与引脚冲突说明放在一起）。
 * 这里只做二次确认，避免有人单独包本文件时拿到未定义的宏。 */
#ifndef WIFI_ENABLE
#define WIFI_ENABLE                (0)
#endif

/*=========================== WiFi 连接参数 ===========================*/
/* 【必须按现场实际填】默认按"电脑开 Windows 移动热点"这种最省事的接法：
 *   电脑热点 SSID/密码填下面两个，电脑在热点网络里的 IP 固定是 192.168.137.1，
 *   逐飞助手上位机在电脑上监听 8086 端口，车作为 TCP 客户端连上去。
 * 换成路由器接法时，把 TARGET_IP 改成电脑在路由器下的实际 IP（ipconfig 看）。
 *
 * 【连不上时的排查顺序，别乱试】
 *   1) g_wifi_init_ret != 0 且 wifi_spi_version 为空串 → SPI 都没通，
 *      是引脚问题 —— 但【别找 WIFI_SWAP_CS_MISO 这个宏,本仓库里没有】,
 *      早期设想的那个开关始终没实现。真接反了只能改 board_pins.h 里
 *      WIFI_SPI 那几个引脚宏,或者直接改接线;
 *   2) version 读到了但 g_wifi_link != KART_WIFI_LINK_SOCKET_OK →
 *      SPI 通了、WiFi 或 TCP 没通，看 wifi_spi_ip_addr_port 有没有拿到 IP：
 *        没拿到 IP → SSID/密码错，或热点是 5G 频段（模块只支持 2.4G）；
 *        拿到 IP 但 socket 失败 → 上位机没开、端口不对，或电脑防火墙拦了。
 *   3) 【最常见】Windows 移动热点的网段不一定是 192.168.137.x，
 *      开了热点先 ipconfig 确认一次，别默认它是 137。 */
#define WIFI_SSID                  "ap"
#define WIFI_PASSWORD              "12345678"
#define WIFI_TARGET_IP             "192.168.137.1"
#define WIFI_TARGET_PORT           "8086"
#define WIFI_LOCAL_PORT            "6666"

/* 上电时要不要卡在这儿等 WiFi 连上。
 * 【这个宏没有任何人读它】全仓库只有下面这一行定义,改成 1 也不会有任何变化 ——
 * kart_wifi_init() 里没有 #if WIFI_BLOCK_UNTIL_LINK 这样的分支。留着只是
 * 记录当初的取舍。现状等于"不等":init 调一次 wifi_link_up() 就返回,
 * 连不上交给后台重连。
 * 当初为什么定成不等:连不上 WiFi 绝不能影响跑车 ——
 * wifi_spi_wifi_connect 内部最长可能等好几秒，若放在 pit_ms_init 之前阻塞，
 * 现场没开热点就等于车打不着火。改成后台重连，见 kart_wifi_background_poll。 */
#define WIFI_BLOCK_UNTIL_LINK      (0)

/* 后台重连间隔(ms)。连不上时不要死命重试 —— 每次 socket_connect 都阻塞几百毫秒，
 * 频繁重试会把主循环空转段吃光，反而影响日志排空。3s 一次足够。 */
#define WIFI_RECONNECT_MS          (3000)

/*=========================== 发送节奏 ===========================*/
/* 图像发送周期(ms)。见文件头"带宽实算"。200ms = 5FPS。
 * 【这个宏由 cpu0_main.c 执行,不是本文件】50ms 任务里累 wifi_img_elapsed_ms,
 * 到了就调一次 request_image。本文件里那个 img_period_last_raw 是残留:
 * init 里赋过一次值,之后再没人读过,删掉不影响任何行为。
 * 【想看更流畅时改这里,但先看 g_wifi_img_us 实测单帧耗时】
 * 若单帧耗时已经接近本周期,说明带宽顶满了,再调小只会挤掉主循环空转段。 */
#define WIFI_IMG_PERIOD_MS         (200)

/* 示波器发送周期(ms)。数据量小(几十字节)，可以高频。
 * 20ms = 50Hz，足够看清 bearing 的抖动和控制响应。 */
#define WIFI_OSC_PERIOD_MS         (20)

/* 单次 background_poll 最多发多少字节。
 * 【这个数决定最坏阻塞时长，是本模块最重要的安全参数】
 *   4088 是逐飞 wifi_spi 协议单包上限(WIFI_SPI_TRANSFER_SIZE)，超了库内部会拆。
 *   按 10MHz SPI 算，4088 字节理论 3.3ms —— 但主循环空转段没有硬时限
 *   (它本来就是在等下一个 5ms tick)，只要单次 < 5ms 就不会造成漏拍。
 *   取 2048：理论 1.6ms，留一倍余量给模块 INT 等待。
 *   实测 g_wifi_chunk_max_us 若超过 4000us，把这个值减半。 */
#define WIFI_CHUNK_BYTES           (2048)

/*=========================== 链路状态 ===========================*/
typedef enum
{
    KART_WIFI_LINK_OFF = 0,        /* 未启用(WIFI_ENABLE=0)或没调 init */
    KART_WIFI_LINK_SPI_FAIL,       /* SPI 不通：读不到模块版本号。查引脚/试 SWAP_CS_MISO */
    KART_WIFI_LINK_WIFI_FAIL,      /* SPI 通了，WiFi 没连上：SSID/密码错，或热点是 5G */
    KART_WIFI_LINK_SOCKET_FAIL,    /* WiFi 连上了，TCP 没连上：上位机没开/端口错/防火墙 */
    KART_WIFI_LINK_SOCKET_OK       /* 全通，可以发图 */
}kart_wifi_link_enum;

/*=========================== 诊断计数（只读） ===========================*/
extern volatile kart_wifi_link_enum g_wifi_link;
extern volatile int    g_wifi_init_ret;        /* wifi_spi_init 返回码，0 = 成功 */
extern volatile uint32 g_wifi_img_sent;        /* 已发出的完整帧数 */
extern volatile uint32 g_wifi_img_skip;        /* 因忙/未连接被跳过的帧数 */
extern volatile uint32 g_wifi_img_us;          /* 最近一帧从开始到发完的总耗时(us)，含被打断的等待 */
extern volatile uint32 g_wifi_chunk_max_us;    /* 单次 chunk 发送最长耗时(us)。>4000 就把 CHUNK_BYTES 减半 */
extern volatile uint32 g_wifi_osc_sent;        /* 已发出的示波器包数 */
extern volatile uint32 g_wifi_reconnect_cnt;   /* 重连尝试次数 */

/*======================================== 对外接口 ========================================*/

/* 上电初始化。
 * 【位置约束】必须在 pit_ms_init(CCU60_CH0, ...) 之前 —— 内部可能阻塞数秒。
 * 【和 kart_camera 的先后】init 本身不记任何图像地址,这一条不是 init 的约束;
 * 真正依赖摄像头的是 kart_wifi_request_image():它调 kart_camera_frame_ready()
 * 判断有没有新帧,再 memcpy 一份 scc8660_image[0],然后立刻 release。
 * 所以只要第一次请求图像发生在 kart_camera_init() 之后就行,而请求点在
 * 50ms 任务里,天然晚于初始化。
 * 返回 0 成功;非 0 表示没连上,但【不影响跑车】,后台会继续重连。
 * WIFI_ENABLE=0 时本函数直接返回 1 并把 g_wifi_link 置 OFF。 */
uint8 kart_wifi_init                (void);

/* 后台推进。【只能放在主循环空转段】，与 kart_debug_uart_background_poll 同处。
 * 内部按 CHUNK_BYTES 分块推进图像发送、按周期发示波器、断线时按周期重连。
 * 单次调用最坏阻塞约 CHUNK_BYTES/SPI速率，不进 5ms 拍就不会漏拍。
 * 【严禁放进 kart_task_5ms / 10ms / 50ms】 */
void kart_wifi_background_poll      (void);

/* 请求发送一帧图像。非阻塞，只是置个请求标志，真正发送在 background_poll。
 * 上一帧还没发完时直接返回 0 并累加 g_wifi_img_skip —— 宁可丢帧，
 * 绝不排队堆积(排队会让显示的图越来越滞后，比丢帧更难判断)。
 * 返回 1 = 请求已受理。
 *
 * 【谁来调】kart_task_50ms 里按 WIFI_IMG_PERIOD_MS 限频调用。
 * 放 50ms 拍而不是 10ms 拍：这只是置标志，但取帧要和 kart_camera 的 hold/release
 * 配合，放低频拍减少与视觉处理抢帧的概率。 */
uint8 kart_wifi_request_image       (void);

/* 往示波器通道填一个值。ch 范围 0 到 SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT-1。
 * 只写内部缓存，不发送；发送由 background_poll 按周期做。
 * 纯赋值，可以安全地在 5ms 拍里调。 */
void kart_wifi_osc_set              (uint8 ch, float value);

/* 设置示波器通道数。默认 WIFI_OSC_CH_NUM，一般不用改。 */
void kart_wifi_osc_set_channel_num  (uint8 num);

/* 配置图像叠加的边界点(画外接框用)。传 NULL 关闭叠加。
 * 【内部是拷贝,不存指针】dot_num 是点数,x/y 的内容会被拷进模块自己的
 * uint8 数组,调用方的局部数组可以立刻失效。代价是三条隐式截断:
 *   · 点数超过 WIFI_DOT_MAX(32) 的部分直接丢掉;
 *   · 坐标大于 255 的钳到 255(协议里坐标位宽是 8 位);
 *   · 不报错、不计数,只是画出来的框位置不对。
 * 【当前没有任何调用点】全仓库没人调它,所以下面这套叠加框在实际运行中
 * 从没出现过 —— 想用得自己在视觉侧调一次。
 * 【为什么要叠框】只看图判断不了"检测器认为板子在哪" ——
 * 图上有块红、检测器却报 valid=0,不叠框就分不清是判别式没过还是门限拒了。 */
void kart_wifi_set_boundary         (uint16 dot_num, uint16 *x, uint16 *y);

kart_wifi_link_enum kart_wifi_link_state (void);

#endif
