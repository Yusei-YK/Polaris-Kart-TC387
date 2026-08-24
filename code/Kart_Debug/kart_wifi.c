/*********************************************************************************************************************
 * 文件名称  kart_wifi
 * 功能说明  WiFi(SPI) 图传实现。设计取舍与冲突分析见 kart_wifi.h 与 board_pins.h。
 *
 * 【为什么不直接用 seekfree_assistant_camera_send()】
 *   那个函数内部是：
 *       transfer_callback(帧头, 8);
 *       transfer_callback(图像首地址, 38400);   ← 一次性把整帧压进去
 *   而 transfer_callback 就是 wifi_spi_send_buffer，它是【阻塞】的：
 *   内部 while(length) 循环，按 4088 一包发，每包前还要 wifi_spi_wait_idle() 等模块。
 *   也就是说调一次 camera_send 会连续阻塞十几到几十毫秒。
 *   5ms 控制拍容不下这个耗时，主循环空转段虽然没有硬时限，但一次阻塞几十毫秒
 *   会让日志排空和 tick 检测全部推后，等价于人为制造漏拍。
 *
 *   所以这里自己按协议拼包，把 38400 字节拆成 WIFI_CHUNK_BYTES 一块，
 *   用状态机记住发到第几块，每次 background_poll 只推进一块就返回。
 *   协议格式是照 seekfree_assistant.c 的 camera_data_send/camera_dot_send 抄的，
 *   两处必须一致，改库版本时要回来核对这里 —— 这是自己拼包的代价，
 *   写在这里以免以后升级库之后图传变花屏却找不到原因。
 *
 * 【为什么要把帧拷出来，不直接发 scc8660_image】
 *   分块发送跨越多次 background_poll，整个过程要 100ms 量级。
 *   若期间一直 hold 住摄像头帧，视觉处理就完全拿不到新帧 —— 图传把识别饿死了。
 *   若不 hold，DMA 会在发送途中覆写缓冲，发出去的图上半部分是新帧、下半部分是旧帧，
 *   这种撕裂图用来调颜色阈值会得出完全错误的结论(比如以为板子颜色在变)。
 *   所以拷一份：memcpy 38400 字节(约 9600 个 32 位字，实测量级几十 us)，
 *   拷完立刻 release，之后慢慢发自己这份。代价是 38400 字节 DSRAM，
 *   这是为了"图传不影响识别"付的钱，值得。
 ********************************************************************************************************************/
#include "kart_wifi.h"
#include "kart_include.h"
#include <string.h>

#pragma section all "cpu0_dsram"

/*=========================== 编译期护栏 ===========================*/
/* 【本工程最重要的一条编译期检查】
 * 图传绝不能和转向绝对值编码器占同一条 SPI：那颗编码器是转向软限位和回中的
 * 唯一数据源，被图传重配 SPI 或拉动片选之后转向就失控了。
 * 库出厂默认恰好就是冲突的(SPI_4 + P22.3/P22.0/P22.1 + P23.1)，
 * 所以这道检查不是防御性编程，是防一个已经存在的坑。
 *
 * 【为什么放在 .c 用 typedef 数组，而不是 #if】
 *   预处理器不认识 SPI_2 / SPI_4 这种枚举常量，它把未定义标识符当 0 处理，
 *   写 #if (WIFI_SPI_INDEX == KART_STEER_ABS_SPI_INDEX) 会变成 0 == 0 → 恒真，
 *   永远误报。而在 .c 里枚举常量是合法的整数常量表达式，可以安全比较。 */
#if WIFI_ENABLE
typedef char wifi_spi_conflict_check[(WIFI_SPI_INDEX != KART_STEER_ABS_SPI_INDEX) ? 1 : -1];
typedef char wifi_rst_conflict_check[(WIFI_SPI_RST_PIN != KART_STEER_ABS_CS_GPIO_PIN) ? 1 : -1];
/* 图像缓冲必须装得下一整帧 */
typedef char wifi_chunk_check[((WIFI_CHUNK_BYTES > 0) && (WIFI_CHUNK_BYTES <= WIFI_SPI_TRANSFER_SIZE)) ? 1 : -1];
#endif

/*=========================== 诊断量 ===========================*/
volatile kart_wifi_link_enum g_wifi_link       = KART_WIFI_LINK_OFF;
volatile int    g_wifi_init_ret      = -1;
volatile uint32 g_wifi_img_sent      = 0;
volatile uint32 g_wifi_img_skip      = 0;
volatile uint32 g_wifi_img_us        = 0;
volatile uint32 g_wifi_chunk_max_us  = 0;
volatile uint32 g_wifi_osc_sent      = 0;
volatile uint32 g_wifi_reconnect_cnt = 0;

#if WIFI_ENABLE

#define WIFI_RAW_PER_MS            (100000u)   /* 1ms = 100000 × 10ns，与 kart_camera 同口径 */

/* 图像发送状态机 */
typedef enum
{
    WIFI_TX_IDLE = 0,       /* 空闲，可接受新请求 */
    WIFI_TX_HEADER,         /* 待发图像帧头 */
    WIFI_TX_IMAGE,          /* 正在分块发图像数据 */
    WIFI_TX_DOT_HEADER,     /* 待发叠加点帧头 */
    WIFI_TX_DOT_DATA        /* 待发叠加点坐标 */
}wifi_tx_state_enum;

static wifi_tx_state_enum tx_state = WIFI_TX_IDLE;

/* 帧拷贝缓冲。见文件头"为什么要把帧拷出来"。
 * 38400 字节，放 cpu0_dsram（本文件顶部已 #pragma section）。 */
static uint8  img_buf[SCC8660_IMAGE_SIZE];
static uint32 img_offset;              /* 已发出的图像字节数 */

/* 叠加点。坐标用 uint8：160×120 都 < 256，协议里 boundary_data_type=0 就是 8 位坐标。
 * 【坑】若以后改成 320×240，宽高超过 255，必须把 dot_type 的 bit5 置 1 并换成 uint16，
 * 否则上位机会把坐标截断，框画在完全错误的位置。 */
#define WIFI_DOT_MAX               (32)
static uint8  dot_x[WIFI_DOT_MAX];
static uint8  dot_y[WIFI_DOT_MAX];
static uint16 dot_count = 0;
static uint16 dot_sent  = 0;

/* 节拍 */
static uint32 img_period_last_raw = 0;
static uint32 osc_last_raw        = 0;
static uint32 reconnect_last_raw  = 0;
static uint32 img_start_raw       = 0;

/* 重连节制。【为何需要它—— 2026-08-11 实测出的真 bug】
 * 旧代码把 wifi_link_up() 直接放进重连分支，没意识到
 * wifi_spi_init() 是【重度阻塞】的：复位 10ms + 等模块 100ms +
 * get_version 两次 wait_idle(OTHER_TIME_OUT=1000) —— 模块不在时全走超时路径，
 * 单次约 2 秒。每 3s 重试一次 = 3 秒里主循环停转 2 秒。
 * 实测后果：CH27 overrun 在一个 3s 窗口内涨 267（600 拍漏 45%），
 * 日志环形缓冲排不出去 → VOFA 直接断流，看起来像“两个模块不能同时插”，
 * 实际与引脚无关（VOFA 走 UART_10 P13.0/P13.1，WiFi 走 P15.x+P33.5，不重叠）。
 * 【为何不能只把重连间隔改长】改成 30s 只是把漏拍稀释成每 30s 卡 2 秒，
 * 卡的那一下仍然是 400 拍——车在跑的时候这一下就足以让 PID 积分按错误 dt 累积。
 * 所以真正的修法是下面两条：① 先用【非阻塞】探针判模块在不在，
 * 不在就根本不调 init；② 只在 IDLE（车没在跑）时才允许调阻塞的 init。 */
static uint8  link_up_pending = 0;   /* 1 = 请求建链，等一个安全窗口执行 */

/* 示波器 */
#define WIFI_OSC_CH_NUM            (8)
static uint8  osc_ch_num = WIFI_OSC_CH_NUM;
static float  osc_val[SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT];

/*-------------------------------------------------------------------------------------------------------------------
 * 协议帧头。格式照 seekfree_assistant.c:99 camera_data_send 抄，两处必须一致。
 *   camera_type 字段 = (图像类型 << 5) | (无图像标志 << 4) | 边界数量
 *   length      字段 = 帧头结构体自身大小(8)，不含图像数据
 -----------------------------------------------------------------------------------------------------------------*/
static void wifi_build_img_header(seekfree_assistant_camera_struct *h, uint8 boundary_num)
{
    h->head         = SEEKFREE_ASSISTANT_SEND_HEAD;
    h->function     = SEEKFREE_ASSISTANT_CAMERA_FUNCTION;
    h->camera_type  = (uint8)((SEEKFREE_ASSISTANT_RGB565 << 5) | (0 << 4) | boundary_num);
    h->length       = (uint8)sizeof(seekfree_assistant_camera_struct);
    h->image_width  = SCC8660_W;
    h->image_height = SCC8660_H;
}

/* 叠加点帧头。格式照 seekfree_assistant.c:151 camera_dot_send 抄。
 *   dot_type = (边界类型 << 6) | (坐标位宽 << 5) | 边界数量
 *   坐标位宽 0 = 8 位（本工程 160×120 用这个），1 = 16 位 */
static void wifi_build_dot_header(seekfree_assistant_camera_dot_struct *h, uint16 num)
{
    h->head       = SEEKFREE_ASSISTANT_SEND_HEAD;
    h->function   = SEEKFREE_ASSISTANT_CAMERA_DOT_FUNCTION;
    h->dot_type   = (uint8)((XY_BOUNDARY << 6) | (0 << 5) | 1);
    h->length     = (uint8)sizeof(seekfree_assistant_camera_dot_struct);
    h->dot_num    = num;
    h->valid_flag = (uint8)(1 << 0);
    h->reserve    = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 发一块，并记录单次耗时。耗时是本模块唯一的安全指标：
 * g_wifi_chunk_max_us 超过 4000 就说明单次阻塞已经接近一个控制拍，
 * 必须把 WIFI_CHUNK_BYTES 减半。
 -----------------------------------------------------------------------------------------------------------------*/
static void wifi_send_chunk(const uint8 *buf, uint32 len)
{
    uint32 t0 = system_getval();
    uint32 dt;

    (void)wifi_spi_send_buffer(buf, len);

    /* 原始计数差分再换 us，避开 system_getval_us() "先除再差"在回绕处出错 */
    dt = (system_getval() - t0) / 100u;
    if(dt > g_wifi_chunk_max_us)
    {
        g_wifi_chunk_max_us = dt;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 模块 INT 就绪探针。【关键：本函数必须保持非阻塞】
 *
 * 【为何能用 INT 脚当探针】逐飞无线模块上电后把 INT 拉高表示“我空闲”，
 *   库里 wifi_spi_wait_idle() 等的就是这个电平。当前把 WIFI_SPI_INT_PIN 配成
 *   【浮空输入】，避免 MCU 内部下拉继续加载实测只有约 2.9 V、且会掉到 0 V 的
 *   模块 INT 输出。一次 gpio_get_level 仍只有几十个时钟周期。
 *
 * 【只能当就绪条件，不能再当可靠的在位检测】浮空脚在模块未插时电平未定义；
 *   读到 1 也不保证 SPI 能通（可能 MISO/CS 接反、SCK 没接），最终仍由 init
 *   读取版本号确认。这里只用读到 0 来避免在模块明确忙时发起新的阻塞初始化。
 -----------------------------------------------------------------------------------------------------------------*/
static uint8 wifi_module_present(void)
{
    return (uint8)(0 != gpio_get_level(WIFI_SPI_INT_PIN));
}

/*-------------------------------------------------------------------------------------------------------------------
 * 建链。分三步，每步失败都要能从诊断量看出卡在哪一步 ——
 * "连不上"这件事有三个完全不同的原因，混成一个返回码就没法查。
 *
 * 【阻塞告知】本函数单次可能阻塞 2 秒以上（init 内部超时路径），
 * 所以【只允许在 MISSION_IDLE 且 INT 探针为高时调】，调用点只有两处：
 * kart_wifi_init()（上电，pit_ms_init 之前，没有控制环可漏）和
 * kart_wifi_background_poll() 的重连分支（已加双重门禁）。
 -----------------------------------------------------------------------------------------------------------------*/
static uint8 wifi_link_up(void)
{
    /* 第一步：SPI + 模块 WiFi 连接。wifi_spi_init 内部会读版本号，
     * 读不到就是 SPI 层不通（引脚错/MISO 与 CS 接反）。 */
    g_wifi_init_ret = (int)wifi_spi_init(WIFI_SSID, WIFI_PASSWORD);

    if(0 != g_wifi_init_ret)
    {
        /* 区分 SPI 不通 和 WiFi 没连上：版本号是空串说明 SPI 都没通。 */
        if('\0' == wifi_spi_version[0])
        {
            g_wifi_link = KART_WIFI_LINK_SPI_FAIL;
        }
        else
        {
            g_wifi_link = KART_WIFI_LINK_WIFI_FAIL;
        }
        return 1;
    }

    /* 第二步：建 TCP。WIFI_SPI_AUTO_CONNECT=0，所以必须手动连。 */
    if(0 != wifi_spi_socket_connect("TCP", WIFI_TARGET_IP,
                                    WIFI_TARGET_PORT, WIFI_LOCAL_PORT))
    {
        g_wifi_link = KART_WIFI_LINK_SOCKET_FAIL;
        return 2;
    }

    /* 第三步：把逐飞助手的收发回调指到 wifi_spi。
     * 只影响示波器和参数下发；图像是本模块自己拼包直发，不走这个回调。 */
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    g_wifi_link = KART_WIFI_LINK_SOCKET_OK;
    return 0;
}

#endif  /* WIFI_ENABLE */

/*-------------------------------------------------------------------------------------------------------------------
 * 初始化
 -----------------------------------------------------------------------------------------------------------------*/
uint8 kart_wifi_init(void)
{
#if WIFI_ENABLE
    uint8 ret;
    uint32 now;

    tx_state   = WIFI_TX_IDLE;
    img_offset = 0;
    dot_count  = 0;

    memset(osc_val, 0, sizeof(osc_val));

    ret = wifi_link_up();

    now = system_getval();
    img_period_last_raw = now;
    osc_last_raw        = now;
    reconnect_last_raw  = now;

    return ret;
#else
    g_wifi_link = KART_WIFI_LINK_OFF;
    return 1;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 请求发一帧。只拷贝 + 置状态，不发送。
 -----------------------------------------------------------------------------------------------------------------*/
uint8 kart_wifi_request_image(void)
{
#if WIFI_ENABLE
    if(KART_WIFI_LINK_SOCKET_OK != g_wifi_link)
    {
        g_wifi_img_skip++;
        return 0;
    }

    /* 上一帧还没发完就丢掉这一帧，不排队。
     * 排队会让屏幕上看到的图越来越滞后，而"图和当前车况对不上"
     * 比"图少几帧"难查得多。 */
    if(WIFI_TX_IDLE != tx_state)
    {
        g_wifi_img_skip++;
        return 0;
    }

    if(0 == kart_camera_frame_ready())
    {
        g_wifi_img_skip++;
        return 0;
    }

    /* 拷一份就放帧，别占着摄像头慢慢发（见文件头）。 */
    memcpy(img_buf, (const void *)scc8660_image[0], SCC8660_IMAGE_SIZE);
    kart_camera_frame_release();

    img_offset    = 0;
    dot_sent      = 0;
    img_start_raw = system_getval();
    tx_state      = WIFI_TX_HEADER;

    return 1;
#else
    return 0;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 后台推进。只能在主循环空转段调。
 -----------------------------------------------------------------------------------------------------------------*/
void kart_wifi_background_poll(void)
{
#if WIFI_ENABLE
    uint32 now = system_getval();

    /*---------------- 断线重连（双重门禁，见下）----------------*/
    /* 【为何要两道门禁—— 2026-08-11 踩过的坑】
     * wifi_link_up() 内部的 wifi_spi_init() 单次可阻塞 2 秒（复位 110ms
     * 加两次 wait_idle 超时）。旧代码无条件每 3s 调一次，实测把主循环
     * 卡成 3 秒里停 2 秒：overrun 单个 3s 窗口涨 267，日志排不出去、
     * VOFA 断流。而且这不只是“看不到日志”：漏拍会让速度环/转向环的
     * 积分项按错误的 dt 累积，是会让车跑飞的。
     *
     * 门禁一（非阻塞探针）：模块不在位就根本不调 init。
     *   “模块没插”是最常见情况（比赛不带无线模块），一次 gpio_get_level
     *   就能判完，没必要花 2 秒去发现。
     * 门禁二（车没在跑）：只在 MISSION_IDLE 时才允许阻塞。
     *   IDLE 下电机/转向都是停机输出，卡 2 秒只是日志断一下，不会出事；
     *   一旦发车（非 IDLE）就绝对不碰 init —— 图传是调参设施，
     *   宁可上不了图传，不能为了图传漏控制拍。
     *   【副作用】跑车中断线不会自恢复，要停车回 IDLE 才重连。这是有意的。 */
    if(KART_WIFI_LINK_SOCKET_OK != g_wifi_link)
    {
        /* 周期到了就试；或者上次被门禁拦下来过（link_up_pending）且现在
         * 刚好回了 IDLE 又探到模块 —— 这种情况不要再白等剩下的 3s，
         * 立即试。否则“插上模块/停车回 IDLE”之后还要莫名地多等一拍。 */
        if(((now - reconnect_last_raw) >= (WIFI_RECONNECT_MS * WIFI_RAW_PER_MS))
           || (link_up_pending && wifi_module_present()
               && (MISSION_IDLE == kart_mission_get_mode())))
        {
            reconnect_last_raw = now;

            if(0 == wifi_module_present())
            {
                /* 模块不在位。不计入 reconnect_cnt —— 那个计数的语义是
                 * “真的尝试过建链”，把探针未过也算进去会让 CH44 在模块没插时
                 * 也响个不停，分不出“没插”和“插了但连不上”。
                 * link 保持 SPI_FAIL，CH39=1 就是“模块不在或 SPI 不通”。 */
                link_up_pending = 1;
            }
            else if(MISSION_IDLE != kart_mission_get_mode())
            {
                /* 模块在位但车在跑：记下请求，等回 IDLE 再建链。 */
                link_up_pending = 1;
            }
            else
            {
                link_up_pending = 0;
                g_wifi_reconnect_cnt++;
                (void)wifi_link_up();
            }
        }
        return;     /* 没连上就什么都别发 */
    }

    link_up_pending = 0;

    /*---------------- 图像分块推进 ----------------*/
    switch(tx_state)
    {
        case WIFI_TX_HEADER:
        {
            seekfree_assistant_camera_struct hdr;
            wifi_build_img_header(&hdr, (uint8)((dot_count > 0) ? 1 : 0));
            wifi_send_chunk((const uint8 *)&hdr, sizeof(hdr));
            tx_state = WIFI_TX_IMAGE;
            return;         /* 一次只推进一步 */
        }

        case WIFI_TX_IMAGE:
        {
            uint32 remain = (uint32)SCC8660_IMAGE_SIZE - img_offset;
            uint32 len    = (remain > WIFI_CHUNK_BYTES) ? WIFI_CHUNK_BYTES : remain;

            wifi_send_chunk(&img_buf[img_offset], len);
            img_offset += len;

            if(img_offset >= (uint32)SCC8660_IMAGE_SIZE)
            {
                tx_state = (dot_count > 0) ? WIFI_TX_DOT_HEADER : WIFI_TX_IDLE;

                if(WIFI_TX_IDLE == tx_state)
                {
                    g_wifi_img_sent++;
                    g_wifi_img_us = (system_getval() - img_start_raw) / 100u;
                }
            }
            return;
        }

        case WIFI_TX_DOT_HEADER:
        {
            seekfree_assistant_camera_dot_struct dhdr;
            wifi_build_dot_header(&dhdr, dot_count);
            wifi_send_chunk((const uint8 *)&dhdr, sizeof(dhdr));
            tx_state = WIFI_TX_DOT_DATA;
            return;
        }

        case WIFI_TX_DOT_DATA:
        {
            /* 点数据很小(最多 32 字节 × 2)，一次发完不违反耗时约束。
             * 顺序必须是先整条 x 数组、再整条 y 数组 —— 照
             * seekfree_assistant.c:166 的循环顺序，不能交错发。 */
            wifi_send_chunk(dot_x, dot_count);
            wifi_send_chunk(dot_y, dot_count);

            tx_state = WIFI_TX_IDLE;
            g_wifi_img_sent++;
            g_wifi_img_us = (system_getval() - img_start_raw) / 100u;
            return;
        }

        case WIFI_TX_IDLE:
        default:
            break;
    }

    /*---------------- 示波器 ----------------*/
    /* 放在图像之后：图像正在发的时候不插示波器包，避免把一帧图像的字节流
     * 切断(上位机按长度收，中间插别的包会直接解析错乱)。
     * 代价是发图那 100ms 里波形有个空档，可接受。 */
    if((now - osc_last_raw) >= (WIFI_OSC_PERIOD_MS * WIFI_RAW_PER_MS))
    {
        uint8 i;

        osc_last_raw = now;

        for(i = 0; i < osc_ch_num; i++)
        {
            seekfree_assistant_oscilloscope_data.data[i] = osc_val[i];
        }
        seekfree_assistant_oscilloscope_data.channel_num = osc_ch_num;

        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
        g_wifi_osc_sent++;
    }

    /*---------------- 上位机下发参数 ----------------*/
    /* 让上位机能在线改阈值。每次只解析已收到的字节，不阻塞。 */
    seekfree_assistant_data_analysis();
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 示波器通道写值。纯赋值，5ms 拍里可以安全调。
 -----------------------------------------------------------------------------------------------------------------*/
void kart_wifi_osc_set(uint8 ch, float value)
{
#if WIFI_ENABLE
    if(ch < SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT)
    {
        osc_val[ch] = value;
    }
#else
    (void)ch; (void)value;
#endif
}

void kart_wifi_osc_set_channel_num(uint8 num)
{
#if WIFI_ENABLE
    if((num > 0) && (num <= SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT))
    {
        osc_ch_num = num;
    }
#else
    (void)num;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 设置叠加点。内部拷贝成 uint8，不存指针 ——
 * 存指针的话调用方的局部数组一出作用域就野了，而发送要跨多次 poll。
 -----------------------------------------------------------------------------------------------------------------*/
void kart_wifi_set_boundary(uint16 dot_num, uint16 *x, uint16 *y)
{
#if WIFI_ENABLE
    uint16 i;

    if((NULL == x) || (NULL == y) || (0 == dot_num))
    {
        dot_count = 0;
        return;
    }

    if(dot_num > WIFI_DOT_MAX)
    {
        dot_num = WIFI_DOT_MAX;
    }

    for(i = 0; i < dot_num; i++)
    {
        /* 钳到 8 位。超范围的点画在边上比画到对面去要好判断。 */
        dot_x[i] = (uint8)((x[i] > 255u) ? 255u : x[i]);
        dot_y[i] = (uint8)((y[i] > 255u) ? 255u : y[i]);
    }

    dot_count = dot_num;
#else
    (void)dot_num; (void)x; (void)y;
#endif
}

kart_wifi_link_enum kart_wifi_link_state(void)
{
    return g_wifi_link;
}

#pragma section all restore
