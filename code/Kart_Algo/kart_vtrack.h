/*********************************************************************************************************************
 * 文件名称  kart_vtrack
 * 所属分层  Kart_Algo。算法本身是纯数学,但本文件第 72 行 include 了
 *           zf_common_headfile.h,kart_vtrack.c 又用 #pragma section all
 *           "cpu0_dsram" 给金字塔定段,两条都是 AURIX/TASKING 专有 ——
 *           要在 PC 上单测得先补类型垫片、去掉 pragma,不能直接编。
 * 功能说明  视觉跟踪：多特征 LK 光流 + forward-backward 一致性 + 中值 scale → 方位角 + 距离等级
 *
 * 【为什么必须有它：Detector 设定可靠性上限，Tracker 决定实际帧率】
 *   kart_vision（颜色分割 Detector）只给方位角 + 绝对测距，后者已在 kart_follow.h
 *   注释证明对身体挂载的柔性板结构上不可行（侧身 60° 时投影宽缩到 0.5 倍，
 *   测距把 1.5m 报成 3m）。改成纯 Detector 逐帧重检也不行：
 *       1) 重检代价高（全图扫描 + 连通域 + 门限判定），30FPS 全跑必然挤占控制拍；
 *       2) 颜色本身不稳定（曝光/阴影/遮挡/同色干扰），连续好帧率做不到 100%，
 *          丢一帧就让速度/转向指令跳变，车会抖。
 *   所以架构上必须:Detector 低频建锚(如 3Hz),Tracker 高频(30Hz)维护特征点集,
 *   Detector 只设可靠性天花板(颜色不对根本认不出来)。
 *
 * 【当前接入状态:这套 LK Tracker 最终没有上车】
 *   kart_vtrack_init() 确实在 user/cpu0_main.c:295 调了,但
 *   kart_vtrack_update() 全工程只有 kart_bench.c 两个调用点(B9/B12),两处
 *   都包在 #if BENCH_ENABLE 里,而 BENCH_ENABLE=0,且
 *   kart_bench_run_one/_all 自己也零调用者 —— 下面这套 LK 一拍都没真跑过。
 *   真正上车的是【本文件定义的 kart_vtrack_result_t 这个数据结构】:
 *     - 科目三:kart_mission.c 的 s3_vision_to_vtrack() 拿颜色 Detector 的
 *       bearing + dist_m 合成一个 result;
 *     - 人体跟随:kart_person_link_vtrack() 由 PLINK 链路合成。
 *   两条都直接喂 kart_follow_update()。也就是说 kart_follow 那三档死区的
 *   语义是照本文件设计的,但产生这些量的不是下面的 LK/FB/MAD。
 *
 * 【为什么是 LK 而不是模板匹配/直方图反投影/Kalman 预测】
 *   模板匹配对旋转/缩放/光照变化全不鲁棒，板子一侧身模板就对不上；
 *   直方图反投影需要目标和背景颜色差异大，黄板 vs 黄色地面/警示物必然混；
 *   Kalman 预测【不是观测】，它只能平滑已有观测，自己不产生位置，丢了就是丢了。
 *   LK 给的是【像素级的帧间位移】，不依赖全局模板，只要局部纹理够就能跟 ——
 *   这正好契合"身上挂的纸/布有褶皱、有图案、有黑边"这个事实。
 *
 * 【为什么要 forward-backward 一致性】
 *   LK 只优化当前帧的光度误差，不知道点有没有被遮挡、有没有漂移到背景上。
 *   遮挡 1 帧后再出现，特征点已经不在原位，但 LK 仍然会给一个"收敛"的结果
 *   （因为背景也有纹理），这就是经典的 drift-on-background。
 *   Forward-Backward 测法：p0 → LK前向 → p1 → LK后向 → p0'，算 ||p0 - p0'||。
 *   若点真的对应同一个物理位置，往返误差应 < 1px；若点已经漂移/被遮挡，
 *   往返误差会跳到几个、十几个像素 —— 这是 LK 自己不给的置信度指标。
 *
 * 【为什么要 MAD（中值绝对偏差）剔除异常点】
 *   即使有 FB 一致性，仍然会有小部分点落在重复纹理（如衣服条纹）或镜面反光上，
 *   LK 收敛、FB 误差也不大，但位移向量和大部队方向不一致。这些点参与 scale 计算
 *   会把中值拉偏。MAD 剔除：算所有位移的中值方向，然后剔除角度偏离 > 30° 的点。
 *
 * 【为什么纵向只给 TOO_NEAR/NORMAL/TOO_FAR 三档，不做连续测距】
 *   见 kart_follow.h:75 的【重要】注释：单目宽度测距要求目标刚性、正对、完整可见，
 *   身上挂的板三条全不满足。改用【中值尺度比】：
 *       scale_r = median( d_ij(t) / d_ij(t-1) )  over surviving pairs
 *   d_ij 是点 i、j 之间的欧氏距离。scale_r 只告诉"这一帧比上一帧大了还是小了"，
 *   配合起跟时锚定的参考宽度，给出三档：太近(加速了或人突然靠近)、正常、太远。
 *   优点：不需要 f_px 标定，不需要量板子实宽，板子形变时只会误判成"远了一点"
 *   而不是报出 3m 这种错得离谱的绝对值。代价：车不知道自己离人几米，
 *   只知道"比起跟的时候更近还是更远" —— 本任务只要求稳定跟住，这个信息量够。
 *
 * 【为什么横向（方位角）仍然可以用质心】
 *   质心来自所有存活点的 x 坐标中值（而非均值，抗outlier），再除以 f_px 得角度。
 *   只要点集大部分仍在目标上（FB + MAD 保证），质心就可靠。方位角不依赖板宽，
 *   侧身时横向投影按 cos 缩，但质心的横向位置不变 —— 这是它比宽度测距鲁棒的地方。
 *
 * 【输出】
 *   - bearing_rad：方位角（弧度），质心导出，对侧身/遮挡鲁棒
 *   - scale_level：NEAR/NORMAL/FAR 三档，喂给 kart_follow 的距离死区逻辑
 *   - confidence：综合置信度 0-255，由以下加权：存活点数占比、FB 中值误差、
 *                 |scale_r - 1|（形变过大说明目标在剧烈旋转，不可信）、
 *                 距上次 Detector 锚定的帧数（太久没校准，drift 风险上升）
 *   - valid：本帧有效标志。无效时 kart_follow 走丢失减速逻辑。
 *
 * 【Detector 与 Reacquisition 的分工】
 *   本模块不含 Detector 实现 —— Detector 就是 kart_vision(颜色分割),
 *   已经存在并验证过。本模块留了 kart_vtrack_reacquire(vis) 接受 Detector
 *   的锚定:从 bbox 里均匀撒点、提特征、建参考尺度。
 *   【但这个函数全工程零调用者】"低频调用、Detector 设可靠性上限"是设计
 *   意图,不是已发生的事实。
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-08-10        Kart                首版：LK + FB + MAD + 中值 scale，未接硬件
 ********************************************************************************************************************/

#ifndef KART_VTRACK_H_
#define KART_VTRACK_H_
#include "zf_common_headfile.h"
#include "kart_vision.h"

/* 总开关。【注意】除了下面这两行自己,全工程没有任何代码读这个宏,
 * kart_vtrack.c 里也没有 #if VTRACK_ENABLE —— 改成 1 不会多编出任何东西。
 * 真正决定 LK 跑不跑的是 kart_bench.h 的 BENCH_ENABLE(当前 0)。 */
#ifndef VTRACK_ENABLE
#define VTRACK_ENABLE              (0)
#endif

/*=========================== 图像金字塔 ===========================*/
/* LK 在多尺度金字塔上迭代，粗尺度给大位移的初值、细尺度精修。
 * 160×120 下推荐 2 层（原图 + 1/2 下采样）。再多一层收益递减且内存翻倍。 */
#define VTRACK_PYRAMID_LEVELS      (2)

/* 金字塔图像宽高。0 层 = 原图，1 层 = 1/2 下采样 = 80×60。
 * 【内存占用】两层灰度图：(160×120 + 80×60) = 24000 字节，放 cpu0_dsram。 */
#define VTRACK_L0_W                (160)
#define VTRACK_L0_H                (120)
#define VTRACK_L1_W                (80)
#define VTRACK_L1_H                (60)

/*=========================== 特征点管理 ===========================*/
/* 最大跟踪点数。越多越鲁棒，但线性增加计算量。
 * 取 32:每点双向 LK,单方向最坏 VTRACK_PYRAMID_LEVELS(2) ×
 * VTRACK_LK_MAX_ITER(20) = 40 次迭代,双向 80 次;每次迭代扫 11x11 = 121 个
 * 窗口点,每点 6 次双线性插值(I_prev / I_curr / ±x / ±y 梯度)。
 * 存活率按 70% 估,稳态约 22 点在跟。
 * 【耗时从未实测】原来这里按"视觉跑在 CPU0 的 10ms 拍里"估了个 4ms;
 * 2026-08-15 视觉整条流水线搬到 core3(见 kart_preprocess.h),那个预算前提
 * 已经不成立,而 B9 基准从没跑过,所以板上真实耗时至今未知。 */
#define VTRACK_MAX_POINTS          (32)

/* Reacquire 时在 bbox 内撒点的策略。均匀网格，行列数各 N_SIDE。
 * 6×6 = 36 个候选，提完特征、Shi-Tomasi 评分后取前 MAX_POINTS 个。
 * 【为什么不直接用 Detector 的质心 + 四角】
 *   1) 只有 5 个点，丢 2 个就不够算 scale；
 *   2) 柔性板/衣服没有固定的"角"，质心也会因遮挡偏移；
 *   3) 均匀撒点能覆盖内部纹理（褶皱、图案、黑边），比只守四角鲁棒。 */
#define VTRACK_REACQ_GRID_SIDE     (6)

/* Shi-Tomasi 角点响应阈值。低于此值的候选点不要：没纹理，LK 收敛不了。
 * 阈值取法：160×120 灰度图，3×3 窗口，梯度平方和量级 10^2 - 10^4，
 * 特征值差量级 10^1 - 10^3。取 20.0：普通边缘能过，纯平坦区被拦。
 * 现场若存活点数长期 < 10，降到 10.0；若误跟背景，升到 50.0。 */
#define VTRACK_MIN_CORNER_RESPONSE (20.0f)

/*=========================== LK 光流参数 ===========================*/
/* LK 窗口半径（像素）。窗口大小 = (2*r+1)×(2*r+1)。
 * 5 → 11×11 = 121 像素参与优化，对小幅旋转/光照变化有一定容忍。
 * 再大计算量平方增长；再小窗口覆盖不了足够纹理，噪声敏感。 */
#define VTRACK_LK_WIN_RADIUS       (5)

/* LK 迭代收敛判据：位移增量(dx² + dy²) < EPS² 时停止。单位像素。
 * 0.01 → 子像素精度 0.1px。按 kart_vision.h 现在的 VISION_FPX(83.5f)算,
 * 角度精度 ~0.07°(原文写的 f_px=211 对不上,数已订正,结论不变),
 * 足够。再小迭代次数暴涨，收益递减。 */
#define VTRACK_LK_EPS              (0.01f)

/* LK 单层最大迭代次数。防止不收敛时死循环。
 * 正常 3-8 次迭代就收敛；取 20 是兜底，达到说明窗口已经滑到无纹理区。 */
#define VTRACK_LK_MAX_ITER         (20)

/* 金字塔层间位移缩放因子。上一层（粗尺度）的解乘以此数作为下一层初值。
 * 2.0：标准值，因为每层图像尺寸是上一层的 2 倍。 */
#define VTRACK_PYR_SCALE           (2.0f)

/*=========================== Forward-Backward 一致性 ===========================*/
/* FB 往返误差阈值（像素）。p0 → forward → p1 → backward → p0'，
 * 若 ||p0 - p0'|| > FB_THRESH，判定为漂移/遮挡，剔除此点。
 * 1.0：子像素精度下仍可能有 0.5px 的数值误差，留一倍余量。
 * 真实遮挡/漂移的往返误差通常 > 5px，1.0 能可靠分辨。 */
#define VTRACK_FB_THRESH           (1.0f)

/*=========================== MAD 异常点剔除 ===========================*/
/* 位移向量与中值方向夹角超过此值（度）时剔除。
 * 所有点跟同一个刚体，理论上位移方向应完全一致（只有平移时），
 * 或呈辐射状（有旋转/接近时）。落在重复纹理/反光的点方向会偏离。
 * 30°：对小幅旋转（如人转身 < 30°/帧）仍能容忍，同时把 90° 偏的误跟拦住。 */
#define VTRACK_MAD_ANGLE_DEG       (30.0f)

/*=========================== 中值 scale 与距离等级 ===========================*/
/* 存活点对之间欧氏距离的中值比 scale_r = median(d_ij(t) / d_ij(t-1))。
 * scale_r > 1 表示点集在扩张（目标靠近或板子展开），< 1 表示收缩（远离或折叠）。
 *
 * 三档阈值：
 *   scale_r > NEAR_THRESH        → TOO_NEAR  加速了或人突然靠近，该减速/后退
 *   FAR_THRESH < scale_r < NEAR  → NORMAL    死区内，保持当前速度
 *   scale_r < FAR_THRESH         → TOO_FAR   人走远了，该加速追
 *
 * 阈值取法：人正常步速 1.2 m/s，车跟随速度上限 1.8 m/s，跟车距离目标 1.5m。
 * 1 帧 33ms 时，人横向最大相对位移 = (1.8 + 1.2) × 0.033 ≈ 0.1m，
 * 对 1.5m 距离 → 尺度变化 ΔS ≈ 0.1/1.5 ≈ 6.7%。死区宽度取 ±10% 留余量。 */
#define VTRACK_SCALE_NEAR_THRESH   (1.10f)
#define VTRACK_SCALE_FAR_THRESH    (0.90f)

/*=========================== 置信度 ===========================*/
/* 存活点数下限（个）。低于此值判 valid=0，走丢失逻辑。
 * 取 8：scale 需要至少 3 对点（6 个）才有意义，留一倍余量。 */
#define VTRACK_MIN_ALIVE_POINTS    (8)

/* Detector 重锚的最大间隔（帧）。超过此帧数未重锚，强制 valid=0。
 * LK 是纯帧间，误差累积无界；Detector 是绝对观测，校准漂移。
 * 不能无限依赖 LK，必须定期回 Detector 锚定。
 * 取 100 帧 ≈ 3.3s @ 30FPS：这段时间若一直无法 Detector（颜色全不对），
 * 说明已经彻底跟丢，该停车了。 */
#define VTRACK_MAX_FRAMES_SINCE_DET (100)

/* 置信度衰减系数（每帧）。无 Detector 锚定时 confidence 按此系数衰减。
 * 0.995^30 ≈ 0.86，即 1 秒衰减到 86%；0.995^100 ≈ 0.61。
 * 配合 Detector 周期（如 3Hz = 10 帧一次）：Detector 周期内衰减不超过 5%，
 * 可接受；长期无 Detector 时加速衰减，逼停车。 */
#define VTRACK_CONF_DECAY_PER_FRAME (0.995f)

/*=========================== 输出 ===========================*/
typedef enum
{
    KART_VTRACK_SCALE_TOO_NEAR = -1,    /* 目标太近，该减速或停止 */
    KART_VTRACK_SCALE_NORMAL   =  0,    /* 死区内，保持当前速度 */
    KART_VTRACK_SCALE_TOO_FAR  =  1     /* 目标太远，该加速追 */
}kart_vtrack_scale_level_enum;

typedef struct
{
    uint8  valid;                       /* 1 = 本帧跟踪有效 */
    float  bearing_rad;                 /* 方位角（弧度），质心导出，>0 = 右侧 */
    kart_vtrack_scale_level_enum scale_level; /* 距离等级 */
    float  scale_r;                     /* 原始 scale 比值，调试用 */
    uint8  confidence;                  /* 综合置信度 0-255 */
    uint16 alive_count;                 /* 本帧存活点数 */
    uint16 total_count;                 /* 总管理点数（含未激活） */
    float  fb_error_median;             /* FB 往返误差中值（px），调试用 */
    uint16 frames_since_detector;       /* 距上次 Detector 锚定的帧数 */
}kart_vtrack_result_t;

/*======================================== 对外接口 ========================================*/

/* 初始化。清空点集、帧计数、置信度。
 * 【不分配任何内存】金字塔 pyr_L0/pyr_L1 是 kart_vtrack.c 的 static 数组,
 * 由该文件顶部的 #pragma section all "cpu0_dsram" 在链接期定段;本函数既不
 * 申请也不清零金字塔(第一帧 build_pyramid 会整幅写满)。
 * 本函数不碰摄像头,与 kart_camera_init() 没有先后依赖;唯一的约束是要在
 * 第一次 kart_vtrack_update() 之前调。当前在 user/cpu0_main.c:295。 */
void kart_vtrack_init (void);

/* 单帧更新。img 指向当前帧 RGB565(w×h,行优先),w/h 必须正好 VTRACK_L0_W/H,
 * 否则把快照置无效后原样返回。全工程唯一的调用点是 kart_bench.c 的 B9/B12,
 * 喂的是 fake_img_rgb565 假图,并不是 kart_vision_process 那一路真帧。
 * 内部：RGB→灰度 → 建金字塔 → 对所有存活点做双向 LK → FB 一致性检查 →
 *       MAD 剔除 → 中值 scale → 质心方位角 → 更新 confidence。
 * 纯计算、不阻塞、不改全局状态（金字塔和点集是本模块私有）。
 * 返回值同时写入内部快照，可用 kart_vtrack_get() 再取。
 * 【耗时未实测】4-5ms @ 300MHz 是 2026-08-10 的纸面估算,B9 基准没跑过。
 * valid=1 的门槛有两条,上面没写全:存活点数 >= VTRACK_MIN_ALIVE_POINTS(8)
 * 且 confidence >= 50(kart_vtrack.c 里的字面常数,没有对应的宏)。 */
const kart_vtrack_result_t *kart_vtrack_update (const uint16 *img, int16 w, int16 h);

/* 取最近一次的输出快照 */
const kart_vtrack_result_t *kart_vtrack_get (void);

/* Reacquisition：接受 Detector（kart_vision）的锚定结果，在 bbox 内重新撒点。
 * vis 传 kart_vision_get() 的结果；若 vis->valid=0 则什么都不做。
 * 调用时机：
 * 调用时机(以下是设计意图;本函数当前零调用者,三条都没有真正发生):
 *   1) 首次起跟:kart_follow 进入 TRACKING 状态时调一次,建立初始点集;
 *   2) 置信度过低:confidence 低于阈值(如 100)时低频调(如 3Hz),重新锚定;
 *   3) 完全丢失后重捕获:valid=0 持续一段时间、然后 Detector 又认到了。
 * 【不要每帧都调】:Reacquisition 会清空当前点集重来,丢掉帧间连续性,
 * 只在必要时用它校准 LK 的累积漂移。 */
void kart_vtrack_reacquire (const kart_vision_result_t *vis);

/* 复位。清空点集、重置 confidence、帧计数归零。
 * 设计上在跟随任务切换时调用,避免用到上一次跑车的残留点;实际零调用者。 */
void kart_vtrack_reset (void);

#endif
