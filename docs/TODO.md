# 整理清单

哈尔滨理工大学 北极星队 · 第二十一届全国大学生智能汽车竞赛 卡丁快跑组 · 全国二等奖。
完赛之后的收尾工作。
目标：把这一年的东西收拾干净，客观记下做过什么。
自己以后翻回来能看懂、能想起来，别人想看也看得明白。

## 起点

- `6067bb3` / tag `race-final-2026` —— 实车跑出全国二等奖的那一份，一个字没动。
  任何时候想回到"当时就是这么跑的"，`git switch --detach race-final-2026`。
- 这个仓库原本是队友的（`RyanChenJH/SmartCar`，private）。2026-08-24 起工作仓库换成
  自己的 `Yusei-YK/Polaris-Kart-TC387`，remote 名 `mine`。
  2026-08-25 把队友的 `origin` remote 移掉了，不再往那边推——项目已经跟他分开。
  为了留存来源，原地址记在这里：`https://github.com/RyanChenJH/SmartCar.git`。
  暂时设为 private——全历史里有队友 14 条提交，翻公开之前先跟他说一声。

## 分支怎么摆

一条分支 `main`，就这一条。要留住某个时刻用标签，不要新开分支——
分支是"还在往下写的线"，标签是"钉住不动的点"，混着用两三个月后就没人认得出谁是谁。

| 标签 | 指向 | 是什么 |
|---|---|---|
| `race-final-2026` | `6067bb3` | 实车跑出全国二等奖的那一份 |
| `flash-s3-20260823` | `a383fe2` | 科目三最后一次烧进车里的版本 |
| `freeze-20260726-light-voice` | `7c23740` | 灯 + 语音做完时的冻结点 |
| `snapshot-pre-multicore-20260722` | `c818976` | 视觉搬去 core3 之前的老 main |
| `archive/teammate-main-20260722` | `a5513dc` | 队友那条线的最后快照，5 条提交 |

2026-08-25 删掉的四条分支，一条提交都没丢：`flash/s3-20260823` 和 `main` 同一个
commit；`freeze/light-voice-20260726` 和上表那个 tag 同一个 commit；
`cleanup/dead-code-20260726`（`86291d1`）本来就在 `main` 历史里；
`archive/teammate-main-20260722` 转成了标签。

## 几条规矩

- 本机没有编译器，只做源码级检查。每批改完由人在 AURIX Studio 里编一次。
  永远不写"编译通过"。
- 改文件走 python 脚本（`G:/CODE/claude-tmp/pNN_*.py`），每个锚点断言只命中一次，
  写完检查字节数、行数、CR 数、注释配对、括号平衡、`#if`/`#endif` 配对。
- 不碰：SeekFree / Infineon 库、TLD7002 驱动（9463 行第三方代码）、
  `SCC8660_Product-master`。
- 一批 2 到 4 个文件，同一批里绝不混"只改注释"和"删代码"——
  编译失败时原因必须唯一。
- 行尾：git 里存 LF，工作区 CRLF，由 `.gitattributes` 锁死。
- `kart_` 前缀不动。C 没有命名空间，前缀就是命名空间，而且仓库里同时挂着
  四套第三方代码，去掉更容易撞名。只改词不达意的函数名。
  另外 `kart_` 就是卡丁车，跟组别对得上，不是随手起的，README 里提一句。

## 注释怎么写

现在的问题是话太多、太像 AI 写的，还有一些只有当时在场的人才看得懂。规矩：

1. 只写"为什么"，不写"是什么"。代码本身能说明的事不要重复一遍。
2. 不写口语化铺垫。"我们来看一下"、"值得注意的是"这类全删。
3. 一条注释要能被没参与过的人看懂。"那次那个问题"要么展开说清楚，要么删掉。
4. 单条注释超过 3 行，考虑是不是该搬进 `docs/`，代码里只留一句加指路。
5. 记录"踩过的坑"是有价值的，但要写成结论："X 不能放在 Y 之后，因为 Z"，
   而不是把当时的排查过程复述一遍。

## A 立规矩

- [x] A1 `.gitignore` 已补。`*.launch`、`menudiff.txt`、两个 build 日志原本就在里面，
      真缺的是脚本改代码掉下来的渣：`*.new` `*.bak` `*.orig` `*.rej` `*.patch` `*.tmp`。
      点名的 `build_log.txt`/`build_full.txt` 合成一条 `build_*.txt`。
      没加 `*.txt` 这种大网——`code/本文件夹作用.txt` 是已入库的说明文件，会被吞掉。
- [x] A2 确认 `release build/` 和 `.metadata` 未被跟踪 —— 已确认，`.gitignore` 已覆盖
- [x] A3 `.gitattributes` 已建。故意不写 `* text=auto`、也不给厂商目录任何规则——
      没有 text 属性就等于 git 完全不做转换，第三方代码原样进原样出。
      规则一律按路径点名，只管 `code` 和 `user` 下的 `.c`/`.h`；
      第三方 `code/Kart_TPL` 再单独锁回 `eol=lf`（那整个目录本来就是纯 LF，零改动）。
- [x] A4 行尾已规范化。实测自己的代码是 82 个文件 26849 行：42 个纯 LF、
      21 个纯 CRLF、19 个同一份里混着两种。全刷成纯 CRLF，动了 61 个文件、
      21176 个换行。第三方 `code/Kart_TPL` 的 12 个文件 10640 行没碰。
      git 那边一个字节没变：本机 `core.autocrlf=true`，库里一直存的就是 LF，
      61 个文件按属性过滤后的 blob 哈希和索引里的完全一致。
      以后脚本锚点用 CRLF 形式；`kart_params.c` 那条"只有 1 处 CRLF、
      绝不许整文件归一化"的限制到此作废。

## B 目录

- [x] B1 已删。五个草稿都是 8 月 10 到 16 号的东西，真文件全比它们新，
      逐个核实过内容已落地或已作废，删了不丢信息：
      - `kart_bench.c.new` —— 0 字节空文件
      - `bench_fill.txt` —— B9/B11/B12 三个 bench 用例怎么填的施工笔记，
        `kart_bench.c` 里 156/170/178 行三个 case 都实现了
      - `kart_mission_preprocess_patch.txt` —— 那段调用现在在
        `user/kart_multicore.c:655`（视觉搬到 core3 之后挪过去了）
      - `cpu0_main.c.patch` —— 已应用，摄像头与点阵屏的调用顺序硬约束
        就在 `cpu0_main.c:285`
      - `kart_vtrack_fix.txt` —— 金字塔缓冲超配诊断（分配 38400 字节、
        实际只需 24000），`pyr_gray` 这个变量随视觉重构已经不存在了
      另外还有 `build_full.txt`、`build_log.txt`、`menudiff.txt` 三个日志。
      2026-08-25 删完：8 个文件 6826 字节，删前确认过一个都没被 git 跟踪，
      删后工作区没有任何文件变成 deleted 状态。
- [x] B2 已删。根目录现在只有 `Kart_TC387 Debug.launch` 和
      `Kart_TC387 release build.launch` 两份配置，带括号的副本一个不剩。
- [x] B3 那个凌瞳产品包目录早就不在了（git 没跟踪过，盘上也没有），
      欠的那条 `docs/` 说明已经补在 `docs/README.md` 末尾：它是什么、
      为什么移出编译工程、在用的驱动和包装层在哪、说明书留在哪。
- [x] B4 已处置。仓库的同级目录现在只剩 `.claude`、`CLAUDE.md` 和仓库本身，
      归档、tmp、审计目录都没了。
- [x] B5 已删。同名带后缀的未跟踪副本目录一个都不存在了。
- [ ] B6 历史里有个 52.67 MB 的 Infineon TC3xx 用户手册 PDF
      （`TC387_Library-master/【文档】说明书 芯片手册等/核心板文档/`），超了 GitHub
      50 MB 的建议线。从历史里摘掉要重写提交，先不动，记在这里。

## C 文档

- [x] C1 README 重写。这车是什么、四个科目怎么跑的、四核怎么分工、
      二次开发要注意什么，都写进去了。成绩和名次还欠着（见 C2）。
- [ ] C2 开发日志补全。日志已提到根目录 `开发日志.md`，按阶段记到 2026-08-04。
      正文里还留着四处【待补】：各科目跑成什么样与最终名次、旧灯板还有哪些设计
      错误、`KART_MULTICORE_COMPAT_ENABLE` 当年留 0 的真实原因、赛后那段是为
      决赛准备还是补作业。git log 是现成的素材。
      标签 `archive/teammate-main-20260722` 上那份不用再对：它覆盖的九个日期
      （2026-06-29 ~ 07-13）主线全都有，是同一批事的旧稿。
- [ ] C3 标定手册：`kart_calib.h` 里每个数是怎么测出来的。
      已知要写进去的：抓地悬崖 1.8 m/s、后轮 `v = 0.00046*duty - 0.10`、
      编码器 0.00036816 m/脉冲、后轮轮距 0.60 m、转向电机 1800 counts/s、
      静摩擦死区约 63 counts。
      Ackermann 常数按代码事实是 1480（`kart_calib.h:35`，全仓库 15 处，1410 零处），
      `:32-34` 那五组实测均值 1477.5 也对得上。但那五组是旧齿轮时代量的，
      2026-07-30 换齿轮 / 转向编码器 / 转向电机之后只重标了第四节，第一节没重测。
      满舵半径同理：文件里是 1480/1064 = 1.39 m。要动这两个数得先重测。
- [x] C4 那两份散在 `docs/` 的存档已归位，没有并进日志——日志是按日期走的流水，
      这两份是参考件。`会话交接主文档.md` → `06_整车状态存档_2026-07-26.md`，正文
      未动，文件头加横幅列出已被推翻的六条（通道数 33→43、模式枚举、CPU1/2/3 分工、
      语音已放开、点阵屏已修好、视觉是主力）。`倒车提速排查存档.md` →
      `07_倒车提速排查.md`，纯改名。`docs/README.md` 重写成完整索引。
- [x] C5 队伍身份：哈尔滨理工大学 北极星队 / Team Polaris (HRBUST) /
      项目代号 Polaris Kart / 组别卡丁快跑组。
      英文缩写用 HRBUST，不要用 HUST——那个是华中科技大学的。
- [ ] C5b 成员名单，写 README 时补
- [x] C6 参考来源写清楚。README 加了「参考与自研」一节，跟着 TopSpeed 学的
      （分层架构、控制节拍、PID 算子、统一 Power 出口、Mission 状态机）和自己做的
      （转向角度环、里程推算、轨迹复刻、视觉与 CPU3 异步、灯光扫描适配、人车联动、
      在线调参、跟随、遥测）分开列。致谢里补了 TLD7002 点阵例程的上游地址、
      当年固定的提交 `335abb7` 和 GPL-3.0，子模块没恢复。
      标签上那份 `docs/05_官方例程与第三方来源.md` 没有搬进来：它说 TLD7002 底层
      还没移植、没上板，而日志 07-25 灯板已恢复、07-26 实车验证通过，是过期稿。
- [x] C7 捞回那份单元测试。`tests/kart_light_logic/` 已在库里，是整个仓库唯一
      一份能脱离硬件跑的测试。捞之前核对过接口：测试用到的 24 个 `kart_light_*`
      符号在当前代码里一个不缺。编译命令改指 `code/Kart_App/`，这条命令没有在本机
      跑过。被作者自己 Revert 掉的另外两套没捞。

## D 代码

按 include 层级过，不按文件名：
`Kart_Config` → `Kart_Driver` → `Kart_App` → `Kart_Algo` → `Kart_Decision`
→ `Kart_Debug` → `user`。这样改后面的注释时，前面的事实已经核实过。

`user/` 里 10 个文件 2445 行也要过：`cpu0_main.c`、`cpu1/2/3_main.c`、
`kart_multicore.c/h`、`isr.c/h`、`isr_config.h`、`cpu0_main.h`。

- [x] D1 注释标准样板：`kart_calib.h` 不用重写，它本身就是标准。四条特征是
      后面 93 个文件要照抄的：每个数带标定来源和日期、派生量标明"不要手改"、
      重标步骤编号列出、踩过的坑留在原地（例如 `:137` 记着 2026-07-29 误改
      符号导致一动就打死不回中，已回退）。文件里六处算术全复算过，对得上。
- [x] D2 按上面顺序过完了。七层加 `user/` 加 `code/kart_include.h` 全走到。
      找到的错注释分五类：行号引用跑了、结论的前提已被别处改掉、
      说法从来就没对过、把编译期已死的分支当成出厂路径写、
      索引悄悄落在它声称镜像的目录后面。
      改法是【错了就删干净】：不保留错的说法，也不留"原来写的是 X、订正为 Y"
      这类交代，留下来的每一句都是现值直陈。引用一律改成函数名/宏名/槽名
      这类不随行号漂的锚点。
      只改注释，去注释后代码逐字节等价由脚本强制。
- [x] D3 订正已知的过期注释，五条都处理完了：`kart_playback.h` 的 Ke 建议
      （改成直接填实车完赛那对 Ke=-600，并写明当前剖面看不到 e_lat）、
      `kart_vtrack.c:332` 的 `pyr_gray`（实际是 `pyr_L0`/`pyr_L1`）、
      `kart_params.c:71` 的 `PB RevScl`（补上它也管科目三那段开环倒车）、
      `kart_follow.h` 近距联锁那三层叠着的阈值推导（1.35/1.15 → 0.70/0.85 →
      现值 1.67/1.36）压成一层现值，FRAMES 与 SLOW_MIN_RATIO 两段也按 1.10 的
      巡航速度重算。`board_pins.h` 那条本来就不用改，`:269-278` 已经是订正后的说法。
      顺带查实 `kart_follow` 是活代码（`kart_mission.c:538` `:648` 无条件调），
      `FOLLOW_ENABLE` 是死宏，已记进 D4。
- [x] D4 死代码清理，删了两处、留了两处：
      - 删 `user/cpu0_main.c` 的旧主循环回退（`KART_USE_SCHEDULER` 的 `#else`
        分支连宏和两处 `#if`/`#endif` 一起）和三个点阵屏自检死循环，
        577 → 475 行。调度器 overrun 恒 0，回退没有留的理由；而且旧循环里那份
        模式显示副本还挂着科目四改名前的编号，本身就是过期注释源。
        活的那份在 `kart_dot_show_mode()`，车上看到的 444 没变。
      - 删 `kart_follow.h` 的死宏 `FOLLOW_ENABLE`：全仓库零读取点，
        `kart_mission.c:538` `:648` 一直是无条件调，跟随从来就是活代码。
      - 留 `kart_bench.c`/`.h`（389 行）：B1-B12 一项都没实测过，但这 12 项的
        划分有参考价值。文件头已写清"未实现、不参与运行"，以及真要开先处理
        38400 字节假图与 cpu0 DSRAM 那 240K 的争抢。
      - 留 `kart_hw_test.c`/`.h`：开关常 0，但换板时是现成的落地工具，
        文件头已标状态。
      `kart_menu.c` 2602 行不是死代码，是活的菜单，挪去 D2 按层级过。
      删宏之后 `board_pins.h` 和 `kart_debug_uart.c:577` 两处悬空引用已改说法。
- [ ] D5 函数名清单：对照表列在下面，等人工确认后再改。改名要连
      调用点一起动，所以本条只给判断，不动代码。
      扫了 `code/`（不含 `zf_*` 和 `TLD7002_driver`）加 `user/` 的所有 `.c`，
      537 个自研函数的名字逐个看过，真正词不达意的六条，
      另有五条属于命名不统一而不是名字错，分开列。

      建议改的六条：
      - `kart_calib_init/update/reset/get_stat/print_report`
        （`kart_vision_calibrate.c`）→ 加 vision，例如 `kart_vision_calib_*`。
        这五个是视觉颜色/焦距标定工具，但 `kart_calib_` 这个前缀在别处
        指的是 `kart_calib.h`——整车标定常数那份（阿克曼常数、编码器系数）。
        在调用点看见 `kart_calib_update()` 的人会翻错文件。这条最值得改。
      - `power_check_poll` / `power_check_is_done`（`kart_power.c`）→
        `power_boot_selftest_poll` / `_is_done`。名字里的 check 读起来是只读的
        状态查询，实际上它按 Debug_Stage 1→10→2→20→3→30→4→40→5 的顺序
        真的给后轮和转向发 duty，车会动。当状态查询调一下就是轮子转起来。
      - `IMU_check`（`kart_imu.c`）→ `kart_imu_calib_gyro_bias`。它不做检查，
        做的是陀螺零偏静止标定（带极差静止检测和重试），而且阻塞约 6s。
        名字来自 TopSpeed 的同名函数，改了就断掉迁移对照，要改得在注释里留一句。
      - `draw_begin/clear/string/line/point/image/commit`
        （`kart_multicore.c`，对外声明在 `kart_multicore.h`）→ 建议加 `kart_` 前缀。
        两个问题：一是全仓库就这七个对外函数没有 `kart_` 前缀；二是出厂档
        `DRAW_ON_CORE2=1` 时它们根本不画，只是往 `draw_q` 塞指令等 CPU2 执行，
        而 `draw_begin()` 返回 0 时整帧被静默丢掉（`draw_q.dropped++`）。
        名字说“画”，行为是“排队，也可能不画”。
      - `kart_playback_complete`（`kart_playback.c`）→ `kart_playback_finish`。
        名字像个判断（“完成了吗”），实际是动作。而且开环倒车档它并不完成——
        置 `playback_braking=1` 后保持 `playback_running=1`，真正完成要等
        `kart_playback_poll_brake()` 那边。
      - `draw_straight_line`（`kart_calc.c`）→ `line_from_two_points` 之类。
        它不画任何东西，是用两点算 ax+by+c=0 的系数。注释里已经写明，
        但名字还在误导。它和 `point_to_straight_line_distance` /
        `get_point_to_line_dir` 全工程零调用者，改不改都不影响运行。

      属于命名不统一，不建议现在动：
      - `kart_power.c` 的公开接口全是 `power_*` 没有 `kart_` 前缀
        （`power_init`/`power_sync`/`power_stop` 等十一个），而同文件的静态辅助
        反而叫 `kart_limit_duty`/`kart_set_dir_pwm`/`kart_slew_step`。
        前缀正好用反了。改要动十一个符号加全部调用点。
      - 一批 static 辅助用了完整的公开前缀，例如 `kart_vision_erode_once`、
        `kart_vtrack_bearing`、`kart_playback_find_nearest`、`kart_steer_wrap180`。
        看名字分不出哪个是对外接口。量太大，收益太小。
      - `kart_calc.c` 从 TopSpeed 的 GPS.c 搬来那批没有前缀：`get_angle`、
        `get_distance`、`get_relative_angle`、`invSqrt`、`quaternionToEuler`。
        其中 `get_angle` 最含糊——它返回的是方位角（度，正北 0，逆时针为正），
        不是任意“角度”。
      - `power_sync` 名字没说方向，它做的是把 `Power_now` 的四个 duty 快照取出来、
        过变化率限制、写 PWM，叫 `power_apply` 更准。
      - `kart_odom_read_center_distance` 返回的是累计里程，不是这一拍走的距离，
        `read`+`distance` 容易读成增量。

      核实过没问题、不要改的（防下一届当成漏项）：
      - `kart_horn_isr` 真的是中断里调的（`isr.c:97`）。
      - `kart_menu_input_poll` / `kart_menu_enc_poll` 名副其实。
      - `vision_dev_cell` 的 dev 是色相偏差，和文件顶部 VIS_DEV 那套词汇一致。
      - `kart_light_publish_text` 的 publish 指双缓冲切帧，是准确的。
      - `kart_calib_print_report` 真的在 printf。
      - `kart_task_light_10ms` 和 `kart_task_10ms` 都在 10ms 拍，不是重名。
      - `motion_goto_axis_s` 的 s 是轴线上的带符号投影，注释里定义过。

## E 收尾

- [x] E1 新建自己的仓库——已完成 2026-08-24。`Yusei-YK/Polaris-Kart-TC387`，private，
      作者信息原样保留。2026-08-25 收成 1 条分支 + 5 个标签（见上面"分支怎么摆"）。
      `gh` 用的是免安装 zip，解压在 `G:/CODE/gh/bin/gh.exe`，没进 C 盘、没进 PATH。
      老 main（`c818976`，多核重构前快照）先打了 tag `snapshot-pre-multicore-20260722`
      再快进到 `6067bb3`；队友 main 上那 5 条分叉提交钉在标签
      `archive/teammate-main-20260722`（原来是分支，08-25 转成标签）。
      那 5 条提交里的灯板和 TLD7002 驱动 `main` 里都有、而且更新——只是目录重排过，
      从平铺的 `user/` 挪进了 `code/Kart_App`、`code/Kart_TPL` 等。
      真正只在那条线上的是 8 个文件，处置见 C6 和 C7。
- [ ] E1b 决定要不要翻成 public（会连带公开队友的 14 条提交，先问他）
- [ ] E2 从干净 clone 验证一次能编译过
- [ ] E3 打 tag，写 CHANGELOG。CHANGELOG 已经写完（仓库根 `CHANGELOG.md`，
      `b961551`），按阶段记不按语义化版本，因为只发布过一份固件。
      开头第一件事就是告诉接手的人检出 `race-final-2026`，别用 `main`、
      别用当前分支尖端。里面引用的 29 个提交号、5 个标签、两处分支落差
      和注释批的 54 条，全由 `p90_changelog.py` 现算校验，数字漂了写不出来。
      剩下的 tag 等 E2 从干净 clone 编译验证过再打——现在打等于给一份
      没编译过的代码盖章。

## 已经关掉的线索，不要重开

- 固定动作/盲盒段只占 5 到 9 秒、4 到 6 米，挖不出时间。
- "让引导员出库先走 8 到 10 米直线"物理上做不到，一号锥桶紧贴发车区。
- 实时性没问题：overrun 恒为 0，最坏分发耗时 1064 到 1076 us，窗口 5000 us。
- 提高 `Flw Cruise` 不会让跟随段变快，瓶颈是引导员走多快
  （实测中位 1.21 到 1.55 m/s，低于当时 1.70 的设定）。
