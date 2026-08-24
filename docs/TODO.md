# 整理清单

哈尔滨理工大学 北极星队 · 第二十一届全国大学生智能汽车竞赛 卡丁快跑组 · 全国二等奖。
完赛之后的收尾工作。
目标：把这一年的东西收拾干净，客观记下做过什么。
自己以后翻回来能看懂、能想起来，别人想看也看得明白。

## 起点

- `6067bb3` / tag `race-final-2026` —— 实车跑出全国二等奖的那一份，一个字没动。
  任何时候想回到"当时就是这么跑的"，`git switch --detach race-final-2026`。
- 这个仓库原本是队友的（`RyanChenJH/SmartCar`，private）。2026-08-24 起工作仓库换成
  自己的 `Yusei-YK/Polaris-Kart-TC387`，remote 名 `mine`；队友的 `origin` 留着但不再推。
  暂时设为 private——全历史里有队友 14 条提交，翻公开之前先跟他说一声。

## 几条规矩

- 本机没有编译器，只做源码级检查。每批改完由人在 AURIX Studio 里编一次。
  永远不写"编译通过"。
- 改文件走 python 脚本（`C:/tmp/pNN_*.py`），每个锚点断言只命中一次，
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
      点名的 `build_log.txt`/`build_full.txt` 合成一条 `Kart_TC387/build_*.txt`。
      没加 `*.txt` 这种大网——`code/本文件夹作用.txt` 是已入库的说明文件，会被吞掉。
- [x] A2 确认 `release build/` 和 `.metadata` 未被跟踪 —— 已确认，`.gitignore` 已覆盖
- [x] A3 `.gitattributes` 已建。故意不写 `* text=auto`、也不给厂商目录任何规则——
      没有 text 属性就等于 git 完全不做转换，第三方代码原样进原样出。
      规则一律按路径点名，只管 `Kart_TC387/code` 和 `Kart_TC387/user` 下的 `.c`/`.h`；
      第三方 `code/Kart_TPL` 再单独锁回 `eol=lf`（那整个目录本来就是纯 LF，零改动）。
- [x] A4 行尾已规范化。实测自己的代码是 82 个文件 26849 行：42 个纯 LF、
      21 个纯 CRLF、19 个同一份里混着两种。全刷成纯 CRLF，动了 61 个文件、
      21176 个换行。第三方 `code/Kart_TPL` 的 12 个文件 10640 行没碰。
      git 那边一个字节没变：本机 `core.autocrlf=true`，库里一直存的就是 LF，
      61 个文件按属性过滤后的 blob 哈希和索引里的完全一致。
      以后脚本锚点用 CRLF 形式；`kart_params.c` 那条"只有 1 处 CRLF、
      绝不许整文件归一化"的限制到此作废。

## B 目录

- [ ] B1 删散落文件。五个草稿都是 8 月 10 到 16 号的东西，真文件全比它们新，
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
- [ ] B2 删重复的调试配置：`Kart_TC387 Debug (1).launch`、`Debug（1）.launch`、
      `Debug（2）.launch`，只留 `Kart_TC387 Debug.launch` 和 release build 那个
- [ ] B3 `SCC8660_Product-master/SCC8660_Product-master/` 移出编译工程
      （厂商例程 + PDF + png，不该在这里），`docs/` 留一条说明
- [ ] B4 外层 `SmartCar_归档`、`tmp`、`.reference_audit` 处置
- [ ] B5 未跟踪副本 `Kart_TC387 2` 删掉
- [ ] B6 历史里有个 52.67 MB 的 Infineon TC3xx 用户手册 PDF
      （`TC387_Library-master/【文档】说明书 芯片手册等/核心板文档/`），超了 GitHub
      50 MB 的建议线。从历史里摘掉要重写提交，先不动，记在这里。

## C 文档

- [ ] C1 README 重写。客观记录：这车是什么、四个科目怎么跑的、
      一路上遇到过什么、最后拿了什么成绩。给自己，也给后来的人。
- [ ] C2 `docs/devlog.md`：把这几个月补回来，git log 是现成的素材
- [ ] C3 标定手册：`kart_calib.h` 里每个数是怎么测出来的。
      已知要写进去的：Ackermann 常数实测 1410（注释里写的 1480 是错的）、
      抓地悬崖 1.8 m/s、后轮 `v = 0.00046*duty - 0.10`、
      编码器 0.00036816 m/脉冲、满舵半径 1.32 m、后轮轮距 0.60 m、
      转向电机 1800 counts/s、静摩擦死区约 63 counts
- [ ] C4 `docs/` 现有的 `会话交接主文档.md`、`倒车提速排查存档.md`、
      `horn_timing_issue_resolution.md` 归位或合并进 devlog
- [x] C5 队伍身份：哈尔滨理工大学 北极星队 / Team Polaris (HRBUST) /
      项目代号 Polaris Kart / 组别卡丁快跑组。
      英文缩写用 HRBUST，不要用 HUST——那个是华中科技大学的。
- [ ] C5b 成员名单，写 README 时补
- [ ] C6 参考来源写清楚：这套方案是照着东北大学秦皇岛 TopSpeed 的开源学的。
      README 里把"跟着他们的思路做的部分"和"我们自己搞出来的部分"分开写，
      别让后来人误会成全是原创，也算给人家一个交代。

## D 代码

按 include 层级过，不按文件名：
`Kart_Config` → `Kart_Driver` → `Kart_App` → `Kart_Algo` → `Kart_Decision`
→ `Kart_Debug` → `user`。这样改后面的注释时，前面的事实已经核实过。

`user/` 里 10 个文件 2445 行也要过：`cpu0_main.c`、`cpu1/2/3_main.c`、
`kart_multicore.c/h`、`isr.c/h`、`isr_config.h`、`cpu0_main.h`。

- [ ] D1 注释标准样板：先做 `kart_calib.h`（173 行，纯常量+注释，
      改坏了不影响逻辑，本身又是最该留给后人的东西），认可了再铺开
- [ ] D2 按上面顺序过完 94 个文件
- [ ] D3 订正已知的过期注释：
      - Ackermann 1480 → 1410（多处）
      - `kart_playback.h:171` Ke 量程写的 ±800，实际 ±3000
      - `kart_playback.h` 里"Ke 从 100 往上加"的建议，在 Kh 由 Ke 派生之后
        已经是误导（`kart_playback.c:826`）
      - `board_pins.h` 关于点阵屏 SYNC "已 exti_disable" 那段已被证伪，
        真相在 `cpu0_main.c` 的 `WIFI_ENABLE` 块里
      - `kart_follow.h` 有一条陈旧注释
      - `kart_vtrack.c:332` 还在讲 `pyr_gray` 怎么建两帧，这个变量已经没了
      - `kart_params.h` 里 `PB RevScl` 的说明没提它现在也管科目三倒车段
- [ ] D4 死代码清理，先列清单再删：
      - `KART_DOT_ROW0_TEST` / `KART_DOT_ROWS_TEST` / `DOT_ALLON_TEST`
        三个自检死循环，硬件早就确认了
      - `KART_USE_SCHEDULER=0` 的旧主循环分支，还留着做 A/B 对照吗
      - `kart_bench.c` 253 行，还用不用
      - `kart_menu.c` 2602 行是最大的单文件，值得单独看一遍
- [ ] D5 函数名清单：只挑词不达意的，给出对照表，人工确认后再改

## E 收尾

- [x] E1 新建自己的仓库——已完成 2026-08-24。`Yusei-YK/Polaris-Kart-TC387`，private，
      5 个分支 + 4 个 tag 全推上去，作者信息原样保留。
      `gh` 用的是免安装 zip，解压在 `G:/CODE/gh/bin/gh.exe`，没进 C 盘、没进 PATH。
      老 main（`c818976`，多核重构前快照）先打了 tag `snapshot-pre-multicore-20260722`
      再快进到 `6067bb3`；队友 main 上那 5 条分叉提交留在
      `archive/teammate-main-20260722`。
- [ ] E1b 决定要不要翻成 public（会连带公开队友的 14 条提交，先问他）
- [ ] E2 从干净 clone 验证一次能编译过
- [ ] E3 打 tag，写 CHANGELOG

## 已经关掉的线索，不要重开

- 固定动作/盲盒段只占 5 到 9 秒、4 到 6 米，挖不出时间。
- "让引导员出库先走 8 到 10 米直线"物理上做不到，一号锥桶紧贴发车区。
- 实时性没问题：overrun 恒为 0，最坏分发耗时 1064 到 1076 us，窗口 5000 us。
- 提高 `Flw Cruise` 不会让跟随段变快，瓶颈是引导员走多快
  （实测中位 1.21 到 1.55 m/s，低于当时 1.70 的设定）。
