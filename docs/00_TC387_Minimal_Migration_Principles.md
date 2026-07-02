# TC387 最小迁移原则

当前口径：

- 直接在 `G:\CODE\Smart car\SmartCar\6.6_V1_TC264` 里写。
- 后续按 TC387 硬件编译/适配。
- TopSpeed 是主体，能照搬就照搬。
- 不重建架构，不乱删，不大重构。
- 只改硬件差异：引脚、有刷驱动、编码器、前轮绝对值编码器、必要安全信号。
- 优先保留 `Power_now`、`power_sync()`、`Mission`、`Subject`、PID、IMU/GPS、Menu、Diary、Flash、Route。

下一步只做最小代码准备：

1. 在 `6.6_V1_TC264` 里确认当前工程能编译。
2. 梳理 TopSpeed 必须搬的文件清单。
3. 先搬结构，不改算法。
4. 先接 pin map。
5. 再把动力输出底层改成三路有刷 `DIR + PWM`。
6. 再接左右编码器和前轮绝对值编码器。
7. 空载验证方向，再低速跑。

旧文档 `01~05` 只作参考。以后主线以本文和 `03_Kart_Migration_Plan.md` 的最小迁移计划为准。
