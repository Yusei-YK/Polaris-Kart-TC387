# 灯板逻辑主机测试

本目录使用最小类型桩在电脑上测试 `code/Kart_App/kart_light.c`，不依赖 TC387、TLD7002 或 ADS。

在仓库根目录执行：

```sh
cc -std=c99 -Wall -Wextra -Werror \
  -I tests/kart_light_logic \
  -I code/Kart_App \
  code/Kart_App/kart_light.c \
  tests/kart_light_logic/test_kart_light.c \
  -o /tmp/kart_light_logic_test && \
  /tmp/kart_light_logic_test
```

测试覆盖初始化、亮度限幅、非法命令、全部静态图案、左右转向、双闪、雨刷、跨周期更新时间和完整帧复制。测试通过只代表硬件无关的显示逻辑正确，不代表 TLD7002 底层、SYNC 扫描、飞线方向或实物显示已经验证。

2026-08-25 从标签 `archive/teammate-main-20260722` 捞回。核对过接口：测试用到的
24 个 `kart_light_*` / `KART_LIGHT_*` 符号在当前代码里全都还在。编译命令的路径
已按目录上提后的位置改过，但这条命令没有在本机跑过。
