# 速度环安全逻辑主机测试

本目录用最小逐飞类型桩测试 `kart_control.c` 与 `kart_pid.c` 的硬件无关行为。

```sh
cc -std=c99 -Wall -Wextra -Werror \
  -I tests/kart_control_logic \
  -I Kart_TC387/user \
  Kart_TC387/user/kart_pid.c \
  Kart_TC387/user/kart_control.c \
  tests/kart_control_logic/test_kart_control.c \
  -lm -o /tmp/kart_control_logic_test && \
  /tmp/kart_control_logic_test
```

当前覆盖默认关闭、关闭态持续输出零、启用后产生控制输出、`e0` 对应的关闭接口立即清零后轮请求，以及 PID 参数的临界区快照接口。测试不代表 PWM、编码器或电机实物已经验证。
