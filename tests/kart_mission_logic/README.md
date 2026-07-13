# 比赛状态机主机测试

本目录使用最小类型桩测试 `Kart_TC387/user/kart_mission.c`，不依赖 TC387、IPS200、电机或传感器。

在仓库根目录执行：

```sh
cc -std=c99 -Wall -Wextra -Werror \
  -I tests/kart_mission_logic \
  -I Kart_TC387/user \
  Kart_TC387/user/kart_mission.c \
  tests/kart_mission_logic/test_kart_mission.c \
  -o /tmp/kart_mission_logic_test && \
  /tmp/kart_mission_logic_test
```

覆盖内容：底层未就绪禁止启动、科目一顺序与非法跳转、科目二 8 条任务配额及去/返门洞方向校验、科目三记录/返回、运行中丢失就绪位自动故障停机。

测试通过只说明高层流程正确，不代表路径规划、语音、蜂鸣器、转向闭环、底盘动作或实车安全链路已经完成。
