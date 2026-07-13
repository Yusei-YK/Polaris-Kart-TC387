# 官方例程目录

本目录集中保存项目开发中需要对照的官方例程。第三方仓库优先以 Git 子模块引入，避免把上游完整历史直接复制进 SmartCar，也便于固定和更新参考版本。

## 当前例程

| 路径 | 上游 | 固定提交 | 用途 | 许可证 |
|---|---|---|---|---|
| `TLD7002_LED_Dot_Matrix/` | `https://gitee.com/seekfree/TLD7002_LED_Dot_Matrix.git` | `335abb7` | TLD7002、7x15 LED 点阵和 TC387 接口参考 | GPL-3.0 |

## 获取方式

首次克隆 SmartCar 时执行：

```bash
git clone --recurse-submodules https://github.com/RyanChenJH/SmartCar.git
```

已经克隆主仓库时执行：

```bash
git submodule update --init --recursive
```

## 使用约束

- 子模块保留上游原貌，不在其中直接开发 SmartCar 功能。
- 迁移前先核对目标 MCU、引脚、时钟、中断、延时和许可证。
- SmartCar 自有适配代码仍放在 `Kart_TC387/user/`，并记录参考来源和实际修改。
- 引入例程不代表功能已经接入、编译或通过硬件验证。
