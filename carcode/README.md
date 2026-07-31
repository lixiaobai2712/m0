# MSPM0G3507 8路循迹小车

这是一个可独立转发和导入 Code Composer Studio 的完整工程源码目录。

## 环境

- MCU: MSPM0G3507, LQFP-64
- 开发环境: Code Composer Studio Theia
- SDK: MSPM0 SDK 2.11 或兼容版本
- 编译器: TI Arm Clang

## CCS 导入

1. 打开 CCS。
2. 选择 `File -> Import Project(s)`。
3. 选择本目录中的 `CCS/car_mspm0_refactored.projectspec`。
4. 完成导入后执行 Build。

SysConfig 会自动生成 `ti_msp_dl_config.c/.h`、链接脚本和设备选项。
目录中的 `SysConfig/Generated` 同时保留了一份已经验证过的生成文件，便于查看和手工构建。

## 接线

| 功能 | MSPM0G3507 |
|---|---|
| 寻迹 X1-X8 | PA0-PA7，从左到右 |
| TB6612 PWMA/PWMB | PA12/PA13 |
| TB6612 AIN1/AIN2 | PB17/PB19 |
| TB6612 BIN1/BIN2 | PA16/PB24 |
| 启动按键 | PB21，低电平有效 |
| TB6612 STBY | 扩展板固定接 +5V |

代码默认黑线输出低电平。参数统一位于 `Config/car_config.h`。

## 目录

- `App`: 程序入口和应用调度
- `BSP`: 基础板级服务
- `Config`: 小车速度、PID 和控制周期参数
- `Control`: 循迹控制算法
- `Drivers`: 按键、传感器和电机驱动
- `SysConfig`: 外设配置及生成文件
- `CCS`: CCS 工程导入描述

首次烧录请架空车轮，确认左右电机正方向后再落地测试。
