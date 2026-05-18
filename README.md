# Custom-controller

Custom-controller 是基于 STM32H723VGTx 的 6 关节自定义控制器固件。工程由 STM32CubeMX 生成外设初始化代码，使用 STM32 HAL 驱动和 FreeRTOS CMSIS V1 任务调度，通过 FDCAN、UART、SPI 和 TIM 外设完成电机控制、关节角度映射、状态上报、按键输入、WS2812 指示灯和蜂鸣器提示。

## 工程信息

| 项目 | 内容 |
| --- | --- |
| 主控 | STM32H723VGTx，LQFP100，Cortex-M7 |
| CubeMX 配置 | `CtrlBoard-H7_WS1812.ioc` |
| Keil 工程 | `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx` |
| 目标工具链 | MDK-ARM V5.27 |
| 输出文件名 | `CtrlBoard-H7_WS1812` |
| 固件产物 | `MDK-ARM/CtrlBoard-H7_WS1812/CtrlBoard-H7_WS1812.hex` |

## 功能

- 6 关节数据映射：`MoterMap.j0` 到 `MoterMap.j5` 保存控制器输出关节角，单位为弧度。
- ZDT 电机控制：通过 ZDT X42 V2.0 协议读取位置并发送力矩模式命令。
- MIT 电机控制：通过 `CAN_cmd_MIT()` 发送位置、速度、刚度、阻尼和前馈力矩命令。
- 重力补偿：`gravity_compensation()` 根据 6 关节角度计算关节力矩前馈。
- 关节阻抗控制：阻抗控制表示“位置误差和速度误差 → 力矩命令”的计算过程，入口位于 `joint_impedance.c`。
- 运动学计算：正运动学、逆运动学和工具坐标系相对位姿变换位于 `kinematics.c`。
- 状态上报：`CustomController_StructSend()` 以 `0xA5` 帧头、`0x0302` 命令码、CRC8 和 CRC16 组织数据帧。
- 人机反馈：按键处理位于 `userkey.c`，WS2812 指示灯位于 `ws2812.c`，蜂鸣器提示位于 `Safewarning.c`。

## 目录

| 路径 | 说明 |
| --- | --- |
| `Core/Inc`、`Core/Src` | CubeMX 生成的主程序、外设初始化、中断和 FreeRTOS 入口 |
| `Drivers` | STM32 HAL 和 CMSIS 驱动 |
| `Middlewares/Third_Party/FreeRTOS` | FreeRTOS 内核与 CMSIS RTOS V1 适配层 |
| `User/APP` | 控制器应用逻辑、关节映射、运动学、阻抗控制、重力补偿、CRC、按键和提示逻辑 |
| `User/BSP` | FDCAN 板级封装 |
| `User/Devices` | WS2812 和 ZDT X42 V2.0 设备驱动 |
| `MDK-ARM` | Keil 工程、调试配置和编译输出 |

## 外设配置

| 外设 | 用途 | 关键配置 |
| --- | --- | --- |
| FDCAN1 | 关节 4、5、6 的 CAN 通信 | 标称 1 Mbps，PD0 RX，PD1 TX |
| FDCAN2 | 关节 1、2、3 的 CAN 通信 | 标称 1 Mbps，PB5 RX，PB6 TX |
| UART7 | 控制器数据帧发送和舵机数据接收 | 115200 bps，PE7 RX，PE8 TX，DMA |
| USART1 | JustFloat 调试数据发送 | 921600 bps，PA10 RX，PA9 TX，DMA |
| SPI6 | WS2812 数据输出 | 主机发送，6 Mbit/s，PA5 SCK，PA7 MOSI |
| TIM6 | 1 ms 系统周期任务 | 任务频率统计、按键扫描、蜂鸣器状态推进 |
| TIM12 CH2 | 蜂鸣器 PWM | PB15 输出 |

## FreeRTOS 任务

| 任务 | 优先级 | 栈大小 | 周期 | 作用 |
| --- | --- | --- | --- | --- |
| `LED_Task` | Normal | 128 | 约 1 ms | WS2812 刷新和按键蜂鸣器处理 |
| `SERVO_TASK` | High | 1024 | 3 ms | 读取关节位置、更新关节角映射、计算重力补偿 |
| `ROBOT_TASK` | High | 1024 | 10 ms | 计算控制命令并发送电机力矩命令 |
| `MAPDATASEND_TAS` | Normal | 128 | 35 ms | 发送自定义控制器状态帧 |

## 控制流程

启动流程：`main()` 初始化 GPIO、DMA、SPI6、USART1、UART7、FDCAN1、FDCAN2、TIM6、TIM12 → `bsp_can_init()` 启动 CAN 通信 → `Beep_Init()` 初始化蜂鸣器 → UART DMA 接收开启 → TIM6 中断开启 → FreeRTOS 调度器启动。

运行流程：`SERVO_TASK` 读取 ZDT 位置并更新 `theta` → `gravity_compensation()` 计算 `tau` → `ROBOT_TASK` 计算工具位姿、关节速度、阻抗输出和发送力矩 → `MAPDATASEND_TAS` 周期发送上位机状态帧。

## 控制模式

当前控制模式由 `User/APP/servo_mapping.h` 中的 `CONTROLLER_MODE` 定义，现配置为 `CONTROLLER_MODE_ZDT`。`CONTROLLER_MODE_ZDT` 表示关节位置来自 ZDT 电机反馈，控制输出通过 FDCAN 发送到电机驱动器；`CONTROLLER_MODE_SERVO` 表示关节位置来自串口舵机反馈，控制输出通过自定义帧上报。

## 数据帧

`customController_t` 是发送给上位机的数据帧结构，完整长度为 39 字节。

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `SOF` | 1 | 帧头，固定为 `0xA5` |
| `DataLength` | 2 | 数据段长度，等于 `sizeof(moterMapHeader)` |
| `Seq` | 1 | 序号 |
| `CRC8` | 1 | 帧头校验 |
| `CmdID` | 2 | 命令码，当前为 `0x0302` |
| `moterMap` | 30 | 在线状态、气泵状态和 6 个关节角 |
| `FrameTail` | 2 | CRC16 校验 |

## 构建与下载

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx`。
2. 选择目标 `CtrlBoard-H7_WS1812` 并执行 Build。
3. 通过 Keil Download 或调试器烧录生成的 `CtrlBoard-H7_WS1812.hex`。

## 修改入口

| 目标 | 文件 |
| --- | --- |
| 修改引脚、时钟、外设参数 | `CtrlBoard-H7_WS1812.ioc` |
| 修改 FreeRTOS 任务 | `Core/Src/freertos.c` |
| 修改关节映射和控制主逻辑 | `User/APP/servo_mapping.c` |
| 修改 ZDT 电机协议 | `User/Devices/ZDT/ZDT_X42_V2.c` |
| 修改 CAN 发送接收封装 | `User/BSP/bsp_fdcan.c` |
| 修改运动学模型 | `User/APP/kinematics.c` |
| 修改重力补偿模型 | `User/APP/Gravity_comp.c` |
| 修改阻抗控制参数和计算 | `User/APP/joint_impedance.c` |
| 修改状态帧校验 | `User/APP/crc.c` |

## 注意事项

- `CONTROLLER_MODE`、关节限位、力矩映射系数和阻抗参数位于 `User/APP/servo_mapping.h` 和 `User/APP/servo_mapping.c`。
- 当前 `only_gravity` 定义为 `1`，`ROBOT_TASK` 运行纯重力补偿路径。
- `TIM6` 中断承担 1 ms 级周期维护逻辑，修改中断周期会影响任务频率统计、按键消抖和蜂鸣器节拍。
- `MDK-ARM/CtrlBoard-H7_WS1812` 目录包含编译产物，重新构建会更新 `.hex`、`.axf`、`.map`、`.o`、`.crf` 等文件。
