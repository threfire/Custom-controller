# Copilot 使用指南（针对此仓库）

目的：帮助 AI 编码代理快速理解此 STM32 + Keil 项目的结构、常见模式、构建与调试流程，以及修改代码时应遵循的要点。

- **大致架构**：基于 STM32H7 系列 MCU（见 `startup_stm32h723xx.s` / `startup_stm32h723xx.lst`），使用 STM32Cube HAL 驱动（`Drivers/STM32H7xx_HAL_Driver`）和 FreeRTOS（`Core/Src/freertos.c`、`FreeRTOSConfig.h`）。应用代码分层如下：
  - `Core/Inc`：共享头文件与 HAL 配置（如 `stm32h7xx_hal_conf.h`）。
  - `Core/Src`：应用源文件（`main.c`、各外设实现如 `fdcan.c`、`spi.c` 等）。
  - `Drivers/`：CMSIS 与 HAL 驱动代码。
  - `User/APP`, `User/BSP`, `User/Devices`：板级/设备驱动与应用逻辑（扩展点）。

- **关键文件与模式（示例）**：
  - 初始化模式：CubeMX 风格的 `MX_<PERIPHERAL>_Init()` 系列，例如 `MX_GPIO_Init()`、`MX_SPI6_Init()`。参见 `Core/Src/main.c` 中的调用顺序和 HAL DMA/UART 初始化示例。
  - 中断/回调：HAL 回调（`HAL_UARTEx_RxEventCallback` / `HAL_TIM_PeriodElapsedCallback` 等）在 `main.c` 中实现。中断和回调修改需小心遵循现有任务频率统计与 FreeRTOS 上下文规则。
  - 外设文件命名：`<peripheral>.c/.h`（例如 `fdcan.c`/`fdcan.h`、`usart.c`/`usart.h`），修改时同时更新对应 header。

- **构建与 Flash/调试 流程（可复现）**：
  - 首选：使用 Keil uVision 打开项目 `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx`，在 IDE 中编译、下载到板子并调试。输出产物位于 `MDK-ARM/CtrlBoard-H7_WS1812/`（例如 `.axf`）。
  - 常见替代：使用 Segger J-Link 工具（Keil 内或外部）对 `.axf`/`.elf` 文件烧录与调试。仓库包含 `JLinkSettings.ini`，表明常用 J-Link 配置。
  - 示例（需根据本机工具调整）：
    - 在 Keil 中：打开 `.uvprojx` → Build → Flash
    - 使用 J-Link（示例）：`JLink.exe -device STM32H723 -if SWD -speed 4000 -CommanderScript flash.jlink`（请根据实际 J-Link 工具和设备名调整）。

- **项目约定与注意事项（只说明可在代码中发现的规则）**：
  - 不直接在中断回调中做繁重工作；当前模式使用回调把数据放缓冲并通过任务处理（参见 UART DMA 与 `HAL_UARTEx_ReceiveToIdle_DMA` 的用法）。
  - CubeMX 自动生成区与用户区：保留 `/* USER CODE BEGIN */` / `/* USER CODE END */` 区块内的自定义逻辑，避免覆盖自动生成代码。
  - 时钟与功率设置通过 `SystemClock_Config()` 管理，修改要注意 PLL / 分频器对外设时钟的影响（见 `main.c`）。

- **集成点与外部依赖**：
  - HAL（Drivers/STM32H7xx_HAL_Driver）和 CMSIS（Drivers/CMSIS）是基础依赖；任何外设改动通常在 `Core/Inc` 与 `Core/Src` 两端配对修改。
  - FreeRTOS（`Core/Src/freertos.c`、`RTE/` 下构建产物）、以及可能的第三方中间件在 `Middlewares/Third_Party`。

- **当你需要改动某个外设（建议工作流程）**：
  1. 在 `Core/Inc` / `Core/Src` 查找对应外设文件（如 `fdcan.c` / `fdcan.h`）。
  2. 在 `main.c` 中找到 `MX_<PERIPHERAL>_Init()` 的调用点，确认初始化顺序不会破坏已有 DMA / UART 链路。
  3. 如果改动会影响中断或 DMA 回调，确保回调处理逻辑仍然是“尽快退出并通知任务/队列”，不要阻塞。
  4. 在 Keil 中对 `.uvprojx` 编译并使用板子或 J-Link 测试。

- **调试贴士**：
  - 查看 `MDK-ARM/*/*.d` 和 `.crf` 文件可以帮助理解当前编译器/链接器参数（Keil 构建日志也在 `MDK-ARM/*/build_log.htm`）。
  - 若遇到硬件相关崩溃，优先检查 `SystemClock_Config()`、外设时钟使能、NVIC 配置与中断优先级。

如果这份指南中有遗漏的特殊流程（如 CI、特殊脚本、或其他工程约定），请指出我会把它们合并进来。
