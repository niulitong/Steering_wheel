# SteeringWheel_Button 方向盘按钮板固件

BIT FSAE 方向盘按钮板固件，与方向盘显示屏配套使用。板上读取旋钮、电位器和两个按键，把驾驶指令以 CAN 报文发出：

- 旋钮（4 位）选择驾驶模式，档位变化时发送 0x310 DriveMode_Request。
- 电位器设定滑移率等级，共 8 级（0～7），数值变化时发送 0x700。
- KEY1 触发录像/相机切换（0x701），KEY2 触发消错（0x702）。

微控制器为 STM32F103C8T6，裸机轮询，无 RTOS。CAN 位率 500 kbit/s。

## 硬件

| 项目 | 参数 |
| --- | --- |
| MCU | STM32F103C8T6（Cortex-M3，64 KB Flash，20 KB RAM） |
| 系统时钟 | HSE 8 MHz × 9 = 72 MHz，APB1 = 36 MHz |
| CAN | bxCAN1，500 kbit/s（36 MHz ÷ 4 分频 ÷ 18 tq） |
| 调试口 | SWD（PA13/PA14） |

### 引脚分配

引脚定义在 `Core/Inc/main.h`，源头是 CubeMX 工程 `SteeringWheel_Button.ioc`。输入信号均为低有效、内部上拉。

| 引脚 | 标号 | 方向 | 功能 |
| --- | --- | --- | --- |
| PA1 | ADC1_IN1 | 输入 | 电位器，0～3.3 V，量化为滑移等级 0..7 |
| PA4 | LED1 | 输出 | CAN 事件指示，事件发送时拉低 100 ms 点亮 |
| PA5 | LED2 | 输出 | 心跳灯，每 500 ms 翻转（1 Hz） |
| PA6、PA7、PB0、PB1 | S1、S2、S3、S4 | 输入 | 4 位旋钮 |
| PA11、PA12 | CAN_RX、CAN_TX | — | CAN1 收发器接口 |
| PB12 | KEY1 | 输入 | 录像/相机切换 |
| PB13 | KEY2 | 输入 | 消错 |
| PB2、PB11、PB14、PB15 | S5、KEY3、KEY4 | 输入 | 预留，固件未使用 |

## CAN 接口

### 发送

CAN ID 定义在 `Core/Inc/button_panel.h`。滑移等级和驾驶模式采用事件驱动：数值变化才发送，上电后各强制发送一帧当前值。0x701 和 0x702 只在按键释放沿单次发送。

| ID | 帧名 | DLC | 触发条件 | 载荷 |
| --- | --- | --- | --- | --- |
| 0x310 | DriveMode_Request | 8 | 旋钮档位变化（发送后屏蔽 200 ms） | byte0 = '0'..'3'（48..51，对应模式 S/Q/C/E），档位无效时 0xFF |
| 0x700 | SlipLevel | 1 | 滑移等级变化 | byte0 = 0..7 |
| 0x701 | RecorderToggle | 1 | KEY1 释放沿 | bit0 = 1 |
| 0x702 | ErrorClear | 1 | KEY2 释放沿 | bit0 = 1 |

信号定义的权威文件是 `steering_wheel.dbc`（本节点 3 帧）和 `Vehicle_CanB.dbc`（整车 CAN B 协议）。配套显示屏收到 0x700/0x701/0x702 后，合并为 0x703 SteeringCmd 转发给 ECU。

0x310 在 `Vehicle_CanB.dbc` 中的发送节点标为 Display，本板以相同帧格式直接发送。如两侧协议有出入，以 DBC 为准并同步修订。

### 接收

硬件过滤器只放行下列 ID，其余报文丢弃。解析代码在 `Core/Src/can.c` 的 `HAL_CAN_RxFifo0MsgPendingCallback`。

| ID | 内容 | 处理 |
| --- | --- | --- |
| 0x401 | BMS 总电压、电流、SOC、最高温 | 写入 ECU_INFO.bms 和 ERRO[] |
| 0x501 | 车速 | 写入 ECU_INFO.vehicle.speed_kmh |
| 0x502 | 四电机转速 | 写入 ECU_INFO.vehicle.motors[].rpm |
| 0x503、0x504 | 电机故障码（RL/RR、FL/FR） | 写入 motor_error |
| 0x50 | 显示屏发来的 IMU 原始帧 | 按 SubType 转发为 0x60～0x66 |

0x50 帧的转发规则按 byte1（SubType）：0x50/0x51/0x52/0x54 分别重发为 0x60/0x61/0x62/0x66；0x53 时按 byte2 细分为 0x63/0x64/0x65（横滚/俯仰/航向）。转发帧 DLC 固定为 8。

写入 ECU_INFO 的整车数据目前只在本板内部留存，不外发遥测。`Core/Inc/fsae_telemetry.pb.h` 是为编译通过保留的类型 stub，本工程未接入 nanopb。

## 运行行为

核心逻辑在 `Core/Src/button_panel.c`：

- 主循环无阻塞，反复调用 `ButtonPanel_Process()`，内部按 10 ms 周期执行扫描任务。
- 按键和旋钮采用软件去抖，连续 4 次采样一致才更新状态，响应窗口约 40 ms。
- 电位器每次扫描转换一次，做 16 点滑动平均，再按 `平均值 × 8 ÷ 4096` 量化为 0～7。
- 旋钮只认“恰好一档接地”的读数，零档或多档接地判为无效（0xFF）。
- 发送 0x310 后屏蔽 S1～S4 采样 200 ms，等触点稳定后再重新去抖确认档位；档位变化最快约 240 ms 上报一次。
- 邮箱满时发送失败静默丢帧，不重试。

## 目录结构

```
SteeringWheel_Button/
├── Core/
│   ├── Inc/                  # button_panel.h：面板参数与 CAN ID 定义
│   ├── Src/                  # button_panel.c：面板逻辑
│                              # can.c：CAN 收发、过滤与解析
│                              # ecu_info.c：整车信息汇总
│                              # 其余为 CubeMX 生成代码和正点原子模板
│   └── Startup/              # CubeIDE 启动汇编文件
├── Drivers/                   # STM32F1 HAL 与 CMSIS
├── MDK-ARM/                   # Keil MDK 工程
├── cmake/                     # GCC 工具链文件（STM32CubeCLT）
├── CMakeLists.txt             # GCC/Ninja 构建入口
├── CMakePresets.json          # Debug/Release 预设
├── STM32F103XX_FLASH.ld       # 链接脚本
├── SteeringWheel_Button.ioc   # CubeMX 工程，引脚与时钟的源头
├── steering_wheel.dbc         # 本节点 CAN 定义（0x700/0x701/0x702）
├── Vehicle_CanB.dbc           # 整车 CAN B 协议
└── CHANGELOG.md               # 变更记录与待办
```

`main()` 只初始化 GPIO、ADC1、CAN1、TIM1。usart.c、dma.c、delay.c、sys.c 为模板遗留，当前固件没有使用其功能。其中 usart.c 是 interface 工程的 RS485/串口链路代码，两个构建系统都不编译它（CMake 源列表不含，CubeIDE 在 .cproject 中排除）。

## 构建与烧录

### CMake + GCC（STM32CubeCLT）

需要安装 STM32CubeCLT（提供 arm-none-eabi-gcc、CMake、Ninja）。VS Code 装 STM32Cube 扩展后，工程已按 `.vscode/settings.json` 配置好，可直接构建和烧录调试。

命令行方式：

```bash
cmake --preset Debug
cmake --build build/Debug
```

产物为 `build/Debug/SteeringWheel_Button.elf`。当前 Debug 构建占用 FLASH 22.0%、RAM 9.2%。

### Keil MDK

打开 `MDK-ARM/SteeringWheel_Button.uvprojx`，直接构建烧录。

### STM32CubeIDE

工程根目录已含 CubeIDE 工程文件（.cproject、.project、Core/Startup/、STM32F103C8TX_FLASH.ld），用 CubeIDE 直接打开根目录即可构建。Core/Src/usart.c 已在 .cproject 的两个配置中排除编译。

### 修改外设配置

引脚、时钟、外设参数以 `SteeringWheel_Button.ioc` 为准，修改后在 CubeMX 中重新生成代码。自定义代码写在 `/* USER CODE BEGIN */` 与 `/* USER CODE END */` 区间内，重新生成不会覆盖。
