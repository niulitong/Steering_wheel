# CHANGELOG

记录固件行为、接口和构建的变更。新条目放在最上面。本仓库没有 git 历史，早期变更无从追溯，以 2026-10-18 的代码状态作为基线记录。

## 未发布

固件当前能力基线（未打过版本标签）：

- 按钮面板模块（`Core/Src/button_panel.c`）：
  - 4 位旋钮选择驾驶模式，档位变化时发送 0x310 DriveMode_Request，byte0 为 ASCII '0'..'3'，对应模式 S/Q/C/E；档位无效时发送 0xFF。发送后 200 ms 内不采样旋钮引脚，抑制接触抖动引起的模式跳变。
  - 电位器输入经 16 点滑动平均后量化为 8 级，数值变化时发送 0x700 SlipLevel。
  - KEY1 释放沿发送 0x701 RecorderToggle（bit0=1），KEY2 释放沿发送 0x702 ErrorClear（bit0=1）。
  - 扫描周期 10 ms，去抖窗口约 40 ms，全部为事件驱动发送，上电各强制首发一帧。
- CAN 接收（`Core/Src/can.c`）：硬件过滤只收 0x401、0x501、0x502、0x503、0x504、0x50，解析结果写入 ECU_INFO；0x50 IMU 原始帧按 SubType 转发为 0x60～0x66。
- 遗留代码：can.c 的接收解析、ecu_info 汇总结构、usart/dma/delay/sys 模板来自早前的 interface 工程，编译参与但 main 未初始化 USART/DMA。
- 构建双路径：STM32CubeCLT + CMake/Ninja（Debug/Release 预设）与 Keil MDK 工程。Debug 构建占用 FLASH 22.0%、RAM 9.2%（2026-10-18 实测）。

## 未发布/待办

- 与显示屏协议侧对齐 0x310：`Vehicle_CanB.dbc` 中标发送节点为 Display，本板直接发送同一格式；且 DriveModeCode 声明范围为 [48|51]，固件档位无效时发送的 0xFF 超出该范围。
- `Vehicle_CanB.dbc` 定义了 0x784 DriveMode_0x784（SteeringWheel→VCU/Display，byte0 为 0..3，255=Invalid），固件未实现该帧，需确认 VCU 是否需要。
- `can.c` 中 GBK 编码的中文注释在 UTF-8 环境显示为乱码，需重存为 UTF-8。
- KEY3、KEY4、S5 引脚已在 CubeMX 中配置为输入，固件未使用，接入功能或从 .ioc 移除。
- `Core/Inc/fsae_telemetry.pb.h` 是编译 stub，接入 nanopb 遥测时需替换为真实生成文件。
- 固件无版本号宏，CAN 上也不发版本帧；建议增加版本标识便于赛场排查。

## 2026-10-18

- 新增 `README.md`：硬件参数、引脚表、CAN 收发接口、运行行为、目录结构和两条构建路径，事实均核对自代码与 DBC。
- 扩充 `.gitignore`：原 3 行基础上补充 Keil 布局文件、编译中间产物、调试器日志和系统杂项；保留 `.settings` 跟踪。
- 用 ninja 增量构建验证 Debug 配置通过，产物 `build/Debug/SteeringWheel_Button.elf`。
- 初始化 git 仓库，首次导入 127 个文件并推送到 `origin/main`（https://github.com/niulitong/Steering_wheel）。
- `button_panel.c/h`：0x310 发送后增加 200 ms 旋钮检测屏蔽窗口（`DRIVE_MODE_HOLD_MS`），抑制触点抖动导致的模式跳变；ninja 构建通过，FLASH 22.0%。
