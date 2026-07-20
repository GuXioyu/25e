# PID 前馈项扩展

## 目标需求

- 在 PID 模块增加独立的前馈控制项。
- 前馈项计算公式为 `kf × (error_current - error_past)`。
- 使用 `mode_f` 控制前馈项；值为 0 时禁用，值为 1 时启用。

## 涉及模块

- `code/PID.h`
- `code/PID.c`

## 实现思路

- 新增 `PID_FMODE_DISABLE`、`PID_FMODE_ENABLE`、`mode_f` 和 `kf`。
- 在更新当前误差后，使用已保存的 `error_past` 计算本次前馈输出。
- 将前馈输出与 P、I、D 项相加后，再复用现有的输出偏置、限幅和死区处理。
- 前馈项不保存额外状态，不修改外部调用模块。

## 执行步骤

1. 已为 PID 结构体和头文件增加前馈模式与前馈系数。
2. 已在 PID 更新函数中按 `mode_f` 计算前馈输出并参与最终合成。
3. 已使用临时测试验证前馈启用、禁用和 P/F 合成输出。

## 验证方法

- 已运行 `gcc -std=c11 -Wall -Wextra -Werror -I ..\code C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.c ..\code\PID.c -lm`，编译并运行通过且无警告。
- 已验证 `mode_f = 1`、`kf = 2.0f`、本次误差为 `3.0f`、上次误差为 `1.0f` 时前馈输出为 `4.0f`。
- 已验证 `mode_f = 0` 时输出为 `0.0f`，以及比例项为 `3.0f`、前馈项为 `4.0f` 时合成输出为 `7.0f`。
- 已运行 `gcc -std=c11 -Wall -Wextra -Werror -I ..\code -fsyntax-only ..\code\PID.c`，语法检查通过且无警告。
- 未运行 MDK 工程级构建：环境变量中没有 `UV4.exe` 或 ARM 编译器。人工验证可在 `mdk/SeekFree_MSPM0G3519_Device_Library.uvprojx` 中设置 `mode_f`、`kf` 后单步检查前馈输出和总输出。

## 已确认决定

- `mode_f` 为 0 时禁用，为 1 时启用。
- 前馈系数成员命名为 `kf`。
- 前馈项独立于微分项，不复用 `kd` 或 `mode_d`。
- 本次不修改外部调用模块。

## 待澄清问题

- 无。
