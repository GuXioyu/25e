# MSPM0G3519 UART4 视觉坐标驱动云台居中任务 / UART4 Vision-Driven Gimbal Centering Task

## 需求摘要 / Requirement Summary

- UART4 接收摄像头按约定发送的 ASCII 正整数坐标帧 `[x,y]`，其中 `x=0..639`、`y=0..319`；固件信任该上游格式。
  UART4 receives contract-defined ASCII positive-integer coordinate frames in `[x,y]` format, where `x=0..639` and `y=0..319`; the firmware trusts this upstream format.
- 地址 3 控制 yaw，地址 4 控制 pitch，使目标收敛到 640×320 画面的中心 `(320,160)`。
  Motor address 3 controls yaw and address 4 controls pitch so the target converges to `(320,160)`, the center of a 640×320 image.
- 本轮只实现画面坐标居中。它是满足 `REQ-SYS-02` 的控制基础，不等同于实物激光命中靶心。
  This task only implements image-coordinate centering. It supports `REQ-SYS-02` but does not by itself prove that the physical laser hits the target center.
- 最终实机仍须按 `TEST-B2` 的 2 秒、`TEST-B3` 的 4 秒以及 D1≤2 cm 要求验收。
  Final hardware acceptance must still satisfy the 2-second `TEST-B2`, 4-second `TEST-B3`, and D1≤2 cm requirements.

## 已确认方案 / Confirmed Approach

- 在 `project/user` 下建立独立 `Gimbal.c/.h`，不把新逻辑放入 `Task.c/Task.h`。
  Create standalone `Gimbal.c/.h` files under `project/user`; do not place the new logic in `Task.c/Task.h`.
- UART4 继续使用现有 `[`、`]` 接收状态机；包体直接读取两个正整数，不重复检查符号、小数、空格等摄像头不会发送的格式；UART3 继续专用于 EMM 电机通信。
  UART4 keeps the existing `[` and `]` receive state machine, while UART3 remains dedicated to EMM motor communication.
- 使用增量 P 控制：像素误差乘“度/像素”和方向符号，限制单帧增量后累加到绝对目标角度。
  Use incremental proportional control: multiply pixel error by degrees-per-pixel and an axis direction sign, clamp the per-frame step, then accumulate an absolute target angle.
- 默认 yaw/pitch 比例均为 `0.02°/pixel`，方向均为 `+1.0`，死区为 2 pixel，单帧最大增量为 `2.0°`。
  Default yaw/pitch gains are `0.02°/pixel`, both direction signs are `+1.0`, the deadband is 2 pixels, and the maximum per-frame step is `2.0°`.
- 云台允许连续旋转，本轮不加入机械角度限制、I/D 项、滤波器或丢帧复位逻辑。
  The gimbal may rotate continuously; this task adds no mechanical angle limit, I/D term, filter, or lost-frame reset behavior.

## 修改文件 / Files To Change

- 新建 `MSPM0G3519/project/user/inc/Gimbal.h`：只公开初始化和周期任务接口。
  Create `MSPM0G3519/project/user/inc/Gimbal.h`: expose only initialization and periodic-task APIs.
- 新建 `MSPM0G3519/project/user/src/Gimbal.c`：负责帧快照、读取约定坐标和两轴控制。
  Create `MSPM0G3519/project/user/src/Gimbal.c`: own frame snapshotting, contract-defined coordinate reading, and two-axis control.
- 修改 `MSPM0G3519/project/user/src/main.c`：初始化并持续调用云台模块。
  Modify `MSPM0G3519/project/user/src/main.c`: initialize and continuously call the gimbal module.

## 函数与模块设计 / Function And Module Design

### `void Gimbal_Init(void)`

- 目的：初始化地址 3/4 的静态电机对象，将当前位置清零，使能电机，并清零两个累计目标角度。
  Purpose: initialize static motor objects at addresses 3/4, zero their current positions, enable them, and clear both accumulated target angles.
- 输入与输出：无参数、无返回值。
  Input and output: no parameters and no return value.
- 副作用：通过 UART3 向两个 EMM 驱动器发送清零和使能命令。
  Side effects: sends zero-position and enable commands to both EMM drivers through UART3.
- 错误行为：沿用现有电机接口的无返回值行为，不新增阻塞等待或回包确认。
  Error behavior: follows the existing void motor APIs and adds no blocking wait or reply acknowledgement.

### `void Gimbal_Task(void)`

- 目的：消费 UART4 最新完整帧，直接读取约定的两个正整数，并按两轴增量 P 控制更新云台目标。
  Purpose: consume the latest complete UART4 frame and update the gimbal target using two-axis incremental proportional control.
- 输入与输出：无参数、无返回值；输入来自 UART4 接收缓存。
  Input and output: no parameters and no return value; input comes from the UART4 receive buffer.
- 副作用：有效且超出死区的轴会调用 `Motor_SetAbsAngle()`；无帧或无效帧不会发送命令。
  Side effects: axes with valid errors outside the deadband call `Motor_SetAbsAngle()`; missing or invalid frames send no commands.
- 内部流程：原子复制帧并清标志 → 读取两个正整数 → 计算像素误差 → 应用死区、比例和限幅 → 累计绝对角度并发送。
  Internal flow: atomically copy and clear the frame → read two positive integers → calculate pixel error → apply deadband, gain, and clamp → accumulate absolute angles and send commands.

## 任务拆分 / Task Breakdown

- [x] 实现 `Gimbal.c/.h`，并核对初始化、约定坐标、死区、限幅和角度累计逻辑。
  Implement `Gimbal.c/.h` and review initialization, contract-defined coordinates, deadband, clamping, and angle accumulation logic.
- [x] 在 `main.c` 中调用 `Gimbal_Init()` 和 `Gimbal_Task()`，删除固定 100 ms 延时。
  Call `Gimbal_Init()` and `Gimbal_Task()` from `main.c`, and remove the fixed 100 ms delay.
- [x] 审查最终差异，确认 `Task.c/Task.h`、UART 接收层和 UART3 电机通道未被改变。
  Review the final diff and confirm `Task.c/Task.h`, the UART receiver, and the UART3 motor channel remain unchanged.

## 注释规范 / Commenting Plan

- 新增宏采用与函数类似的首字母大写命名，例如 `Gimbal_YawAddr`；每个宏在定义右侧写中文注释，说明用途、单位、范围或调参方向。
  Name new macros with a function-like initial capital, such as `Gimbal_YawAddr`; put a Chinese comment to the right of every macro explaining its purpose, unit, range, or calibration meaning.
- 已确认的上游数据契约直接使用，不为符号、小数、空格等不会出现的输入增加冗余保护；不确定是否需要保护时先询问用户。
  Use confirmed upstream data contracts directly; do not add redundant guards for signs, decimals, whitespace, or other impossible input, and ask the user first whenever the need for a guard is uncertain.
- 关键变量说明坐标、角度单位和累计含义；分支注释说明拒绝或不动作的原因。
  Document coordinate and angle units plus accumulated-state meaning; branch comments explain why input is rejected or motion is skipped.
- 保持注释紧邻代码，不注释显而易见的语法，不修改无关旧代码风格。
  Keep comments close to the code, avoid restating obvious syntax, and do not reformat unrelated existing code.

## 可读性收尾（2026-07-14）/ Readability Follow-up (2026-07-14)

- [x] 为 `Gimbal.c/.h` 的宏、静态状态、函数参数和局部变量补充中文注释，标出用途、单位、范围或标定方向。
  Add Chinese comments for macros, static state, function parameters, and local variables in `Gimbal.c/.h`, documenting purpose, units, ranges, or calibration direction.
- [x] 为临界区复制、可信帧解析、死区过滤、限幅和绝对角度累计等关键控制步骤补充原因说明。
  Add rationale comments for the critical-control steps: critical-section copying, trusted-frame parsing, deadband filtering, limiting, and absolute-angle accumulation.
- [x] 审查重复的两轴控制分支后保持其扁平结构；提取通用轴更新函数会引入电机和目标角度指针，降低当前调试可读性，故本轮不作结构重构。
  Keep the two explicit axis-control branches after review; extracting a generic axis-update function would add motor and target-angle pointers and reduce current debugging clarity, so this follow-up does not refactor the structure.

## 验证清单 / Verification Checklist

- [x] 源码核对：接受 `[320,160]`、`[0,0]`、`[639,319]`，帧后 `\r\n` 不影响接收。
  Accept `[320,160]`, `[0,0]`, and `[639,319]`; trailing `\r\n` does not affect reception.
- [x] 源码核对：中心和 2 像素死区不动作，大误差单帧增量不超过 `2.0°`。
  The center and 2-pixel deadband produce no motion, while large errors never exceed a `2.0°` per-frame step.
- [x] 源码核对：无新帧或不完整帧不发送命令，重复有效帧会累加绝对目标角度。
  Missing or incomplete frames send no command, while repeated valid frames accumulate the absolute target angle.
- [x] 源码核对：新控制只使用电机地址 3/4，底层电机命令仍只通过 UART3 发送。
  New control uses only motor addresses 3/4, and low-level motor commands still use UART3 exclusively.
- [ ] 由用户将 `Gimbal.c` 加入本地 Keil 源文件组，确认 `user/inc` 位于包含路径中并执行编译；用户把具体报错发回后再逐条修改。
  The user adds `Gimbal.c` to the local Keil source group, confirms `user/inc` is on the include path, and performs the build; reported compiler errors are then fixed one by one.
- [ ] 实机标定方向和“度/像素”，再执行 `TEST-B2/B3` 并记录时间与最大 D1。
  Calibrate axis signs and degrees-per-pixel on hardware, then run `TEST-B2/B3` and record time and maximum D1.

## 假设与待确认项 / Assumptions And Open Questions

- 假设图像原点在左上角，x 向右增加，y 向下增加。
  Assume the image origin is at the top-left, with x increasing rightward and y increasing downward.
- UART4 保持 57600 baud、PB10/PB11；串口发送端必须与此配置一致。
  UART4 remains at 57600 baud on PB10/PB11; the sender must use the same configuration.
- 两轴方向宏和比例宏是初始值，必须通过小幅实机运动完成最终标定。
  Axis direction and gain macros are initial values and require final calibration using small hardware movements.
- 仓库未检入 `.uvprojx`，并且用户明确由自己执行 Keil 编译；Codex 不以本地编译作为本轮交付步骤，也不声称全工程编译通过。
  The repository does not contain a `.uvprojx`, and the user explicitly performs the Keil build; Codex does not treat local compilation as a delivery step and does not claim that the full project builds successfully.
