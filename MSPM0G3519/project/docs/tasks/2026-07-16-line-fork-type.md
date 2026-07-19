# Line 岔道类型状态变量

## 目标需求

- 在 Line 模块新增一个状态变量，用于记录岔道类型。
- 岔道类型枚举先包含 3 个取值：
  - 0：与其他枚举一样，表示 `IDLE` 默认状态。
  - 1：左直角。
  - 2：右直角。
- 新增岔道类型判断函数，先只实现左直角逻辑。
- 新增岔道速度设置函数，左直角两轮速度通过宏定义引出，初值左轮 60、右轮 80。
- 左直角进入后记录起始角度，当前角度相对起始角度变化达到 `>= 85°` 后将 `line_state` 置为 `LINE_STATE_IDLE`。
- 给丢线增加延时确认，连续 100ms 未检测到黑线后才进入 `LINE_STATE_LOST`。
- 进入岔道但未识别为左直角时，释放 `LINE_STATE_FORK` 回到 `LINE_STATE_IDLE`，避免卡在岔道状态。

## 涉及模块

- `project/code/Line.c`
- `project/code/Line.h`

## 实现思路

- 在 `Line.h` 新增 `LineForkType_t` 枚举类型，命名风格与 `LineState_t` 保持一致。
- 在 `Line.c` 使用静态状态变量 `line_fork_type` 保存当前岔道类型。
- 使用 `line_digital[0]`、`line_digital[1]`、`line_digital[2]` 同时有效作为左直角进入条件。
- 使用 `HWT101_GetYaw()` 读取的 `line_angle` 与进入岔道时的起始角做差，先将差值归一化到 `-180°～180°`；左转仅在负方向差值达到阈值后退出岔道状态。
- 基于 10ms 采样周期新增丢线计数，连续 10 次采样无线才判定丢线，重新检测到黑线时清零计数。
- 在 `Line_CheckForkType()` 中，如果 `line_fork_type` 仍为 `IDLE` 且当前帧不满足左直角条件，则将 `line_state` 置为 `LINE_STATE_STRAIGHT`，沿用最近一次有效位置继续直线循迹。
- 丢线确认的 100ms 内保持 `LINE_STATE_STRAIGHT`，沿用最近一次有效位置继续直线循迹；确认丢线后才进入 `LINE_STATE_LOST`。
- 左直角完成时直接切换到 `LINE_STATE_STRAIGHT`，在当前采样周期继续计算直线循迹速度，不进入 `LINE_STATE_IDLE` 输出零速度。
- 普通循迹速度参数恢复为原版配置：基础速度 120、最大减速量 40、单侧最大差速 30。

## 执行步骤

1. 确认当前工程中不存在岔道类型枚举标识。
2. 新增岔道类型枚举。
3. 新增岔道类型状态变量并补充中文注释。
4. 新增左直角角度完成判断函数。
5. 新增左直角岔道速度设置函数。
6. 补全当前代码中缺少的中文注释。
7. 新增 100ms 丢线延时确认。
8. 检查旧标识和新标识引用范围。

## 验证方法

- 使用 `rg` 检查新增的 `LineForkType_t`、`LINE_FORK_TYPE_*` 和 `LineForkType` 是否存在于预期文件中。
- 使用 `rg` 检查 `Line_CheckForkType()`、`Line_SetForkSpeed()`、`LINE_FORK_FINISH_ANGLE_DEG`、`LINE_FORK_LEFT_SPEED_LEFT`、`LINE_FORK_LEFT_SPEED_RIGHT` 是否存在于预期文件中。
- 使用 `rg` 检查 `LINE_LOST_CONFIRM_TIME_MS`、`LINE_LOST_CONFIRM_COUNT` 和 `line_lost_count` 是否存在于预期文件中。
- 使用 `rg` 检查是否存在命名遗漏。
- 如本机具备 Keil 命令行，再运行工程编译；否则说明无法编译原因。

## 验证结果

- RED：实现前使用 `rg` 查询 `LineForkType_t|LINE_FORK_TYPE_|line_fork_type`，结果无匹配，确认新增标识尚不存在。
- GREEN：实现后使用 `rg` 确认 `LINE_FORK_TYPE_IDLE = 0`、`LINE_FORK_TYPE_LEFT_RIGHT_ANGLE = 1`、`LINE_FORK_TYPE_RIGHT_RIGHT_ANGLE = 2` 和 `line_fork_type` 均位于预期文件。
- 命名检查：使用 `rg` 确认 `LINE_FORK_TYPE_NONE` 无残留。
- 编译验证：本机 PATH 未找到 `UV4.exe` 或 `UV5.exe`，未运行 Keil 工程编译。
- RED：新增左直角逻辑前使用 `rg` 查询 `Line_CheckForkType|Line_SetForkSpeed|LINE_FORK_FINISH_ANGLE|LINE_FORK_LEFT_SPEED`，结果无匹配，确认新增函数和宏尚不存在。
- GREEN：新增左直角逻辑后使用 `rg` 确认 `Line_CheckForkType()`、`Line_SetForkSpeed()`、`LINE_FORK_FINISH_ANGLE_DEG`、`LINE_FORK_LEFT_SPEED_LEFT`、`LINE_FORK_LEFT_SPEED_RIGHT` 均位于 `Line.c`。
- 注释检查：使用 `rg` 确认 `Line.c` 中无 `//` 注释残留，且原一行 `if (line_timer_flag == 0U)return;` 已改为带花括号的早退。
- 空白检查：使用 `git diff --check -- Line.c Line.h 任务文档` 检查通过，仅提示工作区文件下次由 Git 接触时会从 LF 转为 CRLF。
- 编译验证：本机 PATH 和常见安装路径未找到 `UV4.exe` 或 `UV5.exe`，未运行 Keil 工程编译。
- RED：新增丢线延时逻辑前使用 `rg` 查询 `LINE_LOST_CONFIRM|line_lost_count`，结果无匹配，确认丢线延时标识尚不存在。
- GREEN：新增丢线延时逻辑后使用 `rg` 确认 `LINE_LOST_CONFIRM_TIME_MS`、`LINE_LOST_CONFIRM_COUNT`、`line_lost_count` 和 `line_state = LINE_STATE_LOST` 均位于 `Line.c`。
- 注释检查：使用 `rg` 确认 `Line.c/.h` 中无 `//` 注释残留，且旧一行早退未恢复。
- 空白检查：使用 `git diff --check -- Line.c Line.h 任务文档` 检查通过，仅提示工作区文件下次由 Git 接触时会从 LF 转为 CRLF。
- 编译验证：本机 PATH 和常见安装路径未找到 `UV4.exe` 或 `UV5.exe`，未运行 Keil 工程编译。

## 已确认决定

- 用户确认 0、1、2 三个取值先分别表示 `IDLE` 默认状态、左直角、右直角。
- 用户确认角度完成阈值使用 `>= 85°`。
- 用户确认先只实现左直角。
- 用户确认左直角速度宏初值为左轮 60、右轮 80。
- 用户确认丢线需要连续 100ms 后才判定。
- 用户确认丢线确认的 100ms 内继续执行直线循迹逻辑。
- 用户确认未识别岔道可能属于误识别，应保持直线前进。
- 用户确认左转时偏航角按负方向变化，且 `HWT101_GetYaw()` 的角度范围为 `-180°～180°`，完成判定必须处理回绕。
- 用户确认 `huart5` 串口输出用于调试，暂时保留。
- 用户确认当前左直角识别条件和速度参数暂时合理，保持不变。
- 用户确认左直角完成后不得输出零速度，应直接恢复直线循迹。
- 用户确认普通循迹速度参数回退到原版配置。

## 待澄清问题

- 后续需要确认右直角的传感器判定条件和速度参数。
