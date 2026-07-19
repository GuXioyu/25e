# angle 变量类型调整

## 目标需求

- 将 `Task.c` 中用于接收云台角度增量的 `angle_yaw`、`angle_pitch` 改为 `float`。
- 避免 `Gimbal_GetSpeed()` 返回的浮点结果被隐式转换为 `int16_t` 后截断。

## 涉及模块

- `code/Task.c`
- 关联接口：`Gimbal_GetSpeed()`、`Motor_SetRelAngle()`、`Motor_SetAbsAngle()`

## 实现思路

- 保持 `speed_left`、`speed_right`、`speed_yaw`、`speed_pitch` 为 `int16_t`，继续匹配 `Motor_SetSpeed()`。
- 仅将 `angle_yaw`、`angle_pitch` 改为 `float`，继续匹配电机角度接口的 `float angle_deg` 参数。
- 不修改云台控制逻辑和电机发送调度逻辑。

## 执行步骤

1. 阅读 `Task.c` 中速度、角度变量定义和云台预留控制段。
2. 使用静态断言确认 `angle_yaw`、`angle_pitch` 当前不是 `float`。
3. 修改 `Task.c` 中 `angle_yaw`、`angle_pitch` 的变量类型。
4. 再次运行静态断言确认类型已调整。

## 验证方法

- 静态断言：检查 `Task.c` 中存在 `float angle_yaw, angle_pitch;`。
- 若本机存在 MDK/armclang，再执行完整工程编译确认无新增告警。

## 已确认决定

- 本次只处理角度变量的浮点截断问题。
- 不修改 `speed_yaw`、`speed_pitch` 的类型。
- 不处理 `Task_Motor()` 中云台预留分支的比较变量和 `break` 风险。

## 待澄清问题

- 无。
