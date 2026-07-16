# Gimbal_ReadXY 直接更新全局坐标

## 目标需求

- 修改 `Gimbal_ReadXY`，使其获得 UART4 图像坐标数据后直接修改云台模块内的全局坐标变量。
- 保持现有 `Gimbal_GetImage` 对有效帧标志的处理流程。

## 涉及模块

- `project/code/Gimbal.c`

## 实现思路

- 将 `Gimbal_ReadXY` 从带 `x`、`y` 输出参数的接口改为无参数静态函数。
- 解析到合法 `cam,x,y` 帧后，直接写入 `gimbal_x` 和 `gimbal_y`。
- `Gimbal_GetImage` 只调用 `Gimbal_ReadXY()` 判断是否读到有效帧。

## 执行步骤

1. 阅读 `Gimbal.c`、`Gimbal.h` 和云台调用点，确认坐标变量使用范围。
2. 修改 `Gimbal_ReadXY` 函数注释、定义和内部赋值逻辑。
3. 修改 `Gimbal_GetImage` 调用方式。
4. 运行可用的文本检查或构建验证。

## 验证方法

- 使用搜索确认不存在旧的 `Gimbal_ReadXY(&...)` 调用。
- 如本机具备项目编译环境，运行项目构建；否则记录无法构建原因和人工验证方法。

## 已确认决定

- 用户确认修改目标是 `Gimbal_ReadXY`。
- 解析成功后直接更新 `gimbal_x`、`gimbal_y`。

## 待澄清问题

- 无。
