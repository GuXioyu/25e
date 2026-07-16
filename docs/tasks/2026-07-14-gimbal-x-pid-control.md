# 云台 X 轴 PID 与 Y 轴开环控制

## Requirement Summary

- UART4 接收固定格式 `[x,y]`，图像尺寸为 `640 x 360`。
- X 轴复用 `PID.c` 做闭环控制，Y 轴使用固定比例开环控制。
- `[0,0]` 表示未检测到目标，云台保持当前位置。
- 保持 `main.c`、UART、PID 和 RDK 代码不变；Motor 仅增加相对角度接口。

## Confirmed Approach

- 在 `Gimbal.c` 内收敛帧复制和坐标解析，不增加公共协议接口。
- X PID 输出作为单帧角度增量，直接调用 `Motor_SetRelAngle()`。
- Y 轴按像素误差直接换算角度增量，并使用同一单帧限幅。
- 图像中心直接使用 `ImgWidth / 2` 和 `ImgHeight / 2`，不保留中心坐标宏。
- 不定义 `X_Dir`、`Y_Dir` 等方向变量；X 轴 PID 输出在使用处直接乘 `-1.0f`，Y 轴使用误差自身的正负表示方向。

## Files To Change

- `MSPM0G3519/project/user/src/Gimbal.c`
- `MSPM0G3519/project/user/inc/Gimbal.h`
- `MSPM0G3519/project/code/Motor.c`
- `MSPM0G3519/project/code/Motor.h`
- `AGENTS.md`
- 本任务文档和对应设计文档

## Function Or Module Design

- `Gimbal_ReadXY()`：输入 x/y 输出指针；输出是否读取到新帧；副作用是消费 UART4 接收标志；协议错误不额外兼容，按已确认的 `"x,y"` 契约解析。
- `Gimbal_GetYStep()`：输入 Y 像素误差；输出限幅后的角度增量；无外部副作用。
- `Gimbal_Init()`：初始化 X PID，不维护绝对角度累计状态。
- `Gimbal_Task()`：处理 `[0,0]`、X PID 和 Y 比例控制。
- `Motor_SetRelAngle()`：输入相对角度增量，使用 EMM 相对位置模式发送命令。

## Task Breakdown

- [x] 合并 UART4 取帧与 x/y 解析。
- [x] 初始化并接入 X 轴 PID。
- [x] 保留 Y 轴比例开环并更新图像中心。
- [x] 更新头文件说明。
- [x] 完成静态验证和范围复核。

## Commenting Plan

- 宏、全局状态和局部变量使用中文行尾注释说明用途、单位和范围。
- 临界区、PID 数据流、直接符号换向和 `[0,0]` 分支说明控制原因。
- 函数注释说明输入、输出、副作用和错误行为。

## Verification Checklist

- [x] `Gimbal.c` 通过编译级语法检查。
- [x] `[x,y]` 的最大内容 `"639,359"` 可放入 8 字节缓存。
- [x] X 轴只使用 PID 输出，Y 轴不调用 PID。
- [x] 未定义方向变量；X 轴直接乘 `-1.0f`，Y 轴直接保留误差正负。
- [x] `[0,0]` 不发送新电机角度命令。
- [x] 本次仅为相对位置接口修改 Motor；未修改 `main.c`、PID、UART 和 RDK 文件。

## Open Questions And Decisions

- 电机实际转向仍需上板确认；若 X 轴方向相反，直接调整 PID 输出使用处的负号，不新增方向变量。
- 机械角度软限位不在本次范围内。
- 实机收敛速度和 `TEST-B2/B3/F1/F2` 的全程最大 D1 不由静态检查证明。
