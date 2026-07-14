#ifndef Gimbal_H
#define Gimbal_H // 防止云台模块头文件被重复包含。

/**
 * @brief 初始化二维云台电机和 X 轴 PID 控制器。
 */
void Gimbal_Init(void);

/**
 * @brief 处理 UART4 最新 `[x,y]` 坐标帧，执行 X 轴 PID 闭环和 Y 轴比例开环控制。
 */
void Gimbal_Task(void);

#endif // Gimbal_H
