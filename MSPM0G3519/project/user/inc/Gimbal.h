#ifndef Gimbal_H
#define Gimbal_H // 防止云台模块头文件被重复包含。

/**
 * @brief 初始化二维云台电机，并将上电时的位置作为两个轴的角度零点。
 */
void Gimbal_Init(void);

/**
 * @brief 处理 UART4 最新 `[x,y]` 坐标帧，执行 X 轴 PID 闭环和 Y 轴比例开环控制。
 */
void Gimbal_Task(void);

#endif // Gimbal_H
