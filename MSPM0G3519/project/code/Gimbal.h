#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* Gimbal_GetSpeed() 的电机选择参数 */
#define GIMBAL_MOTOR_YAW    (1U)
#define GIMBAL_MOTOR_PITCH (2U)

/**
 * @brief  初始化云台 PID 控制器
 * @param  无
 * @return 无
 */
void Gimbal_Init(void);

/**
 * @brief  执行一次云台数据处理,不处理数据
 * @note   仅在 Gimbal_Timer() 产生采样请求后处理一次
 * @param  无
 * @return 无
 */
void Gimbal(void);

/**
 * @brief  获取一次尚未读取的云台结果标志
 * @param  无
 * @return 1 表示存在新结果，0 表示没有新结果
 */
uint8_t Gimbal_GetFlag(void);

/**
 * @brief  获取指定轴的云台目标速度
 * @param  motor GIMBAL_MOTOR_YAW 或 GIMBAL_MOTOR_PITCH
 * @return 对应轴目标速度；参数无效时返回 0
 */
float Gimbal_GetSpeed(uint8_t motor);

/**
 * @brief  从 UART4 读取并解析 `cam,x,y` 格式的坐标帧
 * @return 读取到新帧返回 1U；无新帧返回 0U
 * @note   数据格式为[cam,x,y]，例如[cam,320,180]
 */
uint8_t Gimbal_ReadXY(void);

/**
 * @brief  获取图像坐标
 * @param  axis 1 返回 x 坐标，2 返回 y 坐标
 * @return 对应坐标值；参数无效时返回 0
 */
uint16_t Gimbal_GetXY(uint8_t axis);

/**
 * @brief  1 ms 定时器节拍函数，每 10 ms 请求一次云台处理
 * @param  无
 * @return 无
 */
void Gimbal_Timer(void);

#endif /* GIMBAL_H */