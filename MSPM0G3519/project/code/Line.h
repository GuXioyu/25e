#ifndef LINE_H
#define LINE_H

#include "zf_common_headfile.h"

/* 8 路灰度传感器的通道数量。 */
#define LINE_SENSOR_COUNT (8U)

/* Line_GetSpeed() 的电机选择参数。 */
#define LINE_MOTOR_LEFT  (1U)
#define LINE_MOTOR_RIGHT (2U)

/**
 * @brief 循迹状态。
 */
typedef enum
{
    LINE_MODE_IDLE = 0, /* 尚未获得有效循迹数据。 */
    LINE_MODE_LOST,     /* 8 路传感器均未检测到黑线。 */
    LINE_MODE_STRAIGHT, /* 检测到 1～2 路黑线，可执行常规循迹。 */
    LINE_MODE_FORK      /* 检测到 3 路及以上黑线，判定为岔道。 */
} LineMode_t;

/**
 * @brief 执行一次循迹数据处理。
 * @note 仅在 Line_Timer() 产生采样请求后处理一次。
 */
void Line(void);

/**
 * @brief 获取一次尚未读取的循迹结果标志。
 * @return 1 表示存在新结果，0 表示没有新结果。
 */
uint8_t Line_GetFlag(void);

/**
 * @brief 获取指定电机的循迹目标速度。
 * @param motor LINE_MOTOR_LEFT 或 LINE_MOTOR_RIGHT。
 * @return 对应电机目标速度；参数无效时返回 0。
 */
int16_t Line_GetSpeed(uint8_t motor);

/**
 * @brief 1 ms 定时器节拍函数，每 10 ms 请求一次灰度数据处理。
 */
void Line_Timer(void);

#endif /* LINE_H */
