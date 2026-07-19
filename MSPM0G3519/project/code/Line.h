#ifndef LINE_H
#define LINE_H

#include "zf_common_headfile.h"
#include "GraySensor.h"
#include "My_Uart.h"
#include "HWT101.h"

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
    LINE_STATE_IDLE = 0, /* 尚未获得有效循迹数据。 */
    LINE_STATE_LOST,     /* 8 路传感器均未检测到黑线。 */
    LINE_STATE_STRAIGHT, /* 检测到 1～2 路黑线，可执行常规循迹。 */
    LINE_STATE_FORK      /* 检测到 3 路及以上黑线，判定为岔道。 */
} LineState_t;

/**
 * @brief 岔道类型。
 */
typedef enum
{
    LINE_FORK_TYPE_IDLE = 0,              /* 默认状态，当前未识别到具体岔道类型。 */
    LINE_FORK_TYPE_LEFT_RIGHT_ANGLE = 1,  /* 左直角岔道。 */
    LINE_FORK_TYPE_RIGHT_RIGHT_ANGLE = 2  /* 右直角岔道。 */
} LineForkType_t;

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

/*
当前每 10ms 执行一次 Line()，逻辑如下：
读取 8 路灰度和当前偏航角，1 表示检测到黑线。

1正常循迹：
黑线为单段且数量 1~2，进入 LINE_STATE_STRAIGHT。
根据黑线平均位置计算左右轮速度。

2短时丢线：
黑线数量为 0 时开始计数。
未满 100ms 时，若之前处于直线循迹，会保留原来的 line_location 和直线状态，继续按上一帧方向循迹。
连续 100ms 无黑线，进入 LINE_STATE_LOST，两轮速度置 0。

3岔道识别：
黑线超过 2 路，或出现多个黑线段，进入 LINE_STATE_FORK。
左直角条件是 0、1、2 号传感器为黑线，5、6、7 号为白底；3、4 号当前不参与判断。
识别成功后锁定起始角度，左轮输出 25，右轮输出 80。

4左转完成：
计算当前角度相对起始角度的差，并处理 -180°/+180° 回绕。
只有角度差达到 <= -85° 才认为左转完成，清除岔道类型并置为 IDLE。

5误识别岔道：
未满足左直角条件时，直接回到 LINE_STATE_STRAIGHT，根据当前黑线平均位置继续普通循迹，不会停车。
*/
