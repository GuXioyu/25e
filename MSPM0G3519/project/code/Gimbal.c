/**
 * @file Gimbal.c
 * @brief 使用 UART4 目标坐标计算云台目标速度。
 */

#include "Gimbal.h"

#include <stdint.h>

#include "My_Uart.h"
#include "PID.h"
#include "zf_common_headfile.h"

/* 云台数据处理周期，单位为毫秒 */
#define GIMBAL_SAMPLE_PERIOD_MS (10U)

/* 图像参数 */
#define ImgWidth      640U                /* 视觉图像宽度，单位：像素 */
#define ImgHeight     360U                /* 视觉图像高度，单位：像素 */

/* PID 参数 */
#define XKp           0.02f               /* X 轴 PID 比例系数 */
#define XKi           0.0f                /* X 轴 PID 积分系数 */
#define XKd           0.0f                /* X 轴 PID 微分系数 */

/* 控制参数 */
#define YDegPerPx     0.02f               /* Y 轴像素误差到角度增量的比例，单位：度/像素 */
#define DeadbandPx    2                   /* 绝对误差不超过该值时不动作，单位：像素 */
#define MaxStepDeg    2.0f                /* 两轴单帧最大角度增量，单位：度 */

/* 定时器中断置位、主循环读取的采样请求标志 */
static volatile uint8_t gimbal_timer_flag;

/* 云台模块已完成一次数据处理的通知标志 */
static uint8_t gimbal_flag;

/* 当前计算出的云台两轴目标速度 */
static float gimbal_speed_yaw;
static float gimbal_speed_pitch;

/* 当前帧的图像坐标 */
static uint16_t gimbal_x;
static uint16_t gimbal_y;
static uint8_t gimbal_image_valid;

/* X 轴图像坐标闭环 PID 状态 */
static PID_t g_x_pid;

/**
 * @brief  从 UART4 读取并解析 `cam,x,y` 格式的坐标帧
 * @param  x 输出图像 x 坐标，范围 0~639，单位：像素
 * @param  y 输出图像 y 坐标，范围 0~359，单位：像素
 * @return 读取到新帧返回 1U；无新帧返回 0U
 * @note   数据格式为[cam,x,y]，例如[cam,320,180]
 */
static uint8_t Gimbal_ReadXY(uint16_t *x, uint16_t *y)
{
	if (Serial_GetRxFlag(&huart4))
    {
        char *Tag = strtok((char *)Serial_GetRxPacket(&huart4), ",");
        if (Tag != NULL && strcmp(Tag, "cam") == 0)
        {
            char *Value1 = strtok(NULL, ", ");
            char *Value2 = strtok(NULL, ", ");
            if (Value1 != NULL && Value2 != NULL)
            {
                int IntValue1 = atof(Value1);
                *x = IntValue1;

                int IntValue2 = atof(Value2);
                *y = IntValue2;

                return 1U;
            }
        }
    }
    return 0U;
}

/**
 * @brief  读取最新一帧图像坐标
 * @note   串口模块通过 UART4 异步更新数据，因此本次使用的是已解析完成的最新帧
 */
static void Gimbal_GetImage(void)
{
    if (Gimbal_ReadXY(&gimbal_x, &gimbal_y) == 0U)
    {
        gimbal_image_valid = 0U;
        return;
    }
    gimbal_image_valid = 1U;
}

/**
 * @brief  将 Y 轴像素误差换算为受限的开环角度增量
 * @param  error_y 目标相对图像中心的 Y 轴偏差，单位：像素
 * @return 限制在 ±MaxStepDeg 内的角度增量，单位：度
 */
static float Gimbal_GetYStep(int16_t error_y)
{
    float step_deg;                         /* Y 轴当前帧需要增加的目标角度，单位：度 */

    step_deg = (float)error_y * YDegPerPx;

    /* 限制角度增量范围 */
    if (step_deg > MaxStepDeg)
    {
        step_deg = MaxStepDeg;
    }
    else if (step_deg < -MaxStepDeg)
    {
        step_deg = -MaxStepDeg;
    }

    return step_deg;
}

/**
 * @brief  根据图像坐标计算云台两轴目标速度
 */
static void Gimbal_CalculateSpeed(void)
{
    int16_t error_x;                        /* 目标相对图像中心的 X 轴偏差，单位：像素 */
    int16_t error_y;                        /* 目标相对图像中心的 Y 轴偏差，单位：像素 */

    /* 没有有效图像数据时，清零速度 */
    if (gimbal_image_valid == 0U)
    {
        gimbal_speed_yaw = 0.0f;
        gimbal_speed_pitch = 0.0f;
        return;
    }

    /* 未检测到目标时清零速度和 PID 状态 */
    if ((gimbal_x == 0U) && (gimbal_y == 0U))
    {
        g_x_pid.actual_current = (float)ImgWidth / 2.0f;
        g_x_pid.actual_past = (float)ImgWidth / 2.0f;
        g_x_pid.error_current = 0.0f;
        g_x_pid.error_past = 0.0f;
        g_x_pid.error_int = 0.0f;
        g_x_pid.out = 0.0f;
        gimbal_speed_yaw = 0.0f;
        gimbal_speed_pitch = 0.0f;
        return;
    }

    /* 计算目标相对图像中心的偏差 */
    error_x = (int16_t)gimbal_x - (int16_t)(ImgWidth / 2U);
    error_y = (int16_t)gimbal_y - (int16_t)(ImgHeight / 2U);

    /* X 轴 PID 闭环计算 */
    g_x_pid.actual_current = (float)gimbal_x;
    PID_Update(&g_x_pid);

    /* 仅在 X 轴偏差超出死区时更新 Yaw 速度 */
    if ((error_x > DeadbandPx) || (error_x < -DeadbandPx))
    {
        gimbal_speed_yaw = g_x_pid.out * -1.0f;
    }
    else
    {
        g_x_pid.error_int = 0.0f;
        g_x_pid.out = 0.0f;
        gimbal_speed_yaw = 0.0f;
    }

    /* 仅在 Y 轴偏差超出死区时更新 Pitch 速度 */
    if ((error_y > DeadbandPx) || (error_y < -DeadbandPx))
    {
        gimbal_speed_pitch = Gimbal_GetYStep(error_y);
    }
    else
    {
        gimbal_speed_pitch = 0.0f;
    }
}

/**
 * @brief  初始化云台 PID 控制器
 * @param  无
 * @return 无
 */
void Gimbal_Init(void)
{
    /* 初始化 X 轴 PID 参数和内部状态 */
    PID_Init(&g_x_pid,
             XKp,
             XKi,
             XKd,
             -MaxStepDeg,
             MaxStepDeg,
             -MaxStepDeg,
             MaxStepDeg);

    /* X 轴闭环目标为图像宽度中心 */
    g_x_pid.target = (float)ImgWidth / 2.0f;

    /* PID 内部使用 2 像素输出死区 */
    g_x_pid.out_deadzone = (float)DeadbandPx;
}

/**
 * @brief  在采样周期到达时完成一次云台计算
 * @param  无
 * @return 无
 */
void Gimbal(void)
{
    /* 检查定时标志位 */
    if (gimbal_timer_flag == 0U)
    {
        return;
    }

    gimbal_timer_flag = 0U;
    Gimbal_GetImage();
    Gimbal_CalculateSpeed();
    gimbal_flag = 1U;
}

/**
 * @brief  获取并清除一次云台结果通知
 * @param  无
 * @return 1 表示存在新结果，0 表示没有新结果
 */
uint8_t Gimbal_GetFlag(void)
{
    if (gimbal_flag == 0U)
    {
        return 0U;
    }

    gimbal_flag = 0U;
    return 1U;
}

/**
 * @brief  获取指定轴的云台目标速度
 * @param  motor GIMBAL_MOTOR_YAW 或 GIMBAL_MOTOR_PITCH
 * @return 对应轴目标速度；参数无效时返回 0
 */
float Gimbal_GetSpeed(uint8_t motor)
{
    if (motor == GIMBAL_MOTOR_YAW)
    {
        return gimbal_speed_yaw;
    }

    if (motor == GIMBAL_MOTOR_PITCH)
    {
        return gimbal_speed_pitch;
    }

    return 0.0f;
}

/**
 * @brief  获取图像坐标
 * @param  axis 1 返回 x 坐标，2 返回 y 坐标
 * @return 对应坐标值；参数无效时返回 0
 */
uint16_t Gimbal_GetXY(uint8_t axis)
{
    if (axis == 1U)
    {
        return gimbal_x;
    }

    if (axis == 2U)
    {
        return gimbal_y;
    }

    return 0U;
}

/**
 * @brief  1 ms 定时器节拍函数，每 10 ms 请求一次云台处理
 * @param  无
 * @return 无
 */
void Gimbal_Timer(void)
{
    static uint8_t sample_count;

    sample_count++;
    if (sample_count >= GIMBAL_SAMPLE_PERIOD_MS)
    {
        sample_count = 0U;
        gimbal_timer_flag = 1U;
    }
}