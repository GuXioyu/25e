/**
 * @file Gimbal.c
 * @brief 使用 UART4 目标坐标控制二维云台。
 */

#include "Gimbal.h"

#include <stdint.h>

#include "Motor.h"
#include "My_Uart.h"
#include "PID.h"
#include "zf_common_headfile.h"

#define YawAddr       3U                  // X 轴 Yaw 电机的 EMM 驱动器地址。
#define PitchAddr     4U                  // Y 轴 Pitch 电机的 EMM 驱动器地址。
#define ImgWidth      640U                // 视觉图像宽度，单位：像素。
#define ImgHeight     360U                // 视觉图像高度，单位：像素。
#define XKp           0.02f               // X 轴 PID 比例系数，单位：度/像素。
#define XKi           0.0f                // X 轴 PID 积分系数，初始关闭积分。
#define XKd           0.0f                // X 轴 PID 微分系数，初始关闭微分。
#define YDegPerPx     0.02f               // Y 轴像素误差到角度增量的比例，单位：度/像素。
#define DeadbandPx    2                   // 绝对误差不超过该值时不动作，单位：像素。
#define MaxStepDeg    2.0f                // 两轴单帧最大角度增量，单位：度。

static Motor_t g_yaw_motor;       // X 轴 Yaw 电机控制对象。
static Motor_t g_pitch_motor;     // Y 轴 Pitch 电机控制对象。
static PID_t g_x_pid;             // X 轴图像坐标闭环 PID 状态。

/**
 * @brief 复制并消费 UART4 最新 `[x,y]` 帧，同时解析 x、y 坐标。
 * @param x 输出图像 x 坐标，范围 0~639，单位：像素。
 * @param y 输出图像 y 坐标，范围 0~359，单位：像素。
 * @return 读取到新帧返回 1U；无新帧或 UART4 无效返回 0U。
 * @note 接收层已去掉方括号，本函数按已确认的 "x,y" 内容直接解析。
 */
static uint8_t Gimbal_ReadXY(uint16_t *x, uint16_t *y)
{
    char frame[8]; // UART4 坐标帧副本，最大内容为 "639,359"。
    const char *read;             // 当前正在解析的帧字符位置。
    uint8_t uart_index;           // UART4 在串口接收数组中的索引。
    uint16_t i;                   // 坐标帧复制下标。
    uint16_t value;               // 当前累积的十进制坐标值，单位：像素。
    uint32_t primask;             // 进入临界区前保存的全局中断状态。

    uart_index = Serial_GetUartIndex(&huart4); // 获取 UART4 对应的接收缓存位置。
    if (uart_index >= SERIAL_RX_UART_COUNT)    // UART4 未注册时不能读取缓存。
    {
        return 0U;
    }

    primask = interrupt_global_disable();      // 暂停中断，避免复制过程中接收缓存变化。
    if (Serial_RxFlag[uart_index] == 0U)       // 没有完整新帧时立即恢复中断。
    {
        interrupt_global_enable(primask);
        return 0U;
    }

    for (i = 0U; i < (8 - 1U); i++) // 最多复制 7 个有效字符。
    {
        frame[i] = Serial_RxPacket[uart_index][i]; // 从共享串口缓存复制一个字符。
        if (frame[i] == '\0')                       // 字符串结束后无需继续复制。
        {
            break;
        }
    }

    frame[8 - 1U] = '\0';      // 强制补结束符，保证本地字符串完整。
    Serial_RxFlag[uart_index] = 0U;            // 当前完整帧已经由云台任务消费。
    interrupt_global_enable(primask);          // 复制结束后恢复原全局中断状态。

    read = frame;                              // 从帧首开始解析 x 坐标。
    value = 0U;                                // 清零本次十进制累积值。
    while (*read != ',')                       // 上游保证逗号分隔，因此直接读到逗号。
    {
        value = (uint16_t)(value * 10U + (uint16_t)(*read - '0')); // 逐位组合 x 坐标。
        read++;
    }
    *x = value;                                // 输出完整 x 坐标。

    read++;                                    // 跳过 x、y 之间的逗号。
    value = 0U;                                // 清零后开始累积 y 坐标。
    while (*read != '\0')                      // 读取到本地帧结束符。
    {
        value = (uint16_t)(value * 10U + (uint16_t)(*read - '0')); // 逐位组合 y 坐标。
        read++;
    }
    *y = value;                                // 输出完整 y 坐标。

    return 1U;
}

/**
 * @brief 将 Y 轴像素误差换算为受限的开环角度增量。
 * @param error_y 目标相对图像中心的 Y 轴偏差，单位：像素。
 * @return 限制在 ±MaxStepDeg 内的角度增量，单位：度。
 */
static float Gimbal_GetYStep(int16_t error_y)
{
    float step_deg; // Y 轴当前帧需要增加的目标角度，单位：度。

    step_deg = (float)error_y * YDegPerPx; // 误差正负直接决定 Y 轴相对运动方向。

    if (step_deg > MaxStepDeg)        // 正方向增量不能超过单帧上限。
    {
        step_deg = MaxStepDeg;
    }
    else if (step_deg < -MaxStepDeg)  // 负方向增量不能低于单帧下限。
    {
        step_deg = -MaxStepDeg;
    }

    return step_deg;
}

/**
 * @brief 初始化二维云台电机和 X 轴 PID 控制器。
 */
void Gimbal_Init(void)
{
    Motor_Init(&g_yaw_motor, YawAddr, 0U);       // 初始化 X 轴 Yaw 电机对象。
    Motor_Init(&g_pitch_motor, PitchAddr, 0U);   // 初始化 Y 轴 Pitch 电机对象。

    Motor_Enable(&g_yaw_motor);                         // 使能 X 轴电机驱动器。
    Motor_Enable(&g_pitch_motor);                       // 使能 Y 轴电机驱动器。

    PID_Init(&g_x_pid,                                  // 初始化 X 轴 PID 参数和内部状态。
             XKp,
             XKi,
             XKd,
             -MaxStepDeg,
             MaxStepDeg,
             -MaxStepDeg,
             MaxStepDeg);
    g_x_pid.target = (float)ImgWidth / 2.0f;             // X 轴闭环目标为图像宽度中心。
    g_x_pid.out_deadzone = (float)DeadbandPx;    // PID 内部使用 2 像素输出死区。
}

/**
 * @brief 消费 UART4 最新坐标帧，执行 X 轴 PID 闭环和 Y 轴比例开环控制。
 */
void Gimbal_Task(void)
{
    uint16_t x;       // 当前目标的图像 x 坐标，单位：像素。
    uint16_t y;       // 当前目标的图像 y 坐标，单位：像素。
    int16_t error_x;  // 目标相对图像中心的 X 轴偏差，单位：像素。
    int16_t error_y;  // 目标相对图像中心的 Y 轴偏差，单位：像素。
    float step_deg;   // 当前轴受限后的单帧角度增量，单位：度。

    if (Gimbal_ReadXY(&x, &y) == 0U)                   // 没有新帧时保持当前电机目标。
    {
        return;
    }

    if ((x == 0U) && (y == 0U))                        // 完整帧 `[0,0]` 表示当前未检测到目标。
    {
        g_x_pid.actual_current = (float)ImgWidth / 2.0f; // 将 PID 实际值恢复到图像中心。
        g_x_pid.actual_past = (float)ImgWidth / 2.0f;    // 清除后续微分计算的历史跳变。
        g_x_pid.error_current = 0.0f;                   // 清除当前 X 轴误差。
        g_x_pid.error_past = 0.0f;                      // 清除上一帧 X 轴误差。
        g_x_pid.error_int = 0.0f;                       // 清除 X 轴积分累计量。
        g_x_pid.out = 0.0f;                             // 清除 X 轴 PID 输出。
        return;                                         // 不发送两轴电机命令，保持当前位置。
    }

    error_x = (int16_t)x - (int16_t)(ImgWidth / 2U);    // 计算目标相对图像宽度中心的水平偏差。
    error_y = (int16_t)y - (int16_t)(ImgHeight / 2U);   // 计算目标相对图像高度中心的垂直偏差。

    g_x_pid.actual_current = (float)x;                  // 把当前 x 坐标作为 PID 实际值。
    PID_Update(&g_x_pid);                               // 计算中心坐标与当前 x 的闭环输出。
    if ((error_x > DeadbandPx) ||                // 仅在 X 轴偏差超出死区时发送命令。
        (error_x < -DeadbandPx))
    {
        step_deg = g_x_pid.out * -1.0f;                 // PID 使用中心减 x，直接乘 -1 得到 X 轴修正方向。
        Motor_SetRelAngle(&g_yaw_motor, step_deg);      // 直接发送本帧 X 轴相对角度增量。
    }
    else
    {
        g_x_pid.error_int = 0.0f;                       // 中心死区内清积分，避免静止时继续累积。
        g_x_pid.out = 0.0f;                             // 中心死区内明确关闭本帧 PID 输出。
    }

    if ((error_y > DeadbandPx) ||                // Y 轴只在偏差超出死区时开环调整。
        (error_y < -DeadbandPx))
    {
        step_deg = Gimbal_GetYStep(error_y);            // 按固定比例计算 Y 轴角度增量。
        Motor_SetRelAngle(&g_pitch_motor, step_deg);    // 直接发送本帧 Y 轴相对角度增量。
    }
}
