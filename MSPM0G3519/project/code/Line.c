#include "Line.h"

/* 灰度数据处理周期，单位为毫秒。 */
#define LINE_SAMPLE_PERIOD_MS (10U)

/* 速度计算参数，速度单位由电机模块定义。 */
#define LINE_SPEED_MIN       (50)
#define LINE_SPEED_MAX       (150)
#define LINE_SPEED_BASE      (120)
#define LINE_SPEED_REDUCTION (40)
#define LINE_SPEED_DIFFERENCE (30)

/* 传感器位置范围为 -7～7，因此最大绝对位置为 7。 */
#define LINE_POSITION_MAX_ABS (7)

/**
 * @brief 丢线恢复可使用的最近有效偏离方向。
 * @note 该类型仅供循迹模块内部的后续搜线策略使用，不对外暴露。
 */
typedef enum
{
    LINE_LOST_LOCATION_NONE = 0, /* 尚未获得可用于判断方向的循迹数据。 */
    LINE_LOST_LOCATION_LEFT,     /* 黑线最近位于传感器左侧。 */
    LINE_LOST_LOCATION_RIGHT     /* 黑线最近位于传感器右侧。 */
} LineLostLocation_t;

/* 定时器中断置位、主循环读取的采样请求标志。 */
static volatile uint8_t gray_timer_flag;

/* 循迹模块已完成一次数据处理的通知标志。 */
static uint8_t line_flag;

/* 当前帧的循迹状态和 8 路数字量。 */
static LineMode_t line_mode;
static uint8_t line_digital[LINE_SENSOR_COUNT];
static uint8_t line_number;
static uint8_t line_segment_count;
static int8_t line_location;
static LineLostLocation_t line_lost_location;

/* 当前计算出的速度参数和左右电机目标速度。 */
static int16_t line_speed;
static int16_t line_speed_dif;
static int16_t line_speed_left;
static int16_t line_speed_right;

/* 8 路传感器从左至右对应的离散位置。 */
static const int8_t line_sensor_position[LINE_SENSOR_COUNT] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

/**
 * @brief 读取最近一次有效灰度帧，并发起下一次数字量查询。
 * @note 灰度模块通过串口异步更新数据，因此本次使用的是已解析完成的最新帧。
 */
static void Line_GetGray(void)
{
    uint8_t index;

    for (index = 0U; index < LINE_SENSOR_COUNT; index++)
    {
        line_digital[index] = GraySensor_GetDigital(index);
    }

    /* 发起下一帧查询，不在此处等待串口返回，避免阻塞主循环。 */
    GraySensor_SendQuery(&huart1, GRAYSENSOR_QUERY_DIGITAL);
}

/**
 * @brief 统计黑线通道，并计算黑线在传感器阵列中的平均位置。
 */
static void Line_GetLineState(void)
{
    uint8_t index;
    uint8_t previous_active = 0U;
    int16_t line_position_sum = 0;

    line_number = 0U;
    line_segment_count = 0U;
    for (index = 0U; index < LINE_SENSOR_COUNT; index++)
    {
        if (line_digital[index] != 0U)
        {
            line_number++;
            line_position_sum += line_sensor_position[index];

            /* 新的连续黑线段开始，用于排除分离噪声和断线。 */
            if (previous_active == 0U)
            {
                line_segment_count++;
            }

            previous_active = 1U;
        }
        else
        {
            previous_active = 0U;
        }
    }

    if (line_number == 0U)
    {
        return;
    }

    /* 相邻双通道会自然得到中间位置，例如通道 0、1 对应 -6。 */
    line_location = (int8_t)(line_position_sum / (int16_t)line_number);

    /*
     * 仅使用单段、最多双通道的正常循迹数据更新丢线方向。
     * 丢线、岔道和离散噪声都不会覆盖最近一次可信方向。
     */
    if ((line_segment_count == 1U) && (line_number <= 2U))
    {
        if (line_location < 0)
        {
            line_lost_location = LINE_LOST_LOCATION_LEFT;
        }
        else if (line_location > 0)
        {
            line_lost_location = LINE_LOST_LOCATION_RIGHT;
        }
    }
}

/**
 * @brief 将直线循迹速度限制在允许范围内。
 * @param speed 待限制的目标速度。
 * @return 限制后的速度。
 */
static int16_t Line_LimitSpeed(int16_t speed)
{
    if (speed > LINE_SPEED_MAX)
    {
        return LINE_SPEED_MAX;
    }

    if (speed < LINE_SPEED_MIN)
    {
        return LINE_SPEED_MIN;
    }

    return speed;
}

/**
 * @brief 根据黑线位置计算左右轮速度。
 */
static void Line_CalculateStraightSpeed(void)
{
    int16_t location_abs;

    location_abs = (line_location < 0) ? -(int16_t)line_location : (int16_t)line_location;
    /* 偏差越大，基础速度越低，避免进入弯道时继续加速。 */
    line_speed = LINE_SPEED_BASE - (LINE_SPEED_REDUCTION * location_abs) / LINE_POSITION_MAX_ABS;
    line_speed_dif = (LINE_SPEED_DIFFERENCE * location_abs) / LINE_POSITION_MAX_ABS;

    /* 黑线在左侧时左轮减速、右轮加速；右侧情况相反。 */
    if (line_location < 0)
    {
        line_speed_left = Line_LimitSpeed(line_speed - line_speed_dif);
        line_speed_right = Line_LimitSpeed(line_speed + line_speed_dif);
    }
    else
    {
        line_speed_left = Line_LimitSpeed(line_speed + line_speed_dif);
        line_speed_right = Line_LimitSpeed(line_speed - line_speed_dif);
    }
}

/**
 * @brief 停止输出循迹速度，避免丢线或岔道时沿用上一帧速度。
 */
static void Line_Stop(void)
{
    line_speed = 0;
    line_speed_dif = 0;
    line_speed_left = 0;
    line_speed_right = 0;
}

/**
 * @brief 根据有效黑线通道数量更新循迹状态和速度。
 */
static void Line_Action(void)
{
    if (line_number == 0U)
    {
        line_mode = LINE_MODE_LOST;
    }
    else if ((line_number <= 2U) && (line_segment_count == 1U))
    {
        line_mode = LINE_MODE_STRAIGHT;
    }
    else
    {
        line_mode = LINE_MODE_FORK;
    }

    switch (line_mode)
    {
    case LINE_MODE_STRAIGHT:
        Line_CalculateStraightSpeed();
        break;

    case LINE_MODE_LOST:
    case LINE_MODE_FORK:
    case LINE_MODE_IDLE:
    default:
        /* 当前模块没有丢线或岔道策略，安全地输出零速度。 */
        Line_Stop();
        break;
    }
}

/**
 * @brief 在采样周期到达时完成一次循迹计算。
 */
void Line(void)
{
    if (gray_timer_flag == 0U)
    {
        return;
    }

    gray_timer_flag = 0U;
    Line_GetGray();
    Line_GetLineState();
    Line_Action();
    line_flag = 1U;
}

/**
 * @brief 获取并清除一次循迹结果通知。
 */
uint8_t Line_GetFlag(void)
{
    if (line_flag == 0U)
    {
        return 0U;
    }

    line_flag = 0U;
    return 1U;
}

/**
 * @brief 获取指定电机的循迹目标速度。
 */
int16_t Line_GetSpeed(uint8_t motor)
{
    if (motor == LINE_MOTOR_LEFT)
    {
        return line_speed_left;
    }

    if (motor == LINE_MOTOR_RIGHT)
    {
        return line_speed_right;
    }

    return 0;
}

/**
 * @brief 1 ms 定时器节拍函数，每 10 ms 请求一次循迹处理。
 */
void Line_Timer(void)
{
    static uint8_t sample_count;

    sample_count++;
    if (sample_count >= LINE_SAMPLE_PERIOD_MS)
    {
        sample_count = 0U;
        gray_timer_flag = 1U;
    }
}
