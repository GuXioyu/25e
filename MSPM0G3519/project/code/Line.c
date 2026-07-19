#include "Line.h"

/* 灰度数据处理周期，单位为毫秒。 */
#define LINE_SAMPLE_PERIOD_MS (10U)

/* 速度计算参数，速度单位由电机模块定义。 */
#define LINE_SPEED_MIN       (50)  /* 直线循迹输出的最小速度，避免修正后电机速度过低。 */
#define LINE_SPEED_MAX       (150) /* 直线循迹输出的最大速度，限制电机目标速度上限。 */
#define LINE_SPEED_BASE      (80) /* 黑线居中时使用的基础巡线速度。 */
#define LINE_SPEED_REDUCTION (24)  /* 偏离中心时基础速度的最大降低量。 */
#define LINE_SPEED_DIFFERENCE (12) /* 偏离中心时单侧轮速相对基础速度的最大补偿量。 */
#define LINE_START_FORK_FINISH_ANGLE_DEG (65.0f) /* 启动后首个左直角岔道完成判定角度阈值，单位为度。 */
#define LINE_FORK_FINISH_ANGLE_DEG       (65.0f) /* 后续左直角岔道完成判定角度阈值，单位为度。 */
#define LINE_FORK_LEFT_SPEED_LEFT  (25)    /* 左直角岔道时左轮目标速度，可根据实车调节。 */
#define LINE_FORK_LEFT_SPEED_RIGHT (80)    /* 左直角岔道时右轮目标速度，可根据实车调节。 */
#define LINE_LOST_CONFIRM_TIME_MS  (100U)  /* 连续未检测到黑线的丢线确认时间，单位为毫秒。 */
#define LINE_LOST_CONFIRM_COUNT    (LINE_LOST_CONFIRM_TIME_MS / LINE_SAMPLE_PERIOD_MS) /* 丢线确认所需的连续采样次数。 */

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
static volatile uint8_t line_timer_flag;

/* 循迹模块已完成一次数据处理的通知标志。 */
static uint8_t line_flag;

/* 当前帧的循迹状态和 8 路数字量。 */
static LineState_t line_state;                  /* 当前帧识别出的循迹状态。 */
static uint8_t line_digital[LINE_SENSOR_COUNT]; /* 8 路灰度传感器的数字量缓存。 */
static uint8_t line_number;                     /* 当前帧检测到黑线的传感器通道数量。 */
static uint8_t line_segment_count;              /* 当前帧检测到的连续黑线段数量。 */
static uint8_t line_lost_count;                 /* 连续未检测到黑线的采样次数。 */
static int8_t line_location;                    /* 当前帧黑线相对传感器中心的离散位置。 */
static LineLostLocation_t line_lost_location;   /* 最近一次有效黑线偏离方向，用于丢线恢复。 */
static LineForkType_t line_fork_type;           /* 当前识别出的岔道类型。 */
static float line_fork_start_angle;             /* 进入直角岔道时记录的起始偏航角，单位为度。 */
static uint8_t LineStartForkPending = 1U;       /* 启动后首个左直角岔道尚未锁定的标志。 */
static float LineForkFinishAngleDeg;            /* 当前已锁定左直角岔道的完成判定角度，单位为度。 */

/* 当前计算出的速度参数和左右电机目标速度。 */
static int16_t line_speed;       /* 根据偏离程度计算出的基础巡线速度。 */
static int16_t line_speed_dif;   /* 左右轮相对基础速度的单侧补偿量。 */
static int16_t line_speed_left;  /* 左电机当前目标速度。 */
static int16_t line_speed_right; /* 右电机当前目标速度。 */

/* 8 路传感器从左至右对应的离散位置。 */
static const int8_t line_sensor_position[LINE_SENSOR_COUNT] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

static float line_angle; /* 当前 HWT101 偏航角，单位为度。 */

/**
 * @brief 读取最近一次有效灰度帧，数字量查询帧由Task发送。
 * @note 灰度模块通过串口异步更新数据，因此本次使用的是已解析完成的最新帧。
 */
static void Line_ReadSensor(void)
{
    uint8_t index;

    for (index = 0U; index < LINE_SENSOR_COUNT; index++)
    {
        line_digital[index] = GraySensor_GetDigital(index);
    }

    line_angle = HWT101_GetYaw(); /* 缓存当前偏航角，用于直角岔道完成判断。 */
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
 * @brief 判断并维护岔道类型状态。
 * @param 无
 * @return 无
 */
static void Line_CheckForkType(void) /* 判断左直角岔道并在转角完成后退出岔道状态。 */
{
    float angle_diff; /* 当前角度相对进入岔道角度的差值，单位为度。 */

    if (line_state != LINE_STATE_FORK)
    {
        return;
    }

    if (line_fork_type == LINE_FORK_TYPE_IDLE)
    {
        /* 左侧前三路同时检测到黑线且右侧为白底时，立即锁定为左直角岔道。 */
        if ((line_digital[0] == 1U) && (line_digital[1] == 1U) && (line_digital[2] == 1U) &&
            (line_digital[5] == 0U) && (line_digital[6] == 0U) && (line_digital[7] == 0U))
        {
            line_fork_type = LINE_FORK_TYPE_LEFT_RIGHT_ANGLE; /* 记录左直角岔道类型。 */
            line_fork_start_angle = line_angle;                /* 记录左转起始偏航角，用于完成判定。 */
            if (LineStartForkPending != 0U) /* 启动后首个已锁定岔道使用独立完成角度。 */
            {
                LineForkFinishAngleDeg = LINE_START_FORK_FINISH_ANGLE_DEG; /* 保存首个直角弯的完成判定角度。 */
                LineStartForkPending = 0U; /* 标记首个直角弯已锁定，后续岔道恢复常规阈值。 */
            }
            else
            {
                LineForkFinishAngleDeg = LINE_FORK_FINISH_ANGLE_DEG; /* 保存后续直角弯的常规完成判定角度。 */
            }
        }
        else
        {
            line_state = LINE_STATE_STRAIGHT; /* 未识别为已支持岔道时按误识别处理，沿用最近位置继续直线循迹。 */
        }

        return;
    }

    if (line_fork_type != LINE_FORK_TYPE_LEFT_RIGHT_ANGLE)
    {
        return;
    }

    angle_diff = line_angle - line_fork_start_angle;
    if (angle_diff > 180.0f)
    {
        angle_diff -= 360.0f;
    }
    else if (angle_diff < -180.0f)
    {
        angle_diff += 360.0f;
    }

    /* 左转时偏航角按负方向变化，归一化后的差值小于等于负阈值才判定为转角完成。 */
    if (angle_diff <= -LineForkFinishAngleDeg) /* 使用当前已锁定岔道对应的完成判定角度。 */
    {
        line_state = LINE_STATE_STRAIGHT;    /* 左转完成后立即恢复直线循迹，避免当前采样周期输出零速度。 */
        line_fork_type = LINE_FORK_TYPE_IDLE; /* 清除已完成的岔道类型，允许后续重新识别岔道。 */
    }
}

/**
 * @brief 根据岔道类型设置左右轮速度。
 * @param 无
 * @return 无
 */
static void Line_SetForkSpeed(void) /* 根据当前岔道类型输出对应的左右轮速度。 */
{
    switch (line_fork_type)
    {
    case LINE_FORK_TYPE_LEFT_RIGHT_ANGLE:
        line_speed = 0;
        line_speed_dif = 0;
        line_speed_left = LINE_FORK_LEFT_SPEED_LEFT;
        line_speed_right = LINE_FORK_LEFT_SPEED_RIGHT;
        break;

    case LINE_FORK_TYPE_IDLE:
    case LINE_FORK_TYPE_RIGHT_RIGHT_ANGLE:
    default:
        Line_Stop();
        break;
    }
}

/**
 * @brief 根据有效黑线通道数量更新循迹状态和速度。
 */
static void Line_Action(void)
{
    /* 仅在空闲、直线或丢线状态下重新识别路线状态，避免直角转弯过程中反复覆盖岔道状态。 */
    if ((line_state == LINE_STATE_IDLE) || (line_state == LINE_STATE_STRAIGHT) || (line_state == LINE_STATE_LOST))
    {
        if (line_number == 0U)
        {
            /* 丢线确认期间保持上一帧的直线状态和位置，避免短时无线导致车辆停车。 */
            if (line_lost_count < LINE_LOST_CONFIRM_COUNT)
            {
                line_lost_count++; /* 累计连续无线次数，用于过滤短时丢帧。 */
            }

            if (line_lost_count >= LINE_LOST_CONFIRM_COUNT)
            {
                line_state = LINE_STATE_LOST; /* 连续无线达到 100ms 后确认丢线。 */
            }
        }
        else if ((line_number <= 2U) && (line_segment_count == 1U))
        {
            line_lost_count = 0U;          /* 检测到正常循迹线后清除丢线计数。 */
            line_state = LINE_STATE_STRAIGHT; /* 单段最多双通道时进入直线循迹状态。 */
        }
        else
        {
            line_lost_count = 0U;      /* 检测到岔道特征后清除丢线计数。 */
            line_state = LINE_STATE_FORK; /* 多通道或多线段时进入岔道处理状态。 */
        }
    }

    Line_CheckForkType();

    switch (line_state)
    {
    case LINE_STATE_STRAIGHT:
        Line_CalculateStraightSpeed();
        break;

    case LINE_STATE_FORK:
        Line_SetForkSpeed();
        break;

    case LINE_STATE_LOST:
    case LINE_STATE_IDLE:
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
    if (line_timer_flag == 0U)
    {
        return;
    }

    line_timer_flag = 0U; /* 清除本次采样请求，避免同一帧被重复处理。 */

    Line_ReadSensor();
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
 * @brief 查询当前岔道类型是否与指定类型一致
 * @param Type 待查询的岔道类型
 * @return 1 表示当前类型匹配；0 表示当前类型不匹配
 */
uint8_t Line_GetForkType(LineForkType_t Type) //查询当前岔道类型是否匹配
{
    //仅比较当前岔道类型，避免查询操作改变岔道控制状态
    if (line_fork_type == Type)
    {
        return 1U; //通知调用方当前岔道类型匹配
    }

    return 0U; //通知调用方当前岔道类型不匹配
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
        line_timer_flag = 1U;
    }
}
