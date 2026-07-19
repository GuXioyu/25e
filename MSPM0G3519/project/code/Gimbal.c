/**
 * @file Gimbal.c
 * @brief 使用 UART4 目标坐标计算云台目标速度。
 */

#include "Gimbal.h"

#include <stdint.h>

#include "My_Uart.h"
#include "PID.h"
#include "zf_common_headfile.h"
#include "Cam.h"
#include "laser.h"

/* 云台数据处理周期，单位为毫秒 */
#define GIMBAL_TIMER (10U)

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
uint8_t gimbal_aim1_timer_flag;
uint8_t gimbal_aim2_timer_flag;
uint8_t gimbal_aim3_timer_flag;


/* 云台模块已完成一次数据处理的通知标志 */
uint8_t gimbal_flag;

/* 当前计算出的云台两轴目标速度 */
float gimbal_speed_yaw;
float gimbal_speed_pitch;

/* 当前帧的图像坐标 */
uint16_t gimbal_target_x;
uint16_t gimbal_target_y;
uint16_t gimbal_laser_x;
uint16_t gimbal_laser_y;


/* X 轴图像坐标闭环 PID 状态 */
PID_t gimbal_aim1_x_PID = 
{
	.kp = 0.1,
	.ki = 0.001,
	.kd = 0.1,
	
	.target = ImgWidth/2.0,
	
	.intmode = PID_INTMODE_DATA_NORMAL | PID_INTMODE_LIMIT,
	.error_intmax = 100,
	.error_intmin = 100,
	
	.difmode = PID_DIFMODE_NORMAL,
	
	.outmax = 10,
	.outmin = 10,
	
	 
};  
PID_t gimbal_aim1_y_PID;

float aim_x_kp, aim_x_ki, aim_x_kd;
float aim_x_kp, aim_x_ki, aim_x_kd;

PID_t g_x_pid;

/**
 * @brief  
 * @note   
 */
uint8_t Gimbal_GetImage(void)
{
	if (Cam_GetFlag() == 0)return 0;
	else 
	{
		gimbal_target_x = Cam_GetXY(CAM_X, CAM_TARGERT);
		gimbal_target_y = Cam_GetXY(CAM_Y, CAM_TARGERT);
		gimbal_laser_x = Cam_GetXY(CAM_X, CAM_LASER);
		gimbal_laser_y = Cam_GetXY(CAM_Y, CAM_LASER);
		return 1;
	}
}

///**
// * @brief  将 Y 轴像素误差换算为受限的开环角度增量
// * @param  error_y 目标相对图像中心的 Y 轴偏差，单位：像素
// * @return 限制在 ±MaxStepDeg 内的角度增量，单位：度
// */
//static float Gimbal_GetYStep(int16_t error_y)
//{
//    float step_deg;                         /* Y 轴当前帧需要增加的目标角度，单位：度 */

//    step_deg = (float)error_y * YDegPerPx;

//    /* 限制角度增量范围 */
//    if (step_deg > MaxStepDeg)
//    {
//        step_deg = MaxStepDeg;
//    }
//    else if (step_deg < -MaxStepDeg)
//    {
//        step_deg = -MaxStepDeg;
//    }

//    return step_deg;
//}

///**
// * @brief  根据图像坐标计算云台两轴目标速度
// */
//static void Gimbal_CalculateSpeed(void)
//{
//    int16_t error_x;                        /* 目标相对图像中心的 X 轴偏差，单位：像素 */
//    int16_t error_y;                        /* 目标相对图像中心的 Y 轴偏差，单位：像素 */

//    /* 没有有效图像数据时，清零速度 */
//    if (gimbal_image_valid == 0U)
//    {
//        gimbal_speed_yaw = 0.0f;
//        gimbal_speed_pitch = 0.0f;
//        return;
//    }

//    /* 未检测到目标时清零速度和 PID 状态 */
//    if ((gimbal_x == 0U) && (gimbal_y == 0U))
//    {
//        g_x_pid.actual_current = (float)ImgWidth / 2.0f;
//        g_x_pid.actual_past = (float)ImgWidth / 2.0f;
//        g_x_pid.error_current = 0.0f;
//        g_x_pid.error_past = 0.0f;
//        g_x_pid.error_int = 0.0f;
//        g_x_pid.out = 0.0f;
//        gimbal_speed_yaw = 0.0f;
//        gimbal_speed_pitch = 0.0f;
//        return;
//    }

//    /* 计算目标相对图像中心的偏差 */
//    error_x = (int16_t)gimbal_x - (int16_t)(ImgWidth / 2U);
//    error_y = (int16_t)gimbal_y - (int16_t)(ImgHeight / 2U);

//    /* X 轴 PID 闭环计算 */
//    g_x_pid.actual_current = (float)gimbal_x;
//    PID_Update(&g_x_pid);

//    /* 仅在 X 轴偏差超出死区时更新 Yaw 速度 */
//    if ((error_x > DeadbandPx) || (error_x < -DeadbandPx))
//    {
//        gimbal_speed_yaw = g_x_pid.out * -1.0f;
//    }
//    else
//    {
//        g_x_pid.error_int = 0.0f;
//        g_x_pid.out = 0.0f;
//        gimbal_speed_yaw = 0.0f;
//    }

//    /* 仅在 Y 轴偏差超出死区时更新 Pitch 速度 */
//    if ((error_y > DeadbandPx) || (error_y < -DeadbandPx))
//    {
//        gimbal_speed_pitch = Gimbal_GetYStep(error_y);
//    }
//    else
//    {
//        gimbal_speed_pitch = 0.0f;
//    }
//}



/* ==================== Parent ==================== */
uint8_t Gimbal_Aim1(uint8_t mode)// 定->定
{
	static uint8_t gimbal_aim_state = 0U; 			//状态机
	static uint8_t gimbal_aim_find_count = 0U;		//发现目标计数器
	static uint8_t gimbal_aim_error_count = 0U;		//允许误差计数器
	
	if (gimbal_aim1_timer_flag == 0)return 0;		//定时
    gimbal_aim1_timer_flag = 0;

    switch (gimbal_aim_state)
    {
		 case 0U:// 转一圈
			if (mode == 1)
				gimbal_aim_state = 100;
			else if (mode == 2)
				gimbal_aim_state = 1;
        case 100U:// 转一圈
            gimbal_speed_yaw = 20.0f;
            gimbal_speed_pitch = 0.0f;
            gimbal_flag = 1U;
            gimbal_aim_state = 1U;
            break;
        case 1U: //寻找靶子
			if (Gimbal_GetImage() == 1)
				gimbal_aim_find_count++;
			else 
				gimbal_aim_find_count = 0;
			if (gimbal_aim_find_count >= 5) //连续有5帧数据，则发现目标
			{
				gimbal_aim_find_count = 0;
				gimbal_aim_state = 2;
			}
			break;
		case 2: //瞄准
			gimbal_aim1_x_PID.actual_current = gimbal_laser_x;
			PID_Update(&gimbal_aim1_x_PID);
			gimbal_speed_yaw = gimbal_aim1_x_PID.out;
			gimbal_flag = 1;
			//瞄准成功判定
			if (gimbal_aim1_x_PID.error_current <= 5)//允许的误差
				gimbal_aim_error_count++;
			else 
				gimbal_aim_error_count = 0;
			if (gimbal_aim_error_count >= 5)
			{
				gimbal_aim_error_count = 0;
				gimbal_aim_state = 3;
			}
			break;
		case 3://fire
			gimbal_aim_state = 0;
			return 1;
    }
	return 0;
}
void Gimbal_Aim2(void)// 动->定
{
	if (gimbal_aim2_timer_flag == 0)return;		//定时
    gimbal_aim2_timer_flag = 0;
	
	if(0)//直线
	{
		
	}
	
	else if (0)//转弯
	{
		
	}
}
void Gimbal_Aim3(void)// 动->动
{
	if (gimbal_aim2_timer_flag == 0)return;		//定时
    gimbal_aim2_timer_flag = 0;
	
	if(0)//直线
	{
		
	}
	else if (0)//转弯 
	{
		
	}
	
}

/* ==================== 通信 ==================== */
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
/* ==================== Init ==================== */
/**
 * @brief  初始化云台 PID 控制器
 * @param  无
 * @return 无
 */
//void Gimbal_Init(void)
//{
//    /* 初始化 X 轴 PID 参数和内部状态 */
//    PID_Init(&g_x_pid, 
//             XKp,XKi,XKd,
//             -MaxStepDeg,
//             MaxStepDeg,
//             -MaxStepDeg,
//             MaxStepDeg);

//    /* X 轴闭环目标为图像宽度中心 */
//    g_x_pid.target = (float)ImgWidth / 2.0f;

//    /* PID 内部使用 2 像素输出死区 */
//    g_x_pid.out_deadzone = (float)DeadbandPx;
//}

/* ==================== Timer ==================== */
/**
 * @brief  1 ms 定时器节拍函数，每 10 ms 请求一次云台处理
 * @param  无
 * @return 无
 */
void Gimbal_Timer(void)
{
    static uint8_t sample_count;

    sample_count++;
    if (sample_count >= GIMBAL_TIMER)
    {
        sample_count = 0U;
		gimbal_aim1_timer_flag = 1U;
		gimbal_aim2_timer_flag = 1U;
		gimbal_aim3_timer_flag = 1U;
    }
}
