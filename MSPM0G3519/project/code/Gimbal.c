/**
 * @file Gimbal.c
 * @brief 使用 UART4 目标坐标计算云台目标速度。
 */

#include "Gimbal.h"
#include "zf_common_headfile.h"
#include <stdint.h>
#include "My_Uart.h"
#include "PID.h"
#include "Cam.h"
#include "HWT101.h"
#include "Line.h"

/* 云台数据处理周期，单位为毫秒 */
#define GIMBAL_TIMER (10U)

/* 图像参数 */
#define ImgWidth      640U                /* 视觉图像宽度，单位：像素 */
#define ImgHeight     480U                /* 视觉图像高度，单位：像素 */

/* 控制参数 */
#define GIMBAL_AUTO_SPEED			12U
#define GIMBAL_AUTO_ANGLE1			5.0f
#define GIMBAL_AUTO_ANGLE2			1.0f
#define GIMBAL_AUTO_ANGLE3			0.1f
#define GIMBAL_AUTO_ANGLE4			0.01f
#define GIMBAL_YAW_ERROR_OK			4U // Yaw 瞄准完成允许误差，单位：像素
#define GIMBAL_PITCH_START_ERROR	5U // Pitch 启动微调阈值，单位：像素
#define GIMBAL_PITCH_STOP_ERROR		3U // Pitch 停止微调阈值，单位：像素

/* 定时器中断置位、主循环读取的采样请求标志 */
uint8_t gimbal_aim1_timer_flag;
uint8_t gimbal_aim2_timer_flag;
uint8_t gimbal_aim3_timer_flag;
uint8_t gimbal_Tx_timer_flag;


/* 云台模块已完成一次数据处理的通知标志 */
uint8_t gimbal_flag;

/* 当前计算出的云台两轴目标速度 */
int16 gimbal_speed_yaw;
int16 gimbal_speed_pitch;
float gimbal_angle_yaw;
float gimbal_angle_pitch;

/* 当前帧的图像坐标 */
uint16_t gimbal_target_x;
uint16_t gimbal_target_y;
uint16_t gimbal_laser_x;
uint16_t gimbal_laser_y;

static uint8_t gimbal_aim_state = 0U; // 定点瞄准状态机当前状态
static uint8_t gimbal_aim_detect_count = 0U; // 连续发现目标的有效帧数
static uint8_t gimbal_aim_yaw_stable_count = 0U; // Yaw 轴进入允许误差的连续帧数
static uint8_t gimbal_aim_pitch_stable_count = 0U; // Pitch 轴进入允许误差的连续帧数
static uint8_t gimbal_aim_yaw_locked = 0U; // Yaw 轴是否已稳定瞄准
static uint8_t gimbal_aim_pitch_locked = 0U; // Pitch 轴是否已稳定瞄准
static uint8_t gimbal_pitch_adjust_enabled = 0U; // Pitch 是否处于微调状态，用于双阈值防抖


/* X 轴图像坐标闭环 PID 状态 */
PID_t gimbal_aim2_x_straight_PID = 
{
	.kf = 0.15,
	.kp = 0.15,
	.ki = 0.010,
	
	.mode_f = PID_FMODE_ENABLE,
	.mode_p = PID_PMODE_ENABLE,
	.mode_i = PID_IMODE_NORMAL,
	.mode_d = PID_DMODE_DISABLE,
	
	.error_intmax = 100,
	.error_intmin = 100,
	
	.outmax = 10,
	.outmin = -10,
};  
PID_t gimbal_aim2_x_fork_PID = 
{
	.kp = 0.1,
	.ki = 0.001,
	.kd = 0.1,
	
	.mode_f = PID_FMODE_DISABLE,
	.mode_p = PID_PMODE_ENABLE,
	.mode_i = PID_IMODE_NORMAL,
	.mode_d = PID_DMODE_NORMAL,
	
	.error_intmax = 100,
	.error_intmin = 100,
	
	.outmax = 10,
	.outmin = -10,
};
PID_t gimbal_aim1_y_PID;



/* ==================== 定点瞄准控制 ==================== */
/**
 * @brief 执行定点瞄准状态机
 * @param mode 瞄准模式：1 直接瞄准；2 左向扫描；3 右向扫描
 * @return 1 表示两轴瞄准完成；0 表示仍在瞄准流程中
 */
uint8_t Gimbal_Aim1(uint8_t mode) // 执行定点瞄准状态机
{
	uint16_t GimbalXError; // 保存当前帧 X 轴图像误差，单位：像素
	uint16_t GimbalYError; // 保存当前帧 Y 轴图像误差，单位：像素
	
	if (gimbal_aim1_timer_flag == 0)return 0;		//定时
    gimbal_aim1_timer_flag = 0;
	
    switch (gimbal_aim_state)
    {
		case 0U:// 转一圈
			gimbal_pitch_adjust_enabled = 0U; // 新一轮瞄准开始时清除 Pitch 微调状态
			if (mode == 1)
				gimbal_aim_state = 20U;
			else if (mode == 2)
				gimbal_aim_state = 10U;
			else if (mode == 3)
				gimbal_aim_state = 11U;
			break;
        case 10:// 左转一圈
            gimbal_speed_yaw = -1.0 * GIMBAL_AUTO_SPEED;
            gimbal_speed_pitch = 0.0f;
            gimbal_flag = 1U;
			gimbal_aim_state = 19U;
            break;
		case 11:// 右转一圈
            gimbal_speed_yaw = 1.0 * GIMBAL_AUTO_SPEED;
            gimbal_speed_pitch = 0.0f;
            gimbal_flag = 1U;
			gimbal_aim_state = 19U;
            break;
        case 19: //寻找靶子
			if (Cam_GetFlag() == 0)break;			//未发现靶子时保持当前扫描速度
			gimbal_aim_detect_count++;				//累计连续发现靶子的有效帧数
			if (gimbal_aim_detect_count >= 5U)			//连续有5帧数据，则发现目标
			{
				gimbal_aim_detect_count = 0U;
				gimbal_speed_yaw = 0;
				gimbal_speed_pitch = 0;
				gimbal_flag = 0;
				gimbal_aim_state = 20U;
			}
			break;
		case 20: //瞄准
			if (Cam_GetFlag() == 0)break;				//没有新图像时保持上次云台指令
			GimbalXError = Gimbal_GetAbsError(gimbal_target_x, gimbal_laser_x); // 计算本帧 X 轴误差
			GimbalYError = Gimbal_GetAbsError(gimbal_target_y, gimbal_laser_y); // 计算本帧 Y 轴误差
			gimbal_angle_yaw = Gimbal_GetStepAngle(gimbal_target_x, gimbal_laser_x, 1U); // Yaw 保持原有最小步进策略
			if (GimbalYError > GIMBAL_PITCH_START_ERROR)
			{
				gimbal_pitch_adjust_enabled = 1U; // 误差超过 4 像素后启动 Pitch 微调
			}
			else if (GimbalYError <= GIMBAL_PITCH_STOP_ERROR)
			{
				gimbal_pitch_adjust_enabled = 0U; // 误差回到 2 像素内后停止 Pitch 微调
			}

			gimbal_angle_pitch = Gimbal_GetStepAngle(gimbal_target_y, gimbal_laser_y, gimbal_pitch_adjust_enabled); // 按 Pitch 状态生成步进角度
			Gimbal_UpdateAimOk(GimbalXError, GIMBAL_YAW_ERROR_OK, &gimbal_aim_yaw_stable_count, &gimbal_aim_yaw_locked); // 按 X 轴允许误差更新稳定帧状态
			Gimbal_UpdateAimOk(GimbalYError, GIMBAL_PITCH_STOP_ERROR, &gimbal_aim_pitch_stable_count, &gimbal_aim_pitch_locked); // 按 Pitch 停止阈值更新稳定帧状态
			
			if (gimbal_aim_yaw_locked == 1U)gimbal_angle_yaw = 0.0f;
			if (gimbal_aim_pitch_locked == 1U)gimbal_angle_pitch = 0.0f;
			if ((gimbal_aim_yaw_locked == 1U) && (gimbal_aim_pitch_locked == 1U))
			{
				gimbal_aim_state = 21U;
			}
			gimbal_flag = 2;
			break;
		case 21://fire
			gimbal_aim_yaw_locked = 0U;
			gimbal_aim_pitch_locked = 0U;
			gimbal_pitch_adjust_enabled = 0U;
			gimbal_aim_state = 0U;
			return 1;
    }
	return 0;
}

//			gimbal_target_x
//			gimbal_target_y
//			gimbal_laser_x
//			gimbal_laser_y
/**
 * @brief 执行运动过程中的 Yaw 瞄准控制
 * @param 无
 * @return 无
 */
void Gimbal_Aim2(void) // 执行动态 Yaw 瞄准控制
{
	static uint8_t gimbal_line_state; 			//状态机
	static float gimbal_gyroz;
	
	if (gimbal_aim2_timer_flag == 0)return;		//定时
    gimbal_aim2_timer_flag = 0;
	if (Cam_GetFlag() == 0)return;				//new
	
	//gimbal_line_state = Line_GetForkType(LINE_FORK_TYPE_LEFT_RIGHT_ANGLE);
	gimbal_line_state = 0;
	switch (gimbal_line_state)
	{
		case (0)://直线
			gimbal_aim2_x_straight_PID.target = (float)gimbal_target_x;
			gimbal_aim2_x_straight_PID.actual_current = (float)gimbal_laser_x;
			PID_Update(&gimbal_aim2_x_straight_PID);
			gimbal_speed_yaw = (int16_t)(gimbal_aim2_x_straight_PID.out);
			gimbal_flag = 1;
			break;
		case (1)://转弯
			gimbal_gyroz = HWT101_GetGyroZ();
			gimbal_aim2_x_fork_PID.target = (float)gimbal_target_x;
			gimbal_aim2_x_fork_PID.actual_current = (float)gimbal_laser_x;
			PID_Update(&gimbal_aim2_x_fork_PID);
			gimbal_speed_yaw = (int16_t)(-gimbal_gyroz / 6 + gimbal_aim2_x_fork_PID.out);
			gimbal_flag = 1;
			break;
	}
}

/**
 * @brief 按陀螺仪 Z 轴角速度补偿 Yaw 速度
 * @param 无
 * @return 无
 */
void Gimbal_tast(void) // 执行 Yaw 角速度补偿测试
{
	static float gimbal_gyroz;
	if (gimbal_aim2_timer_flag == 0)return;		//定时
    gimbal_aim2_timer_flag = 0;
	gimbal_gyroz = HWT101_GetGyroZ();
	gimbal_speed_yaw = (int16_t)(-gimbal_gyroz / 6);
	gimbal_flag = 1;
}

/**
 * @brief 执行预留的动态瞄准控制
 * @param 无
 * @return 无
 */
void Gimbal_Aim3(void) // 执行预留的动态瞄准控制
{
	static uint8_t gimbal_line_state; 			//状态机
	
	if (gimbal_aim3_timer_flag == 0)return;		//定时
    gimbal_aim3_timer_flag = 0;
	if (Cam_GetFlag() == 0)return;				//new
	
	gimbal_line_state = Line_GetForkType(LINE_FORK_TYPE_LEFT_RIGHT_ANGLE);
	
	switch (gimbal_line_state)
	{
		case (0)://直线
		
		
			break;
		case (1)://转弯
				
		
			break;
	}
}

/* ==================== 控制结果读取接口 ==================== */
/**
 * @brief  获取并清除一次云台结果通知
 * @param  无
 * @return 1 表示存在新结果，0 表示没有新结果
 */
uint8_t Gimbal_GetFlag(void)
{
	uint8_t temp_flag = gimbal_flag;
	gimbal_flag = 0;
    return temp_flag;
}

/**
 * @brief  获取指定轴的云台目标速度
 * @param  motor GIMBAL_MOTOR_YAW 或 GIMBAL_MOTOR_PITCH
 * @return 对应轴目标速度；参数无效时返回 0
 */
float Gimbal_GetSpeed(uint8_t type) // 读取指定云台控制量
{
    if (type == GIMBAL_SPEED_YAW)
        return gimbal_speed_yaw;
    else if (type == GIMBAL_SPEED_PITCH)
        return gimbal_speed_pitch;
	else if (type == GIMBAL_ANGLE_YAW)
        return gimbal_angle_yaw;
	else if (type == GIMBAL_ANGLE_PITCH)
        return gimbal_angle_pitch;
	
    return 0.0f;
}
/* ==================== Timer ==================== */
/**
 * @brief  1 ms 定时器节拍函数，每 10 ms 请求一次云台处理
 * @param  无
 * @return 无
 */
void Gimbal_Timer(void) // 产生云台控制周期标志
{
    static uint8_t sample_count;
	static uint8_t count_100ms;
    sample_count++;
	count_100ms++;
    if (sample_count >= GIMBAL_TIMER)
    {
        sample_count = 0U;
		gimbal_aim1_timer_flag = 1U;
		gimbal_aim2_timer_flag = 1U;
		gimbal_aim3_timer_flag = 1U;
    }
	if (count_100ms >= 100)
	{
		count_100ms = 0;
		gimbal_Tx_timer_flag = 1;
	}
	
}

/* ==================== Child ==================== */
/**
 * @brief 计算两个图像坐标的绝对误差
 * @param Target 目标图像坐标，单位：像素
 * @param Actual 当前激光图像坐标，单位：像素
 * @return 两坐标的绝对误差，单位：像素
 */
static uint16_t Gimbal_GetAbsError(uint16_t Target, uint16_t Actual) // 计算图像坐标绝对误差
{
	if (Target >= Actual) // 按较大坐标减较小坐标，避免无符号减法下溢
	{
		return Target - Actual;
	}

	return Actual - Target; // 返回另一方向的正误差
}

/**
 * @brief 根据图像误差生成单次相对转角
 * @param Target 目标图像坐标，单位：像素
 * @param Actual 当前激光图像坐标，单位：像素
 * @param FineAdjustEnable 1 表示允许最小步进；0 表示禁止最小步进
 * @return 带方向的相对转角，单位：度
 */
static float Gimbal_GetStepAngle(uint16_t Target, uint16_t Actual, uint8_t FineAdjustEnable) // 生成图像误差对应的相对转角
{
	uint16_t Error = Gimbal_GetAbsError(Target, Actual); // 获取本轴当前图像误差
	float StepAngle = 0.0f; // 保存本次无方向步进角度，单位：度

	if (Error > 150U) // 大误差时执行粗调
	{
		StepAngle = GIMBAL_AUTO_ANGLE1;
	}
	else if (Error > 50) // 中等误差时执行中调
	{
		StepAngle = GIMBAL_AUTO_ANGLE2;
	}
	else if (Error > 5U) // 中等误差时执行中调
	{
		StepAngle = GIMBAL_AUTO_ANGLE3;
	}
	else if ((Error > 0U) && (FineAdjustEnable == 1U)) // 允许时才执行最小步进
	{
		StepAngle = GIMBAL_AUTO_ANGLE4;
	}

	if (StepAngle == 0.0f) // 无需调整时返回零角度
	{
		return 0.0f;
	}

	return (Target > Actual) ? StepAngle : -StepAngle; // 根据误差方向返回带符号角度
}

/**
 * @brief 更新单轴连续稳定帧判定
 * @param Error 当前轴图像误差，单位：像素
 * @param Count 连续稳定帧计数器指针
 * @param Ok 单轴瞄准完成标志指针
 * @return 无
 */
static void Gimbal_UpdateAimOk(uint16_t Error, uint16_t ErrorOk, uint8_t *Count, uint8_t *Ok) // 更新单轴瞄准稳定状态
{
	if (Error <= ErrorOk) // 误差进入本轴允许范围时累计稳定帧
	{
		(*Count)++;
	}
	else
	{
		*Count = 0U; // 误差超限时重新计数
	}

	if (*Count >= 5U) // 连续五帧稳定后锁定该轴
	{
		*Count = 0U;
		*Ok = 1U;
	}
}

/**
 * @brief  
 * @note   
 */
void Gimbal_GetImage(uint16_t target_x, uint16_t target_y, uint16_t laser_x, uint16_t laser_y)
{
	gimbal_target_x = target_x;
	gimbal_target_y = target_y;
	
	gimbal_laser_x = laser_x;
	gimbal_laser_y = laser_y;
}

/**
 * @brief 复位定点瞄准状态机和云台输出
 * @param 无
 * @return 无
 */
void Gimbal_ResetAim1(void) // 复位定点瞄准状态机
{
	gimbal_aim_state = 0U; // 使下次瞄准从初始扫描状态开始
	gimbal_aim_detect_count = 0U;
	gimbal_aim_yaw_stable_count = 0U;
	gimbal_aim_pitch_stable_count = 0U;
	gimbal_aim_yaw_locked = 0U;
	gimbal_aim_pitch_locked = 0U;
	gimbal_pitch_adjust_enabled = 0U;
	gimbal_speed_yaw = 0;
	gimbal_speed_pitch = 0;
	gimbal_angle_yaw = 0.0f;
	gimbal_angle_pitch = 0.0f;
	gimbal_flag = 1U; // 请求电机任务发送停止速度
}
