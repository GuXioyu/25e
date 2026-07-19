#include "Task.h"

extern uint16_t tick_ms;
extern uint8_t mode;

uint8_t mode1_N;					//A1：目标圈数
uint8_t mode1_line_flag;	
uint8_t mode2;
uint8_t mode2_gimbal_flag;
uint8_t mode3;					
uint8_t mode3_N;					//B:目标圈数
uint8_t mode3_line_flag;
uint8_t mode3_gimbal_flag;

uint8_t  OLED_timer_flag, screen_Tx_timer_flag,  BLE_Tx_timer_flag;// 定时标志位
uint8_t  motor_timer_flag, sensor_timer_flag;	

uint8_t motor_flag[4]; 												// 动作标志位
uint8_t stop_flag;
uint8_t para_flag;

Motor_t motor_all;
Motor_t motor_left, motor_right, motor_yaw, motor_pitch;

int16_t ang[4];
int16_t speed_left, speed_right, speed_yaw, speed_pitch;
float angle_yaw, angle_pitch;			// 云台角度目标，保留 Gimbal_GetSpeed 的浮点结果

uint16_t gray_analog[8];	// 灰度数据
uint8_t  gray_digital[8];

uint16_t target_x,target_y;
uint16_t laser_x,laser_y;

int8_t laps;		//	已转圈数

float angle;		// hwt101

/* ==================== Timer ==================== */
/**
 * @brief  定时器任务，产生不同周期的定时标志位
 * @param  无
 * @return 无
 */
void Task_Timer(void)
{
	static uint8_t count_1ms, count_10ms, count_100ms;

	// 计数器自增
	tick_ms++;
	count_1ms++;
	count_10ms++;
	count_100ms++;

	// 1ms定时分频
	if (count_1ms >= 1U)
	{
		count_1ms = 0U;
		motor_timer_flag = 1;
		sensor_timer_flag = 1;
	}
	// 10ms定时分频（预留）
	if (count_10ms >= 10U)
	{
		count_10ms = 0U;
	}
	// 100ms定时分频
	if (count_100ms >= 100U)
	{
		count_100ms = 0U;
		OLED_timer_flag = 1;
		BLE_Tx_timer_flag = 1;
		screen_Tx_timer_flag = 1;
	}
	Line_Timer();
	Gimbal_Timer();
}

/* ==================== 通信 ==================== */
/**
 * @brief  OLED界面显示任务，显示系统时间和灰度传感器数据
 * @param  无
 * @return 无
 */
void Task_OLED_UI(void)
{
	// 检查定时标志位
	if (OLED_timer_flag == 0)return;
	OLED_timer_flag = 0;

	// 显示系统时间
	OLED_ShowNum(0,		0, 	tick_ms, 			5, OLED_8X16);
	// 显示灰度传感器模拟值
	OLED_ShowNum(0, 	16, gray_analog[0],		5, OLED_6X8);
	OLED_ShowNum(30, 	16, gray_analog[1], 	5, OLED_6X8);
	OLED_ShowNum(60, 	16, gray_analog[2], 	5, OLED_6X8);
	OLED_ShowNum(90, 	16, gray_analog[3], 	5, OLED_6X8);
	OLED_ShowNum(0, 	24, gray_analog[4], 	5, OLED_6X8);
	OLED_ShowNum(30, 	24, gray_analog[5], 	5, OLED_6X8);
	OLED_ShowNum(60, 	24, gray_analog[6], 	5, OLED_6X8);
	OLED_ShowNum(90, 	24, gray_analog[7], 	5, OLED_6X8);
	// 更新OLED显示
	OLED_Update();
}

/**
 * @brief  蓝牙数据发送任务，通过串口发送系统时间和传感器数据
 * @param  无
 * @return 无
 */
void Task_BLE_Tx(void)
{
	// 检查定时标志位
	if (BLE_Tx_timer_flag == 0)return;
	BLE_Tx_timer_flag = 0;

//	// 发送姿态角数据
	Serial_Printf(&huart5, "t=%d, m=%d, m2=%d, m3=%d\n",
							tick_ms, mode, mode2, mode3);
	Serial_Printf(&huart5, "ay=%f, ap=%f\n",
							angle_yaw, angle_pitch);
//	Serial_Printf(&huart5, "%d, %d, %d, %d, %d, %d, %d, %d, %d\n",
//							tick_ms, gray_analog[0], gray_analog[1], gray_analog[2], gray_analog[3],
//							gray_analog[4], gray_analog[5], gray_analog[6], gray_analog[7]);
	// 发送灰度传感器数字值
	
//	Serial_Printf(&huart5, "%d, %d, %d, %d, %d, %d, %d, %d, %d\n",
//							tick_ms, gray_digital[0], gray_digital[1], gray_digital[2], gray_digital[3],
//							gray_digital[4], gray_digital[5], gray_digital[6], gray_digital[7]);
}
void Task_BLE_Rx(void)
{
	// 检查接收标志位
	if (Serial_GetRxFlag(&huart5) == 0)return;
	
	// 解析命令标签
	char *Tag = strtok((char *)Serial_GetRxPacket(&huart5), ", ");
	if (Tag != NULL && strcmp(Tag, "key") == 0)
	{
		char *name = strtok(NULL, ", ");
		if (name != NULL && strcmp(name, "stop") == 0)
			stop_flag = 1;
	}
	
	
	
}

/**
 * @brief  屏幕数据发送任务，向串口屏发送灰度传感器数据
 * @param  无
 * @return 无
 */
void Task_Screen_Tx(void)
{
	// 检查定时标志位
	if (screen_Tx_timer_flag == 0)return;
	screen_Tx_timer_flag = 0;
	
	if (para_flag)
	{	//while(1)Serial_Printf(&huart5, "111");
		// 发送灰度传感器数据到串口屏
		Serial_Printf(&huart7, "t0.txt=\"Gray:%d %d %d %d %d %d %d %d\"\xff\xff\xff",
								gray_digital[0], gray_digital[1], gray_digital[2], gray_digital[3],
								gray_digital[4], gray_digital[5], gray_digital[6], gray_digital[7]);
		Serial_Printf(&huart7, "t1.txt=\"Angle:%.1f\"\xff\xff\xff", angle);
		Serial_Printf(&huart7, "t2.txt=\"Laps:%d\"\xff\xff\xff", laps);
	}
}

/**
 * @brief  屏幕数据接收任务，解析串口屏命令并切换模式
 * @param  无
 * @return 无
 */
void Task_Screen_Rx(void)
{
	// 检查接收标志位
	if (Serial_GetRxFlag(&huart7) == 0)return;
	
	// 解析命令标签
	char *Tag = strtok((char *)Serial_GetRxPacket(&huart7), ", ");
	if (Tag != NULL && strcmp(Tag, "mode") == 0)
	{
		char *Value1 = strtok(NULL, ", ");
		if (Value1 != NULL)
		{
			if (strcmp(Value1, "index") == 0)
				mode = 0;
			else if (strcmp(Value1, "mode1") == 0)
				mode = 1;
			else if (strcmp(Value1, "mode2") == 0)
				mode = 2;
			else if (strcmp(Value1, "mode3") == 0)
				mode = 3;
			else if (strcmp(Value1, "mode4") == 0)
				mode = 4;

			if (strcmp(Value1, "para") == 0)
				para_flag = 1;
			else 
				para_flag = 0;
		}
	}
	else if (Tag != NULL && strcmp(Tag, "mode1") == 0)
	{
		char *name = strtok(NULL, ", ");
		if (name != NULL && strcmp(name, "N") == 0)
		{
			char *Value1 = strtok(NULL, ", ");
			if (Value1 != NULL)
			{
				int IntValue1 = atof(Value1);
				mode1_N = IntValue1;	
				mode1_line_flag = 1;
				para_flag = 1;
				//while(1)Serial_Printf(&huart5, "111");
			}		
		}
	}	
	else if (Tag != NULL && strcmp(Tag, "mode2") == 0)
	{
		char *name = strtok(NULL, ", ");
		if (name != NULL && strcmp(name, "lock") == 0)
		{
			mode2 = 1;
			para_flag = 1;
		}
		else if (name != NULL && strcmp(name, "auto") == 0)
		{
			mode2 = 2;
			mode2_gimbal_flag = 1;
			para_flag = 1;
			//while(1)Serial_Printf(&huart5, "111");
		}	
	}
	else if (Tag != NULL && strcmp(Tag, "mode3") == 0)
	{
		char *name = strtok(NULL, ", ");
		if (name != NULL && strcmp(name, "B1") == 0)
		{
			mode3 = 1;
			mode3_N = 1;
			para_flag = 1;
		}
		else if (name != NULL && strcmp(name, "B2") == 0)
		{
			mode3 = 2;
			mode3_N = 2;
			para_flag = 1;
		}
		else if (name != NULL && strcmp(name, "B3") == 0)
		{
			mode3 = 3;
			mode3_N = 1;
			para_flag = 1;
		}
	}	
	else if (Tag != NULL && strcmp(Tag, "mode4") == 0)
	{
		char *name = strtok(NULL, ", ");
		if (name != NULL && strcmp(name, "laser") == 0)
		{
			char *condition = strtok(NULL, ", ");
			if (condition != NULL && strcmp(condition, "on") == 0)
				Laser_Enable();
			else if (condition != NULL && strcmp(condition, "off") == 0)
				Laser_Disable();
		}
		else if (name != NULL && strcmp(name, "buzzer") == 0)
		{
			char *condition = strtok(NULL, ", ");
			if (condition != NULL && strcmp(condition, "on") == 0)
				Buzzer_Enable();
			else if (condition != NULL && strcmp(condition, "off") == 0)
				Buzzer_Disable();
		}
		else if (name != NULL && strcmp(name, "angle") == 0)
		{
			char *Value1 = strtok(NULL, ", ");
			char *Value2 = strtok(NULL, ", ");
			if (Value1 != NULL && Value2 != NULL)
			{
				float FloatValue1 = atof(Value1);
				angle_yaw = FloatValue1;	
				motor_flag[2] = 2;
				
				float FloatValue2 = atof(Value2);
				angle_pitch = FloatValue2;	
				motor_flag[3] = 2;
				//while(1)Serial_Printf(&huart5, "111");
			}				
		}
		else if (name != NULL && strcmp(name, "zero1") == 0)
		{
			Motor_SetZero(&motor_yaw);
		}
		else if (name != NULL && strcmp(name, "zero2") == 0)
		{
			Motor_SetZero(&motor_pitch);
		}
	}
	
	if (mode == 0)
	{
		mode1_N = 0;
		mode1_line_flag = 0;
		mode2 = 0;
		mode3 = 0;
		mode3_line_flag = 0;
		HWT101_ClearLapCount();
		//Laser_Disable();
	}
}	

/**
 * @brief  传感器读取任务，获取姿态角和灰度传感器数据
 * @param  无
 * @return 无
 */
void Task_Read_Sensor(void)
{
	// 检查定时标志位
	if (sensor_timer_flag == 0)return;
	sensor_timer_flag = 0;

	// 读取姿态角
	angle = HWT101_GetYaw();
	laps = HWT101_GetLapCount();

	// 发送灰度传感器查询命令
	GraySensor_SendQuery(&huart1, 1);
	// 读取灰度传感器数字值
	for (uint8_t i = 0; i < 8; i++)
	{
		gray_digital[i] = GraySensor_GetDigital(i);
	}
	
	Cam_ReadXY();
	target_x = Cam_GetXY(CAM_X, CAM_TARGERT);
	target_y = Cam_GetXY(CAM_Y, CAM_TARGERT);
	laser_x = Cam_GetXY(CAM_X, CAM_LASER);
	laser_y = Cam_GetXY(CAM_Y, CAM_LASER);
	
}

/* ==================== Task ==================== */
/**
 * @brief  循迹任务
 * @param  无
 * @return 无
 */
void Task_Line(void)
{
	switch (mode)
	{
		case 1://mode1循迹
			if (mode1_line_flag == 0)return;	//开始控制
			Line();						//更新速度，给Task_Motor
			if (abs(laps) >= mode1_N && 
				angle >= -10 && angle <= 10 &&
				gray_digital[0] == 1 && gray_digital[1] == 1 && gray_digital[3] == 1)
			{
				mode = 0;
				motor_flag[0] = 1;
				speed_left = 0;
				motor_flag[1] = 1;
				speed_right = 0;
			}
		break;
		
		case 3://mode3循迹
			if (mode3_line_flag == 0)return;	//开始控制
			Line();						//更新速度，给Task_Motor
			if (abs(laps) >= mode3_N && 
				angle >= -10 && angle <= 10 &&
				gray_digital[0] == 1 && gray_digital[1] == 1 && gray_digital[3] == 1)
			{
				mode = 0;
				motor_flag[0] = 1;
				speed_left = 0;
				motor_flag[1] = 1;
				speed_right = 0;
			}
		break;
	}
}
/**
 * @brief  瞄准任务
 * @param  无
 * @return 无
 */
void Task_Gimbal(void)
{
	static uint8_t mode2_state;
	switch (mode)
	{
		case 2://mode2瞄准
			if (mode2_gimbal_flag == 0)return;
			//自动瞄准状态机
			switch (mode2_state)
			{
				case (0):// 定->定瞄准
					//while(1)Serial_Printf(&huart5, "333");
					if (Gimbal_Aim1(mode2) == 1)
						mode2_state = 1;
					break;
				case (1):// 发射激光
					//while(1)Serial_Printf(&huart5, "444");
					Laser_Enable();
					mode2_state = 2;
					break;
				case (2):// 等待关闭
					if (mode != 2)
					{
						Laser_Disable();
						mode2_state = 0;
					}
					break;
			}	
			
			break;
		
		case 3://mode3瞄准
			if (mode3_gimbal_flag == 0)return;
			break;
	}
	
}
/**
 * @brief  电机运动执行任务，轮询发送电机控制指令
 * @param  无
 * @return 无
 */
void Task_Motor(void)
{
	static uint8_t motor_count;
	static uint8_t motor_send_idx;

	// 循迹速度获取，Line
	if (Line_GetFlag())
	{
		speed_left = Line_GetSpeed(LINE_MOTOR_LEFT);
		if (Motor_CompareLastValue(0, speed_left))
			motor_flag[0] = 1;
		speed_right = Line_GetSpeed(LINE_MOTOR_RIGHT);
		if (Motor_CompareLastValue(1, speed_right))
			motor_flag[1] = 1;
	}

	// 云台控制，Gimbal
	if (Gimbal_GetFlag())
	{
		speed_yaw = Gimbal_GetSpeed(GIMBAL_MOTOR_YAW);
		if (Motor_CompareLastValue(2, speed_yaw))
			motor_flag[2] = 1;
		speed_pitch = Gimbal_GetSpeed(GIMBAL_MOTOR_PITCH);
		if (Motor_CompareLastValue(3, speed_pitch))
			motor_flag[3] = 1;
//		angle_yaw = Gimbal_GetSpeed(GIMBAL_MOTOR_YAW);
//		if (Motor_CompareLastValue(2, angle_yaw))
//			motor_flag[2] = 2;
//		angle_pitch = Gimbal_GetSpeed(GIMBAL_MOTOR_PITCH);
//		if (Motor_CompareLastValue(3, angle_pitch))
//			motor_flag[3] = 2;
	}

	// 电机发送间隔控制
	if (motor_timer_flag == 1)
	{
		motor_timer_flag = 0;
		motor_count++;
	}
	if (motor_count < 2) return;  // 2ms电机间隔
	motor_count = 0;

	// 轮询发送电机指令
	for (uint8_t i = 0; i < 4; i++)
	{
		uint8_t idx = (motor_send_idx + i) % 4;

		if (motor_flag[idx])
		{
			// 发送对应电机指令
			switch (idx)
			{
				case 0:	Motor_SetSpeed(&motor_left, speed_left);     break;
				case 1:	Motor_SetSpeed(&motor_right, speed_right);   break;
				case 2:	if (motor_flag[2] == 1)
							Motor_SetSpeed(&motor_yaw, speed_yaw);
						else if (motor_flag[2] == 2)//相对位置
							Motor_SetRelAngle(&motor_yaw, angle_yaw);    
						else if (motor_flag[2] == 3)//绝对位置
							Motor_SetAbsAngle(&motor_yaw, angle_yaw);break;    
				case 3: if (motor_flag[3] == 1) 
							Motor_SetSpeed(&motor_pitch, speed_pitch);
						else if (motor_flag[3] == 2)//相对位置
							Motor_SetRelAngle(&motor_pitch, angle_pitch);    
						else if (motor_flag[3] == 3)//绝对位置
							Motor_SetAbsAngle(&motor_pitch, angle_pitch);break;
			}

			motor_flag[idx] = 0;            // 清除标志
			motor_send_idx = (idx + 1) % 4; // 下次从下一个电机开始
			return;                         // 一次只发一个，立即退出
		}
	}
	motor_send_idx = 0;
}

void Task_Stop(void)
{
	if (stop_flag == 0)return;
	stop_flag = 0;
	
	Motor_Stop(&motor_all);
	while(1)
	{
		Buzzer_Enable();
		system_delay_ms(100);
		Buzzer_Disable();
		system_delay_ms(100);
	}
	
}

/* ==================== Init ==================== */
/**
 * @brief  电机变量初始化，初始化四个电机的参数
 * @param  无
 * @return 无
 */

void Task_Motor_Init(void)
{
	// 广播
	Motor_Init(&motor_all, 0, 0);
	// 初始化左电机
	Motor_Init(&motor_left, 1, 0);
	// 初始化右电机
	Motor_Init(&motor_right, 2, 1);
	// 初始化云台Yaw轴电机
	Motor_Init(&motor_yaw, 3, 0);
	// 初始化云台Pitch轴电机
	Motor_Init(&motor_pitch, 4, 0);
	
	Motor_SetAbsAngle(&motor_pitch, -0.2);
	
//	for (uint8_t i = 0; i < 4; i++)
//	{
//		motor_flag[i] = 1;
//	}
}

void Task_Motor_Test(void)
{
	Motor_SetSpeed(&motor_yaw, 10);
	
}
