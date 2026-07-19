#include "Task.h"

extern uint16_t tick_ms;
extern uint8_t mode;

uint8_t mode1_N;					//A1：目标圈数
uint8_t mode1_line_flag;			
uint8_t mode2_gimbal_flag;
uint8_t mode3_N;					//B:目标圈数
uint8_t mode3_line_flag;
uint8_t mode3_gimbal_flag;

uint8_t  OLED_timer_flag, screen_Tx_timer_flag,  BLE_Tx_timer_flag;// 定时标志位
uint8_t  motor_timer_flag, sensor_timer_flag;	

uint8_t motor_flag[4]; 												// 动作标志位
uint8_t stop_flag;
uint8_t para_flag;

Motor_t motor_left, motor_right, motor_yow, motor_pitch;

int16_t ang[4];
int16_t speed_left, speed_right, speed_yaw, speed_pitch;

uint16_t gray;				// 灰度数据
uint16_t gray_analog[8];
uint8_t  gray_digital[8];

uint16_t x,y;

int8_t laps;		//	已转圈数

float angle;

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
//	Serial_Printf(&huart5, "%d, %d, %d, %d\n",
//							tick_ms, mode, speed_left, speed_right);
	Serial_Printf(&huart5, "%d, %d %d %d\n",
							mode1_line_flag, para_flag, speed_left, speed_right);
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
		if (name != NULL && strcmp(Tag, "stop") == 0)
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
	{
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
			}		
		}
	}	
	else if (Tag != NULL && strcmp(Tag, "mode2") == 0)
	{
		char *name = strtok(NULL, ", ");
		
	}
	else if (Tag != NULL && strcmp(Tag, "mode3") == 0)
	{
		char *name = strtok(NULL, ", ");
		if (name != NULL && strcmp(Tag, "N") == 0)
		{
			char *Value1 = strtok(NULL, ", ");
			if (Value1 != NULL)
			{
				int IntValue1 = atof(Value1);
				mode3_N = IntValue1;	
			}		
		}
		else if (name != NULL && strcmp(Tag, "OK") == 0)
		{
			mode3_line_flag = 1;
		}
	}	
	if (mode == 0)
	{
		mode1_N = 0;
		mode1_line_flag = 0;
		mode3_N = 0;
		mode3_line_flag = 0;
		HWT101_ClearLapCount();
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
	
	Gimbal_ReadXY();
	x = Gimbal_GetXY(1);
	y = Gimbal_GetXY(1);
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
	switch (mode)
	{
		case 2://mode2瞄准
			if (mode2_gimbal_flag == 0)return;
			//瞄准
			//发射激光
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

	// 循迹速度获取
	if (Line_GetFlag())
	{
		speed_left = Line_GetSpeed(1);
		if (Motor_CompareLastValue(0, speed_left))
			motor_flag[0] = 1;
		speed_right = Line_GetSpeed(2);
		if (Motor_CompareLastValue(1, speed_right))
			motor_flag[1] = 1;
	}

	// 云台控制（预留）
	if (0)
	{
		speed_yaw = Gimbal_GetSpeed(GIMBAL_MOTOR_YAW);
		if (Motor_CompareLastValue(2, speed_left))
			motor_flag[2] = 1;
		speed_pitch = Gimbal_GetSpeed(GIMBAL_MOTOR_PITCH);
		if (Motor_CompareLastValue(3, speed_left))
			motor_flag[3] = 1;
		
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
				case 0:  Motor_SetSpeed(&motor_left, speed_left);     break;
				case 1:  Motor_SetSpeed(&motor_right, speed_right);   break;
				case 2:  Motor_SetAbsAngle(&motor_yow, speed_yaw);    break;
				case 3:  Motor_SetAbsAngle(&motor_pitch, speed_pitch);   break;
//				case 0:  Motor_SetSpeed(&motor_left, 10);    break;
//				case 1:  Motor_SetSpeed(&motor_right, 0);   break;
//				case 2:  Motor_SetSpeed(&motor_yow, 0);    	break;
//				case 3:  Motor_SetSpeed(&motor_pitch, 10);  break;
			}

			motor_flag[idx] = 0;            // 清除标志
			motor_send_idx = (idx + 1) % 4; // 下次从下一个电机开始
			return;                         // 一次只发一个，立即退出
		}
	}
	motor_send_idx = 0;
}

/* ==================== Init ==================== */
/**
 * @brief  电机变量初始化，初始化四个电机的参数
 * @param  无
 * @return 无
 */

void Task_Motor_Init(void)
{
	// 初始化左电机
	Motor_Init(&motor_left, 1, 0);
	// 初始化右电机
	Motor_Init(&motor_right, 2, 1);
	// 初始化云台Yaw轴电机
	Motor_Init(&motor_yow, 3, 0);
	// 初始化云台Pitch轴电机
	Motor_Init(&motor_pitch, 4, 0);
	
//	for (uint8_t i = 0; i < 4; i++)
//	{
//		motor_flag[i] = 1;
//	}
}

void Task_Motor_Test(void)
{
	Motor_SetSpeed(&motor_left, 50);
	
}
