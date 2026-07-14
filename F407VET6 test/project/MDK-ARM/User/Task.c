#include "stm32f4xx_hal.h"
#include "usart.h"
#include "My_Uart.h"
#include "My_Timer.h"
#include "OLED.h"
#include "GraySensor.h"
#include "PID.h"
#include "Motor.h"
#include "HWT101.h"

uint16_t tick_ms;

uint8_t OLED_timer_flag, BLE_timer_flag, motor_timer_flag, sensor_timer_flag;	// 定时标志位
uint8_t motor_flag;																														// 动作标志位
uint8_t mode;

Motor_t motor_left, motor_right, motor_yow, motor_pitch;

int16_t ang[4];
int16_t speed[4];

uint16_t gray;
uint16_t gray_analog[8];
uint8_t  gray_digital[8];

float angle;

void Task_Timer(void)// 放在定时器里运行，防止数据高频收发延时
{
	static uint8_t count_1ms, count_10ms, count_100ms;
	tick_ms++;
	count_1ms++;
	count_10ms++;
	count_100ms++;
	// 分频
	if (count_1ms >= 1U)
	{
		count_1ms = 0U;
		motor_timer_flag = 1;
		sensor_timer_flag = 1;
		
	}
	if (count_10ms >= 10U)
	{
		count_10ms = 0U;
		
		
	}
	if (count_100ms >= 100U)
	{
		count_100ms = 0U;
		OLED_timer_flag = 1;
		BLE_timer_flag = 1;
	}
}

// OLED UI显示
void Task_OLED_UI(void)
{
	if (OLED_timer_flag)
	{
		OLED_timer_flag = 0;
		OLED_ShowNum(0,		0, 	tick_ms, 					5, OLED_8X16);
		OLED_ShowNum(0, 	16, gray_analog[0],		5, OLED_6X8);
		OLED_ShowNum(30, 	16, gray_analog[1], 	5, OLED_6X8);
		OLED_ShowNum(60, 	16, gray_analog[2], 	5, OLED_6X8);
		OLED_ShowNum(90, 	16, gray_analog[3], 	5, OLED_6X8);
		OLED_ShowNum(0, 	24, gray_analog[4], 	5, OLED_6X8);
		OLED_ShowNum(30, 	24, gray_analog[5], 	5, OLED_6X8);
		OLED_ShowNum(60, 	24, gray_analog[6], 	5, OLED_6X8);
		OLED_ShowNum(90, 	24, gray_analog[7], 	5, OLED_6X8);
		OLED_Update();
	}
}

// 蓝牙
void Task_BLE(void)
{
	if (BLE_timer_flag)
	{
		BLE_timer_flag = 0;
			
//		Serial_Printf(&huart1, "%d, %f, %d\n",
//									 tick_ms, angle, gray);
//		Serial_Printf(&huart1, "%d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
//									 tick_ms, gray_analog[0], gray_analog[1], gray_analog[2], gray_analog[3],
//														gray_analog[4], gray_analog[5], gray_analog[6], gray_analog[7]);
		Serial_Printf(&huart1, "%d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
									 tick_ms, gray_digital[0], gray_digital[1], gray_digital[2], gray_digital[3],
														gray_digital[4], gray_digital[5], gray_digital[6], gray_digital[7]);

	}
}

// Motor 变量初始化
void Task_Motor_Init(void)
{
	motor_flag = 1;
	Motor_Init(&motor_left, 1, 0);
	Motor_Init(&motor_right, 2, 1);
	Motor_Init(&motor_yow, 3, 0);
	Motor_Init(&motor_pitch, 4, 0);
}

// Motor 电机运动执行命令
void Task_Motor(void)
{
	static uint8_t motor_count, motor_step;
	if (motor_flag == 0)return;// 没有动作命令，直接退出
	if (motor_timer_flag)
	{
		motor_timer_flag = 0;
		motor_count++;
	}
	if (motor_count >= 2)// 2ms电机间隔
	{
		motor_count = 0;
		motor_step++;
		if (motor_step == 1)					// left
			Motor_SetSpeed(&motor_left, 0);
		else if (motor_step == 2)			// right
			Motor_SetSpeed(&motor_right, 0);
		else if (motor_step == 3)			// yow
			Motor_SetAbsAngle(&motor_yow, 0);
		else if (motor_step == 4)			// pitch
		{
			Motor_SetAbsAngle(&motor_pitch, 0);
			
			motor_step = 0;
			motor_flag = 0;
			motor_timer_flag = 0;
		}
	}
}

void Task_Read_Sensor(void)
{
	if (sensor_timer_flag)
	{
		sensor_timer_flag = 0;
		angle = HWT101_GetYaw();
		GraySensor_SendQuery(&huart4, 1);
		for (uint8_t i = 0; i < 8; i++)
		{
			gray_digital[i] = GraySensor_GetDigital(i);	
		}
		
	}
}

void Task_Serial(void)
{
	if (Serial_RxFlag[3] == 1)
	{
		char *Tag = strtok((char *)Serial_RxPacket[3], ", ");
		// 模式切换
		if (Tag != NULL && strcmp(Tag, "mode") == 0)
		{
			char *Value1 = strtok(NULL, ", ");
			if (Value1 != NULL)
			{
				if (strcmp(Value1, "return") == 0)
				{
					mode = 0;
					
				}
				else if (strcmp(Value1, "mode1") == 0)
					mode = 1;
				else if (strcmp(Value1, "mode2") == 0)  
					mode = 2;
				else if (strcmp(Value1, "mode3") == 0)
					mode = 3;
				else if (strcmp(Value1, "para") == 0)
					mode = 4;
				else if (strcmp(Value1, "test") == 0)
					mode = 5;
			}
		}
		Serial_RxFlag[3] = 0;
	}	
}			

