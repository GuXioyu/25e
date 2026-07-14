#include "GraySensor.h"

uint8_t gray_timer_flag;


uint8_t line_digital[8];

void Line_GetGray()
{
	if (gray_timer_flag == 1)
	{
		gray_timer_flag = 0;
		//GraySensor_SendQuery(&huart1, 1);
		for (uint8_t i = 0; i < 8; i++)
		{
			line_digital[i] = GraySensor_GetDigital(i);	
		}
		
	}
	
}

//						0 0 0 0  0 0 0 0
//							  -1 +1 +3 +5 +7
void Line_GetState()
{
	
	
	
	
}



// 定时器
void Line_Timer()
{
	static uint8_t count_1ms, count_10ms, count_100ms;
	count_1ms++;
	count_10ms++;
	count_100ms++;
	// 分频
	if (count_1ms >= 1U)
	{
		count_1ms = 0;
	}
	if (count_10ms >= 10U)
	{
		count_10ms = 0;
		gray_timer_flag = 1;
	}
	if (count_100ms >= 100U)
	{
		count_100ms = 0;
	}
	
	
}
