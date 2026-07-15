#include "zf_common_headfile.h"
#include "My_Timer.h"
#include "My_Uart.h"
#include "OLED.h"
#include "Task.h"
#include "Line.h"
#include "Gimbal.h"
// **************************** 代码区域 ****************************

uint16_t tick_ms;
uint8_t mode;

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>
    debug_init();                                                               // 调试串口信息初始化
    
	// 初始化
	My_Timer_Init();
    OLED_Init();
	Serial_Init(&huart0);
	Serial_Init(&huart1);
	Serial_Init(&huart3);
	Serial_Init(&huart4);
	Serial_Init(&huart5);
	Serial_Init(&huart6);
	Serial_Init(&huart7);
	//Task_Motor_Init();
	
	int16_t x,y;
    while(true)
    {
// 		Task_OLED_UI();
//		Task_BLE();
//		Task_Read_Sensor();
//		
//		Task_Screen_Rx();
//		
//		Task_Screen_Tx();
//		
		
//		Serial_Printf(&huart1, "111");
//		Serial_Printf(&huart7, "777");
//		system_delay_ms(100);
//		//循迹
//		Line();
//		Task_Line_Motor();
		//Task_Motor();
		
    }
}

// **************************** 代码区域 ****************************
