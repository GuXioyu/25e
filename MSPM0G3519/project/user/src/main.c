#include "zf_common_headfile.h"
#include "My_Timer.h"
#include "My_Uart.h"
#include "OLED.h"
#include "Task.h"
#include "Line.h"
#include "Gimbal.h"
#include "Buzzer.h"
#include "Laser.h"
#include "HWT101.h"
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
	Task_Motor_Init();
	Buzzer_Init();
	Laser_Init();
	HWT101_SetYawZero(&huart6);
	
	Laser_Disable();
//	Task_Motor_Test();
//	while(1);
//	while(1)
//	{
//		Laser_Enable();
//		system_delay_ms(100);
//		Laser_Disable();
//		system_delay_ms(100);
//		
//	}
    while(true)
    {
		
		//通信
		Task_BLE_Rx();
		//Task_BLE_Tx();
		//Task_OLED_UI();
		Task_Screen_Rx();
		Task_Screen_Tx();
		
		
		Task_Read_Sensor();			//读取传感器：灰度 TWH101 Cam
		
		Task_Line();				//循迹
		//Task_Gimbal();
		Task_Motor();				//控制电机
		//Task_Stop();
		
    }
}

// **************************** 代码区域 ****************************
