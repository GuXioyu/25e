#include "zf_common_headfile.h"
#include "My_Timer.h"
#include "My_Uart.h"
#include "OLED.h"
#include "Task.h"
// **************************** 代码区域 ****************************

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
	Serial_Init(&huart4);//*
	Serial_Init(&huart5);//*
	Serial_Init(&huart6);
	Serial_Init(&huart7);

    while(true)
    {
 		Task_OLED_UI();
//		Task_BLE();
		Serial_Printf(&huart0, "000");
		Serial_Printf(&huart1, "111");
		Serial_Printf(&huart3, "333");
		Serial_Printf(&huart4, "444");
		Serial_Printf(&huart5, "555");
		Serial_Printf(&huart6, "666");
		Serial_Printf(&huart7, "777");
		system_delay_ms(100);
    }
}

// **************************** 代码区域 ****************************
