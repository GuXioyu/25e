#ifndef __TASK_H
#define __TASK_H

#include "zf_common_headfile.h"
#include "My_Uart.h"
#include "My_Timer.h"
#include "OLED.h"
#include "GraySensor.h"
#include "PID.h"
#include "Motor.h"
#include "HWT101.h"
#include "Line.h"
#include "Gimbal.h"

/* ==================== Init ==================== */
void Task_Motor_Init(void);
/* ==================== Timer ==================== */
void Task_Timer(void);
/* ==================== 通信 ==================== */
void Task_OLED_UI(void);
void Task_BLE_Rx(void);
void Task_BLE_Tx(void);
void Task_Screen_Rx(void);
void Task_Screen_Tx(void);
void Task_Read_Sensor(void);
/* ==================== Task ==================== */
void Task_Line(void);
void Task_Gimbal(void);
void Task_Motor(void);
//急停任务处理
void Task_Stop(void);


#endif
