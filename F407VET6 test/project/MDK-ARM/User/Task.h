#ifndef __TASK_H
#define __TASK_H

// 初始化
void Task_Motor_Init(void);

// 定时
void Task_Timer(void);
// 任务
void Task_OLED_UI(void);
void Task_BLE(void);
void Task_Read_Sensor(void);
void Task_Motor(void);


#endif
