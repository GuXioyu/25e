/**
 * @file    My_Timer.c
 * @brief   定时器中断任务分发
 *
 * @note    当前使用 TIM1 作为系统任务节拍源。TIM1 周期中断到来后，
 *          在 HAL 回调中调用 `Task_Timer()`，由 Task 层完成毫秒计数、
 *          电机发送节拍、灰度读取节拍、OLED/BLE 刷新节拍等分频处理。
 *
 * @warning 该回调运行在中断上下文中，内部只应做轻量分发，不要在这里
 *          调用阻塞函数或执行耗时逻辑。
 */

#include "stm32f4xx_hal.h"
#include "Task.h"
#include "Line.h"

/**
 * @brief  HAL 定时器周期中断回调
 * @param  htim  触发本次周期中断的定时器句柄
 *
 * @note   HAL_TIM_IRQHandler() 在定时器更新中断中调用本函数。这里通过
 *         `htim->Instance == TIM1` 过滤中断来源，只把 TIM1 的周期中断
 *         作为系统任务节拍交给 `Task_Timer()` 处理。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* 当 TIM1 产生周期中断时（例如 1ms），进入任务层分频调度 */
	if (htim->Instance == TIM1)
	{
		Task_Timer();
		Line_Timer();
	}
}

