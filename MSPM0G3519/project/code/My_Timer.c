/**
 * @file    My_Timer.c
 * @brief   定时器中断任务分发
 *
 * @note    使用逐飞库 PIT_TIM_A0 作为 1ms 系统节拍源。PIT 中断到来后，
 *          回调函数调用上层 Task_Timer()，由任务层完成毫秒计数和分频调度。
 */
#include "My_Timer.h"
#include "Task.h"

/**
 * @brief  PIT 周期中断回调
 * @param  event  逐飞库回调事件参数，本模块暂不使用
 * @param  ptr    用户自定义回调参数，本模块暂不使用
 *
 * @note   该函数运行在中断上下文中，只做轻量分发，不要在这里执行耗时操作。
 */
static void my_timer_pit_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;

    Task_Timer();
}

/**
 * @brief  初始化 1ms 系统节拍定时器
 *
 * @note   使用 TIMA0/PIT_TIM_A0，周期为 1ms。应在 clock_init() 和 debug_init()
 *         之后调用，避免和其他使用 TIMA0 的功能重复初始化。
 */
void My_Timer_Init(void)
{
    pit_ms_init(PIT_TIM_A0, 1, my_timer_pit_callback, NULL);
}
