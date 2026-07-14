/**
 * @file    My_Timer.h
 * @brief   定时器中断任务分发接口说明
 *
 * @note    本模块不提供额外初始化函数。定时器硬件由 CubeMX 生成的
 *          TIM 初始化代码配置，业务代码只需要在系统初始化后启动
 *          TIM1 周期中断即可。
 *
 *          调用示例（放在初始化代码中）：
 *          HAL_TIM_Base_Start_IT(&htim1);
 *
 * @warning `HAL_TIM_PeriodElapsedCallback()` 在中断上下文运行，用户任务
 *          应通过 `Task_Timer()` 置位标志或做轻量计数，耗时操作放到主循环。
 */

#ifndef __MY_TIMER_H
#define __MY_TIMER_H

#endif
