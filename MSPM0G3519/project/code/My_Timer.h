/**
 * @file    My_Timer.h
 * @brief   定时器中断任务分发接口
 *
 * @note    My_Timer_Init() 使用 TIMA0/PIT_TIM_A0 产生 1ms 周期中断，
 *          每次中断都会分发到上层 Task_Timer()。
 */

#ifndef __MY_TIMER_H
#define __MY_TIMER_H

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 1ms 系统节拍定时器
// 参数说明     void
// 返回参数     void
// 使用示例     My_Timer_Init();
// 备注信息     建议在 clock_init()、debug_init() 之后调用
//-------------------------------------------------------------------------------------------------------------------
void My_Timer_Init(void);

#endif
