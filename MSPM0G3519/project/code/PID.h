#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <math.h>

#define PID_PMODE_DISABLE          (0U) /* 禁用比例项 */
#define PID_PMODE_ENABLE           (1U) /* 启用比例项 */

#define PID_IMODE_DISABLE          (0U) /* 禁用积分项 */
#define PID_IMODE_NORMAL           (1U) /* 普通积分 */
#define PID_IMODE_VAR_NONLINEAR    (2U) /* 变速非线性积分 */
#define PID_IMODE_SEPARATION       (3U) /* 积分分离 */

#define PID_DMODE_DISABLE          (0U) /* 禁用微分项 */
#define PID_DMODE_NORMAL           (1U) /* 普通微分 */
#define PID_DMODE_FRONT            (2U) /* 微分先行 */
#define PID_DMODE_IMPERFECT        (3U) /* 不完全微分 */

typedef struct
{
	float kp; //比例系数
	float ki; //积分系数
	float kd; //微分系数

	float target; //目标值
	float actual_current; //当前测量值
	float actual_past; //上次测量值
	float out; //本次输出值

	float error_current; //当前误差
	float error_past; //上次误差
	float error_int; //误差积分值
	float dif_out; //微分项历史输出值

	uint8_t mode_p; //比例项模式
	uint8_t mode_i; //积分项模式
	uint8_t mode_d; //微分项模式
	float intk_nonline; //非线性积分系数
	float int_sep_threshold; //积分分离阈值
	float error_intmax; //积分上限
	float error_intmin; //积分下限
	float dif_filter; //微分滤波系数，范围为 0 至 1

	float out_offset; //非零输出偏置
	float outmax; //输出上限
	float outmin; //输出下限
	float out_deadzone; //输入误差死区
} PID_t;

//更新一次 PID 控制器输出
void PID_Update(PID_t *p); //更新一次 PID 控制器输出

#endif
