#include "PID.h"

/**
 * @brief 更新一次位置式 PID 控制输出
 * @param p PID 控制器实例指针
 * @return 无
 */
void PID_Update(PID_t *p) //更新一次 PID 控制器输出
{
	float ProportionalOut = 0.0f; //保存本次比例项输出
	float IntegralOut = 0.0f; //保存本次积分项输出

	/* 更新本次控制所需的误差历史 */
	p->error_past = p->error_current; //保存上次误差
	p->error_current = p->target - p->actual_current; //计算当前误差

	/* 按比例模式计算比例项 */
	if (p->mode_p == PID_PMODE_ENABLE)
	{
		ProportionalOut = p->kp * p->error_current; //生成比例项输出
	}

	/* 按积分模式累积误差并始终执行积分限幅 */
	if ((p->mode_i == PID_IMODE_DISABLE) || (p->ki == 0.0f))
	{
		p->error_int = 0.0f; //禁用积分或积分系数为零时清除积分状态
	}
	else if (p->mode_i == PID_IMODE_NORMAL)
	{
		p->error_int += p->error_current; //按普通积分累积误差
	}
	else if (p->mode_i == PID_IMODE_VAR_NONLINEAR)
	{
		p->error_int += p->error_current / (p->intk_nonline * fabsf(p->error_current) + 1.0f); //按非线性系数累积误差
	}
	else if (p->mode_i == PID_IMODE_SEPARATION)
	{
		if (fabsf(p->error_current) > p->int_sep_threshold)
		{
			p->error_int = 0.0f; //误差超过阈值时清除积分状态
		}
		else
		{
			p->error_int += p->error_current; //误差在阈值内时累积积分
		}
	}
	else
	{
		p->error_int = 0.0f; //无效积分模式时清除积分状态
	}
	if (p->error_int > p->error_intmax)
	{
		p->error_int = p->error_intmax; //限制积分上限
	}
	if (p->error_int < p->error_intmin)
	{
		p->error_int = p->error_intmin; //限制积分下限
	}
	IntegralOut = p->ki * p->error_int; //生成积分项输出

	/* 按微分模式生成每实例微分输出 */
	if ((p->mode_d == PID_DMODE_DISABLE) || (p->kd == 0.0f))
	{
		p->dif_out = 0.0f; //禁用微分或微分系数为零时清除微分状态
	}
	else if (p->mode_d == PID_DMODE_NORMAL)
	{
		p->dif_out = p->kd * (p->error_current - p->error_past); //按误差变化计算普通微分
	}
	else if (p->mode_d == PID_DMODE_FRONT)
	{
		p->dif_out = -p->kd * (p->actual_current - p->actual_past); //按测量值变化计算微分先行
	}
	else if (p->mode_d == PID_DMODE_IMPERFECT)
	{
		p->dif_out = (1.0f - p->dif_filter) * p->kd * (p->error_current - p->error_past)
					 + p->dif_filter * p->dif_out; //使用历史微分输出进行一阶滤波
	}
	else
	{
		p->dif_out = 0.0f; //无效微分模式时清除微分状态
	}
	p->actual_past = p->actual_current; //保存当前测量值供下一次微分使用

	/* 合成输出并保留原有执行器处理能力 */
	p->out = ProportionalOut + IntegralOut + p->dif_out; //合成三项控制输出
	if (p->out > 0.0f)
	{
		p->out += p->out_offset; //为正向非零输出补偿偏置
	}
	else if (p->out < 0.0f)
	{
		p->out -= p->out_offset; //为反向非零输出补偿偏置
	}
	if (p->out > p->outmax)
	{
		p->out = p->outmax; //限制输出上限
	}
	if (p->out < p->outmin)
	{
		p->out = p->outmin; //限制输出下限
	}
	if (fabsf(p->error_current) < p->out_deadzone)
	{
		p->out = 0.0f; //误差进入死区时停止输出
	}
}
