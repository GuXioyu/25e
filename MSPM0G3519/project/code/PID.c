#include "PID.h"

float intC;    //积分增量的比例系数
float difout;  //微分项的输出值

/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：PID_t * 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->error_past = p->error_current;				        //获取上次误差
	p->error_current = p->target - p->actual_current;	    //获取本次误差，目标值减实际值，即为误差值
	
    /*误差积分*/
    if (p->ki != 0)					                        //如果Ki不为0
    {
        /*正常积分*/
        if (p->intmode & PID_INTMODE_DATA_NORMAL)
        {
            p->error_int += p->error_current ;	
        }
        /*非线性变速积分*/
        else if (p->intmode & PID_INTMODE_DATA_VAR_NONLINEAR)
        {
            intC = 1 / (p->intk_nonline * fabsf(p->error_current) + 1);
            p->error_int += intC * p->error_current;
        }
        /*积分分离*/
        if (p->intmode & PID_INTMODE_SEPARATION)
        {
            if (fabsf(p->error_current) > p->int_sep_threshold) {p->error_int = 0;}
        }
        /*积分限幅，防止积分饱和*/
        if (p->intmode & PID_INTMODE_LIMIT)	    
        {
            if (p->error_int > p->error_intmax) {p->error_int = p->error_intmax;}	
            if (p->error_int < p->error_intmin) {p->error_int = p->error_intmin;}	
        }
    }
    else							            //否则
    {
        p->error_int = 0;			            //误差积分直接归0
    }
    
    /*微分*/
    {
        /*正常微分*/
        if (p->difmode & PID_DIFMODE_NORMAL)
        {
            difout = p->kd * (p->error_current - p->error_past);
        }
        /*微分先行，防止目标值突变引起的微分冲击*/
        else if (p->difmode & PID_DIFMODE_FRONT)
        {
            difout = - p->kd * (p->actual_current - p->actual_past);
            p->actual_past = p->actual_current;				        //获取上次实际值
        }
        /*不完全微分,微分滤波，平滑数据但实际产生延时*/
        else if (p->difmode & PID_DIFMODE_IMPERFECT)
        {
            difout = (1 - p->dif_filter) * p->kd * (p->error_current - p->error_past) + p->dif_filter * p->dif_filter * difout;
        }
    }

	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->out = p->kp * p->error_current
		   + p->ki * p->error_int
		   + difout;
	
    /*输出偏移，防止死区*/
    if      (p->out > 0) {p->out += p->out_offset;}	//如果输出值大于0，则在输出值基础上加上输出最小值，防止死区
    else if (p->out < 0) {p->out -= p->out_offset;}	//如果输出值小于0，则在输出值基础上加上输出最大值，防止死区
    else                 {p->out = 0;}				//如果输出值等于0，则直接将输出值置0

	/*输出限幅*/
	if (p->out > p->outmax) {p->out = p->outmax;}	//限制输出值最大为结构体指定的OutMax
	if (p->out < p->outmin) {p->out = p->outmin;}	//限制输出值最小为结构体指定的OutMin

    /*输入死区，防止*/
    if (fabsf(p->error_current) < p->out_deadzone) {p->out = 0;}	//如果误差值小于输出偏移值，则直接将输出值置0，防止死区
}
