#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <math.h>

/*积分策略*/
/*intmode = PID_INTMODE_DATA_NORMAL | PID_INTMODE_LIMIT*/
#define PID_INTMODE_DATA_NORMAL        (0x01U)  /* 正常积分 */
#define PID_INTMODE_DATA_VAR_NONLINEAR (0x02U)  /* 变速非线性积分 */
#define PID_INTMODE_SEPARATION         (0x04U)  /* 积分分离 */
#define PID_INTMODE_LIMIT              (0x08U)  /* 积分限幅 */

/*微分策略*/
/*difmode = PID_DIFMODE_NORMAL | PID_DIFMODE_FRONT | PID_DIFMODE_IMPERFECT*/
#define PID_DIFMODE_NORMAL               (0x01U)  /* 普通微分 */
#define PID_DIFMODE_FRONT                (0x02U)  /* 微分先行 */
#define PID_DIFMODE_IMPERFECT            (0x03U)  /* 不完全微分 */

typedef struct {
	float kp;
	float ki;
	float kd;

	float target;
	float actual_current;
	float actual_past;
	float out;
	
	float error_current;
	float error_past;
	float error_int;

	uint8_t intmode;
	float intk_nonline;
	float int_sep_threshold;
	float error_intmax;
	float error_intmin;
	
	uint8_t difmode;
	float dif_filter;		//取值0-1，值越大微分滤波效果越明显，但实际产生的延时也越大

	float out_offset;
	float outmax;
	float outmin;
	float out_deadzone;
} PID_t;

void PID_Update(PID_t *p);

#endif
