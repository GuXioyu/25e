#include <stddef.h>

#include "Emm_V5.h"
#include "Motor.h"

/*===========================================================================
 * 内部宏定义
 *===========================================================================*/

#define MOTOR_DIR_CW       0U
#define MOTOR_DIR_CCW      1U
#define MOTOR_POS_ABSOLUTE 1U
#define MOTOR_FRAME_TAIL   0x6BU
#define MOTOR_CMD_POS      0x36U
#define MOTOR_CMD_STATUS   0x3AU

/*===========================================================================
 * 全局数据
 *===========================================================================*/

volatile Motor_Rx_t Motor_Rx;

/*===========================================================================
 * 内部工具函数
 *===========================================================================*/

/* 获取有符号速度的绝对值，并限制到协议允许范围 */
static uint16_t Motor_AbsLimitInt16(int16_t value)
{
	int32_t temp = value;

	if (temp < 0)
	{
		temp = -temp;
	}

	if (temp > (int32_t)MOTOR_MAX_RPM)
	{
		temp = (int32_t)MOTOR_MAX_RPM;
	}

	return (uint16_t)temp;
}

/* 限制带正负速度，保证结构体缓存值和实际发送值一致 */
static int16_t Motor_LimitSignedSpeed(int16_t speed)
{
	if (speed > (int16_t)MOTOR_MAX_RPM)
	{
		return (int16_t)MOTOR_MAX_RPM;
	}

	if (speed < -(int16_t)MOTOR_MAX_RPM)
	{
		return -(int16_t)MOTOR_MAX_RPM;
	}

	return speed;
}

/* 角度只表示大小，方向由正负号单独转换为 dir */
static float Motor_AbsFloat(float value)
{
	return (value < 0.0f) ? -value : value;
}

/* 角度转 Emm_V5 位置模式需要的脉冲数 */
static uint32_t Motor_AngleToPulse(float angle_deg)
{
	float pulse;

	pulse = Motor_AbsFloat(angle_deg) / MOTOR_DEG_PER_PULSE;
	if (pulse >= 4294967295.0f)
	{
		return 0xFFFFFFFFU;
	}

	return (uint32_t)(pulse + 0.5f);
}

/* 根据电机安装方向设置转换实际发送方向 */
static uint8_t Motor_ApplyDirReverse(const Motor_t *motor, uint8_t dir)
{
	if (motor->dir_reverse != 0U)
	{
		return (dir == MOTOR_DIR_CW) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
	}

	return dir;
}

/*===========================================================================
 * 电机控制接口
 *===========================================================================*/

/* 初始化电机结构体变量，不发送使能命令 */
void Motor_Init(Motor_t *motor, uint8_t addr, uint8_t dir_reverse)
{
	if (motor == NULL)
	{
		return;
	}

	motor->addr = addr;
	motor->mode = MOTOR_MODE_STOP;
	motor->target_speed = 0;
	motor->target_angle = 0.0f;
	motor->dir_reverse = (dir_reverse == 0U) ? 0U : 1U;
}

/* 特殊接口：发送使能电机命令 */
void Motor_Enable(Motor_t *motor)
{
	if (motor == NULL)
	{
		return;
	}

	Emm_V5_En_Control(motor->addr, 1U, MOTOR_DEFAULT_SYNC);
}

/* 特殊接口：发送失能电机命令 */
void Motor_Disable(Motor_t *motor)
{
	if (motor == NULL)
	{
		return;
	}

	Emm_V5_En_Control(motor->addr, 0U, MOTOR_DEFAULT_SYNC);
}

/* 停止电机 */
void Motor_Stop(Motor_t *motor)
{
	if (motor == NULL)
	{
		return;
	}

	Emm_V5_Stop_Now(motor->addr, MOTOR_DEFAULT_SYNC);
	motor->mode = MOTOR_MODE_STOP;
	motor->target_speed = 0;
}

/* 速度模式：输入带正负 RPM，自动判断方向 */
void Motor_SetSpeed(Motor_t *motor, int16_t speed_rpm)
{
	uint8_t dir;
	uint16_t rpm;

	if (motor == NULL)
	{
		return;
	}

	dir = (speed_rpm < 0) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
	dir = Motor_ApplyDirReverse(motor, dir);
	rpm = Motor_AbsLimitInt16(speed_rpm);

	Emm_V5_Vel_Control(motor->addr, dir, rpm, MOTOR_DEFAULT_ACC, MOTOR_DEFAULT_SYNC);

	motor->mode = MOTOR_MODE_SPEED;
	motor->target_speed = Motor_LimitSignedSpeed(speed_rpm);
	motor->target_angle = 0.0f;
}

/* 绝对位置模式：输入目标角度，内部换算为脉冲数 */
void Motor_SetAbsAngle(Motor_t *motor, float angle_deg)
{
	uint8_t dir;
	uint32_t pulse;

	if (motor == NULL)
	{
		return;
	}

	dir = (angle_deg < 0.0f) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
	dir = Motor_ApplyDirReverse(motor, dir);
	pulse = Motor_AngleToPulse(angle_deg);

	Emm_V5_Pos_Control(motor->addr,
	                   dir,
	                   MOTOR_DEFAULT_POS_RPM,
	                   MOTOR_DEFAULT_ACC,
	                   pulse,
	                   MOTOR_POS_ABSOLUTE,
	                   MOTOR_DEFAULT_SYNC);

	motor->mode = MOTOR_MODE_POSITION;
	motor->target_speed = 0;
	motor->target_angle = angle_deg;
}

/*===========================================================================
 * 内部状态机与协议解析实现
 *===========================================================================*/

static void Motor_ParseStatusFrame(const uint8_t *frame)
{
	uint8_t status = frame[2];

	Motor_Rx.addr = frame[0];
	Motor_Rx.status = status;
	Motor_Rx.ens_tf = (status & 0x01U) ? 1U : 0U;
	Motor_Rx.prf_tf = (status & 0x02U) ? 1U : 0U;
	Motor_Rx.cgi_tf = (status & 0x04U) ? 1U : 0U;
	Motor_Rx.cgp_tf = (status & 0x08U) ? 1U : 0U;
	Motor_Rx.esi_lf = (status & 0x10U) ? 1U : 0U;
	Motor_Rx.esi_rf = (status & 0x20U) ? 1U : 0U;
	Motor_Rx.oac_tf = (status & 0x80U) ? 1U : 0U;
	Motor_Rx.status_ready = 1U;
}

static void Motor_ParsePositionFrame(const uint8_t *frame)
{
	uint32_t pos;
	float angle;

	pos = ((uint32_t)frame[3] << 24) |
	      ((uint32_t)frame[4] << 16) |
	      ((uint32_t)frame[5] << 8) |
	      ((uint32_t)frame[6]);

	angle = (float)pos * MOTOR_POS_DEG_PER_CNT;
	if (frame[2] != 0U)
	{
		angle = -angle;
	}

	Motor_Rx.addr = frame[0];
	Motor_Rx.pos_sign = frame[2];
	Motor_Rx.pos_raw = pos;
	Motor_Rx.pos_angle = angle;
	Motor_Rx.pos_ready = 1U;
}

/* 串口逐字节解析电机回包：支持 0x36 实时位置帧和 0x3A 状态帧 */
void Motor_ProcessByte(uint8_t uartIndex, uint8_t byte)
{
	static uint8_t frame[8];
	static uint8_t index = 0U;
	static uint8_t expect_len = 0U;

	(void)uartIndex;

	if (index == 0U)
	{
		frame[index++] = byte;
		return;
	}

	if (index == 1U)
	{
		frame[index++] = byte;

		if (byte == MOTOR_CMD_STATUS)
		{
			expect_len = 4U;
		}
		else if (byte == MOTOR_CMD_POS)
		{
			expect_len = 8U;
		}
		else
		{
			index = 0U;
			expect_len = 0U;
		}
		return;
	}

	frame[index++] = byte;

	if ((expect_len != 0U) && (index >= expect_len))
	{
		if (frame[expect_len - 1U] == MOTOR_FRAME_TAIL)
		{
			if (frame[1] == MOTOR_CMD_STATUS)
			{
				Motor_ParseStatusFrame(frame);
			}
			else if (frame[1] == MOTOR_CMD_POS)
			{
				Motor_ParsePositionFrame(frame);
			}
		}

		index = 0U;
		expect_len = 0U;
	}

	if (index >= sizeof(frame))
	{
		index = 0U;
		expect_len = 0U;
	}
}
