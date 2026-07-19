#include "Motor.h"

/*===========================================================================
 * 内部宏定义
 *===========================================================================*/

#define MOTOR_DIR_CW       0U
#define MOTOR_DIR_CCW      1U
#define MOTOR_POS_RELATIVE 0U
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

/**
 * @brief 获取有符号速度的绝对值，并限制到协议允许范围。
 * @param value 带正负号的转速。
 * @return 限幅后的无符号转速。
 */
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

/**
 * @brief 限制带正负速度，保证结构体缓存值和实际发送值一致。
 * @param speed 上层输入的目标速度。
 * @return 限幅后的目标速度。
 */
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

/**
 * @brief 取浮点数绝对值。
 * @param value 输入浮点数。
 * @return 绝对值。
 */
static float Motor_AbsFloat(float value)
{
	return (value < 0.0f) ? -value : value;
}

/**
 * @brief 将角度换算为位置模式脉冲数。
 * @param angle_deg 输入角度。
 * @return 换算后的脉冲数。
 */
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

/**
 * @brief 根据电机安装方向修正发送方向。
 * @param motor 电机对象。
 * @param dir 原始方向。
 * @return 修正后的方向。
 */
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

/**
 * @brief 初始化电机结构体变量，不发送使能命令。
 * @param motor 电机对象。
 * @param addr 电机地址。
 * @param dir_reverse 方向是否对调。
 */
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

/**
 * @brief 最近一次输入值缓存（多通道）。
 */
static int32_t Motor_LastCompareValue[MOTOR_COMPARE_CHANNELS] = {0}; /* 各通道上次比较值，初始值为 0 */

/**
 * @brief 比较指定通道本次输入值与历史值，并保存本次输入值。
 * @param channel 通道编号，取值范围 0..(MOTOR_COMPARE_CHANNELS-1)。
 * @param value 本次输入值。
 * @return 本次值不同于历史值返回 1，相同或 channel 越界返回 0。
 */
uint8_t Motor_CompareLastValue(uint8_t channel, int32_t value) /* 判断通道输入值是否发生变化 */
{
	uint8_t result = 0U;

	if (channel >= MOTOR_COMPARE_CHANNELS)
	{
		return 0U; /* 无效通道直接返回 0，不改变任何历史值 */
	}

	result = (value != Motor_LastCompareValue[channel]) ? 1U : 0U; /* 与初始值或上次值不同则通知调用方 */
	Motor_LastCompareValue[channel] = value; /* 保存本次值作为下次比较基准 */

	return result;
}

/**
 * @brief 发送失能电机命令。
 * @param motor 电机对象。
 * @return 无。
 */
void Motor_Disable(Motor_t *motor)
{
	if (motor == NULL)
	{
		return;
	}

	Emm_V5_En_Control(motor->addr, 0U, MOTOR_DEFAULT_SYNC);
}

/**
 * @brief 立即停止电机。
 * @param motor 电机对象。
 * @return 无。
 */
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

/**
 * @brief 将电机当前位置设为单圈回零零点并保存。
 * @param motor 电机对象。
 * @return 无。
 */
void Motor_SetZero(Motor_t *motor) /* 设置并保存电机单圈回零零点 */
{
	if (motor == NULL) /* 检查电机对象是否有效 */
	{
		return; /* 空对象不发送设零命令 */
	}

	Emm_V5_Origin_Set_O(motor->addr, 1U); /* 将当前位置设为零点并保存至驱动器 */
}

/**
 * @brief 速度模式控制。
 * @param motor 电机对象。
 * @param speed_rpm 目标转速，带正负号。
 * @return 无。
 */
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

/**
 * @brief 绝对位置模式控制。
 * @param motor 电机对象。
 * @param angle_deg 目标角度。
 * @return 无。
 */
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

/**
 * @brief 相对位置模式控制。
 * @param motor 电机对象。
 * @param angle_deg 本次角度增量。
 * @return 无。
 */
void Motor_SetRelAngle(Motor_t *motor, float angle_deg)
{
	uint8_t dir;    /* 本次相对运动方向 */
	uint32_t pulse; /* 本次相对运动脉冲数 */

	if (motor == NULL)
	{
		return;
	}

	dir = (angle_deg < 0.0f) ? MOTOR_DIR_CCW : MOTOR_DIR_CW; /* 角度正负决定运动方向 */
	dir = Motor_ApplyDirReverse(motor, dir);                  /* 按电机安装方向修正运动方向 */
	pulse = Motor_AngleToPulse(angle_deg);                    /* 将角度增量换算为脉冲数 */

	Emm_V5_Pos_Control(motor->addr,
	                   dir,
	                   MOTOR_DEFAULT_POS_RPM,
	                   MOTOR_DEFAULT_ACC,
	                   pulse,
	                   MOTOR_POS_RELATIVE,
	                   MOTOR_DEFAULT_SYNC);

	motor->mode = MOTOR_MODE_POSITION; /* 记录当前使用位置模式 */
	motor->target_speed = 0;           /* 位置模式不保留速度目标 */
	motor->target_angle = angle_deg;   /* 记录本次相对角度增量 */
}

/*===========================================================================
 * 内部状态机与协议解析实现
 *===========================================================================*/



/**
 * @brief 解析状态帧。
 * @param frame 原始回包帧。
 * @return 无。
 */
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

/**
 * @brief 解析实时位置帧。
 * @param frame 原始回包帧。
 * @return 无。
 */
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

/**
 * @brief 串口逐字节解析电机回包。
 * @param uartIndex 串口编号。
 * @param byte 当前接收字节。
 * @return 无。
 */
void Motor_ProcessByte(uint8_t uartIndex, uint8_t byte)
{
	/* 逐字节组帧缓存：8 字节足够容纳当前支持的最长帧。 */
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

		/* 第二字节决定帧类型，按命令字确定后续期望长度。 */
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
		/* 仅当尾字节正确时才认为整帧有效。 */
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
