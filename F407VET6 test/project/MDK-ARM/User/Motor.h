#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* 速度命令最大值：Emm_V5 速度模式支持 0~5000 RPM，超过会自动限幅 */
#define MOTOR_MAX_RPM 5000U

/* 绝对位置模式默认转速：调用 Motor_SetAbsAngle() 时使用，单位 RPM */
#define MOTOR_DEFAULT_POS_RPM 100U

/* 每个脉冲对应的角度：1.8 度步距角 / 256 细分 = 0.00703125 度 */
#define MOTOR_DEG_PER_PULSE   0.00703125f

/* EMM 固件实时位置返回值：0~65535 对应一圈 0~360 度 */
#define MOTOR_POS_DEG_PER_CNT  (360.0f / 65536.0f)

/* 默认加速度：调用速度/位置命令时使用；0 表示不加减速，直接启动 */
#define MOTOR_DEFAULT_ACC     0U

/* 默认同步标志：0=立即执行命令，1=等待 Emm_V5_Synchronous_motion() 同步触发 */
#define MOTOR_DEFAULT_SYNC    0U

/* 上层记录的电机工作模式 */
typedef enum
{
	MOTOR_MODE_STOP = 0U,    /* 停止/未运动 */
	MOTOR_MODE_SPEED,        /* 速度模式 */
	MOTOR_MODE_POSITION,     /* 位置模式 */
} Motor_Mode_t;

/* 电机对象，每个物理电机定义一个变量 */
typedef struct
{
	uint8_t addr;             /* 电机地址 */
	Motor_Mode_t mode;        /* 当前模式 */
	int16_t target_speed;     /* 目标速度，带正负，单位 RPM */
	float target_angle;       /* 目标绝对角度 */
	uint8_t dir_reverse;      /* 方向设置：0=默认方向，1=方向对调 */
} Motor_t;

/* 电机串口回包解析结果 */
typedef struct
{
	uint8_t addr;             /* 回包电机地址 */
	uint8_t pos_ready;        /* 收到实时位置帧标志 */
	uint8_t status_ready;     /* 收到状态帧标志 */

	uint8_t pos_sign;         /* 位置符号：0=正，1=负 */
	uint32_t pos_raw;         /* 原始实时位置 */
	float pos_angle;          /* 换算后的实时角度 */

	uint8_t status;           /* 原始状态字节 */
	uint8_t ens_tf;           /* 使能状态 */
	uint8_t prf_tf;           /* 到位状态 */
	uint8_t cgi_tf;           /* 堵转标志 */
	uint8_t cgp_tf;           /* 堵转保护标志 */
	uint8_t esi_lf;           /* 左限位状态 */
	uint8_t esi_rf;           /* 右限位状态 */
	uint8_t oac_tf;           /* 掉电标志 */
} Motor_Rx_t;

extern volatile Motor_Rx_t Motor_Rx;

/*
 * 使用示例：
 *
 * 1. 先定义电机变量，例如：
 *    Motor_t motor1;
 *    Motor_t motor2;
 *
 * 2. 初始化电机地址和方向：dir_reverse 为 0 使用默认方向，为 1 对调 CW/CCW
 *    Motor_Init(&motor1, 1, 0);   // motor1 对应地址 1，默认方向
 *    Motor_Init(&motor2, 2, 1);   // motor2 对应地址 2，方向对调
 *
 * 3. 速度模式：
 *    Motor_SetSpeed(&motor1, 800);    // 正数：CW 方向，800 RPM
 *    Motor_SetSpeed(&motor1, -800);   // 负数：CCW 方向，800 RPM
 *
 * 4. 绝对位置模式：
 *    Motor_SetAbsAngle(&motor1, 90.0f);    // 转到绝对 90 度位置
 *    Motor_SetAbsAngle(&motor1, -90.0f);   // 反方向转到绝对 90 度位置
 *
 * 5. 停止/失能：
 *    Motor_Stop(&motor1);
 *    Motor_Disable(&motor1);
 */

/* 初始化电机对象：addr 为电机地址，dir_reverse 为 0 默认方向、1 方向对调 */
void Motor_Init(Motor_t *motor, uint8_t addr, uint8_t dir_reverse);

/* 特殊接口：发送使能命令 */
void Motor_Enable(Motor_t *motor);

/* 特殊接口：发送失能命令 */
void Motor_Disable(Motor_t *motor);

/* 立即停止电机 */
void Motor_Stop(Motor_t *motor);

/* 速度模式：speed_rpm 带正负，正数=CW，负数=CCW */
void Motor_SetSpeed(Motor_t *motor, int16_t speed_rpm);

/* 绝对位置模式：angle_deg 为目标角度，内部换算为脉冲数 */
void Motor_SetAbsAngle(Motor_t *motor, float angle_deg);

/* 串口逐字节解析电机回包 */
void Motor_ProcessByte(uint8_t uartIndex, uint8_t byte);

#endif /* __MOTOR_H */
