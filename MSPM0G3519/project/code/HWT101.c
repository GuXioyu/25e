/**
 * @file    HWT101.c
 * @brief   HWT101 姿态传感器数据解析
 *
 * @note    协议帧固定 11 字节：
 *          | 0x55 | TYPE | Data0~Data7 | SUM |
 *
 *          TYPE = 0x52：角速度帧
 *          Data4-5: 角速度 Z 原始值，低字节在前
 *          gyroZ = rawGyroZ / 32768 * 2000 (deg/s)
 *
 *          TYPE = 0x53：角度帧
 *          Data4-5: 偏航角 Z 原始值，低字节在前
 *          Data6-7: 版本号，低字节在前
 *          yaw = rawYaw / 32768 * 180 (deg)
 *
 *          SUM = 前 10 字节累加和的低 8 位
 */

#include "HWT101.h"

#ifndef __disable_irq
#define __disable_irq() ((void)0)
#endif

#ifndef __enable_irq
#define __enable_irq() ((void)0)
#endif

#define HWT101_FRAME_HEAD       0x55U
#define HWT101_TYPE_GYRO        0x52U
#define HWT101_TYPE_ANGLE       0x53U
#define HWT101_FRAME_LENGTH     11U
#define HWT101_PAYLOAD_LENGTH   8U
#define HWT101_CMD_LENGTH       5U
#define HWT101_DELAY_UNLOCK_MS  200U
#define HWT101_DELAY_ZERO_MS    500U
#define HWT101_LAP_JUMP_DEG     90.0f

/*===========================================================================
 * 内部状态机
 *===========================================================================*/

typedef enum
{
	HWT_WAIT_HEAD = 0,  /**< 等待帧头 0x55        */
	HWT_COLLECT   = 1   /**< 收集剩余 10 字节     */
} HWT101_State_t;

/** @brief HWT101 最新解析数据 */
HWT101_Data_t hwt101Data = {0};

static const uint8_t hwt101UnlockCmd[HWT101_CMD_LENGTH] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t hwt101YawZeroCmd[HWT101_CMD_LENGTH] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
static const uint8_t hwt101SaveCmd[HWT101_CMD_LENGTH] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

static HWT101_State_t hwtState = HWT_WAIT_HEAD;           /**< 解析状态 */
static uint8_t hwtBuf[HWT101_FRAME_LENGTH] = {0};         /**< 帧缓存   */
static uint8_t hwtIdx = 0;                                /**< 写入索引 */
static int32_t hwtLapCount = 0;                           /**< Z 轴累计圈数 */
static float hwtLastYaw = 0.0f;                           /**< 上一次 Z 轴角度 */
static uint8_t hwtYawReady = 0;                           /**< Z 轴角度是否已初始化 */

/**
 * @brief  低字节在前，高字节在后，合成为有符号 16 位数据
 * @param  low   低 8 位
 * @param  high  高 8 位
 * @return 合成后的 int16_t 数据
 */
static int16_t HWT101_ToInt16(uint8_t low, uint8_t high)
{
	return (int16_t)(((uint16_t)high << 8) | low);
}

/**
 * @brief  计算 HWT101 校验和
 * @param  buf  11 字节帧缓存
 * @return 前 10 字节累加和的低 8 位
 */
static uint8_t HWT101_Checksum(const uint8_t *buf)
{
	uint16_t sum = 0;

	for (uint8_t i = 0; i < (HWT101_FRAME_LENGTH - 1U); i++)
	{
		sum += buf[i];
	}

	return (uint8_t)sum;
}

/**
 * @brief  根据 Z 轴角度突变更新累计圈数
 * @param  yaw  当前 Z 轴角度，单位 deg
 * @return 无
 */
static void HWT101_UpdateLapCount(float yaw)
{
	/* 首帧只记录角度基准，不参与圈数判断 */
	if (hwtYawReady == 0U)
	{
		hwtLastYaw = yaw;
		hwtYawReady = 1U;
		return;
	}

	/* 判断 +180 到 -180 的正向跨界 */
	if ((hwtLastYaw > HWT101_LAP_JUMP_DEG) && (yaw < -HWT101_LAP_JUMP_DEG))
	{
		hwtLapCount++;
	}
	/* 判断 -180 到 +180 的反向跨界 */
	else if ((hwtLastYaw < -HWT101_LAP_JUMP_DEG) && (yaw > HWT101_LAP_JUMP_DEG))
	{
		hwtLapCount--;
	}

	/* 保存本帧角度，供下一帧比较 */
	hwtLastYaw = yaw;
}

/**
 * @brief  校验并解析完整 11 字节帧
 * @param  buf  11 字节帧缓存
 * @note   校验失败或 TYPE 不支持时静默丢弃，不更新 hwt101Data。
 */
static void HWT101_ParseFrame(const uint8_t *buf)
{
	if (HWT101_Checksum(buf) != buf[10])
	{
		return;
	}

	if (buf[1] == HWT101_TYPE_GYRO)
	{
		int16_t rawGyroZ = HWT101_ToInt16(buf[6], buf[7]);

		__disable_irq();
		hwt101Data.rawGyroZ = rawGyroZ;
		hwt101Data.gyroZ = (float)rawGyroZ / 32768.0f * 2000.0f;
		__enable_irq();
	}
	else if (buf[1] == HWT101_TYPE_ANGLE)
	{
		int16_t rawYaw = HWT101_ToInt16(buf[6], buf[7]);
		uint16_t version = (uint16_t)(((uint16_t)buf[9] << 8) | buf[8]);
		float yaw = (float)rawYaw / 32768.0f * 180.0f;

		__disable_irq();
		HWT101_UpdateLapCount(yaw);
		hwt101Data.rawYaw = rawYaw;
		hwt101Data.yaw = yaw;
		hwt101Data.version = version;
		__enable_irq();
	}
}

/**
 * @brief  逐字节解析 HWT101 回包
 * @param  byte  接收到的单字节
 * @note   可在串口接收中断回调中调用，只做轻量状态机和内存写入。
 */
void HWT101_ProcessByte(uint8_t byte)
{
	switch (hwtState)
	{
	case HWT_WAIT_HEAD:
		if (byte == HWT101_FRAME_HEAD)
		{
			hwtBuf[0] = byte;
			hwtIdx = 1;
			hwtState = HWT_COLLECT;
		}
		break;

	case HWT_COLLECT:
		hwtBuf[hwtIdx++] = byte;
		if (hwtIdx >= HWT101_FRAME_LENGTH)
		{
			HWT101_ParseFrame(hwtBuf);
			hwtIdx = 0;
			hwtState = HWT_WAIT_HEAD;
		}
		break;

	default:
		HWT101_Reset();
		break;
	}
}

/**
 * @brief  复位 HWT101 解析状态机
 */
void HWT101_Reset(void)
{
	hwtState = HWT_WAIT_HEAD;
	hwtIdx = 0;

	for (uint8_t i = 0; i < HWT101_FRAME_LENGTH; i++)
	{
		hwtBuf[i] = 0;
	}
}

/**
 * @brief  读取角速度 Z 原始值
 * @return int16_t 原始值
 */
int16_t HWT101_GetRawGyroZ(void)
{
	int16_t val;

	__disable_irq();
	val = hwt101Data.rawGyroZ;
	__enable_irq();

	return val;
}

/**
 * @brief  读取角速度 Z
 * @return 单位 deg/s
 */
float HWT101_GetGyroZ(void)
{
	float val;

	__disable_irq();
	val = hwt101Data.gyroZ;
	__enable_irq();

	return val;
}

/**
 * @brief  读取偏航角 Z 原始值
 * @return int16_t 原始值
 */
int16_t HWT101_GetRawYaw(void)
{
	int16_t val;

	__disable_irq();
	val = hwt101Data.rawYaw;
	__enable_irq();

	return val;
}

/**
 * @brief  读取偏航角 Z
 * @return 单位 deg
 */
float HWT101_GetYaw(void)
{
	float val;

	__disable_irq();
	val = hwt101Data.yaw;
	__enable_irq();

	return val;
}

/**
 * @brief  读取版本号
 * @return 版本号，高字节在返回值高 8 位
 */
uint16_t HWT101_GetVersion(void)
{
	uint16_t val;

	__disable_irq();
	val = hwt101Data.version;
	__enable_irq();

	return val;
}

/**
 * @brief  读取 Z 轴累计圈数
 * @return 累计圈数，正数表示正向圈数，负数表示反向圈数
 */
int32_t HWT101_GetLapCount(void)
{
	int32_t val;

	__disable_irq();
	val = hwtLapCount;
	__enable_irq();

	return val;
}

/**
 * @brief  清零 Z 轴累计圈数
 * @return 无
 */
void HWT101_ClearLapCount(void)
{
	__disable_irq();
	/* 只清除累计圈数，保留上一帧角度用于继续判断跨界 */
	hwtLapCount = 0;
	__enable_irq();
}

/**
 * @brief  设置 Z 轴角度零点并保存
 * @param  huart  HWT101 所连接的 UART 句柄
 * @note   会阻塞等待 200ms + 500ms，主循环上下文调用，不要在中断中调用。
 */
void HWT101_SetYawZero(UART_HandleTypeDef *huart)
{
	if (huart == NULL)
	{
		return;
	}

	Serial_SendData(huart, hwt101UnlockCmd, HWT101_CMD_LENGTH);
	system_delay_ms(HWT101_DELAY_UNLOCK_MS);

	Serial_SendData(huart, hwt101YawZeroCmd, HWT101_CMD_LENGTH);
	system_delay_ms(HWT101_DELAY_ZERO_MS);

	Serial_SendData(huart, hwt101SaveCmd, HWT101_CMD_LENGTH);

	__disable_irq();
	/* 设置零点后重新建立角度基准，不清除累计圈数 */
	hwtYawReady = 0U;
	__enable_irq();
}
