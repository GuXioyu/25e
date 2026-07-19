/**
 * @file    GraySensor.c
 * @brief   灰度循迹传感器 —— 8路收发实现
 *
 * @note    协议：主机发送 2 字节，根据查询类型接收数字量或模拟量回包
 *          主机发送：| 0x57 | ID |
 *          数字量接收：| 0x75 | Data0 | 0x02 |
 *          模拟量接收：| 0x75 | Data0~15（16字节） | 0x11 |
 *
 *          Data0-1:   第 1 路模拟值（高8位在前）
 *          Data2-3:   第 2 路模拟值（高8位在前）
 *          ...
 *          Data14-15: 第 8 路模拟值（高8位在前）
 *          0x02:      8路数字量帧尾
 *          0x11:      8路模拟量帧尾
 */

#include "GraySensor.h"

#ifndef __disable_irq
#define __disable_irq() ((void)0)
#endif

#ifndef __enable_irq
#define __enable_irq() ((void)0)
#endif

/*===========================================================================
 * 全局数据
 *
 * 说明：`gsData` 由解析函数在 ISR 或接收回调中更新；主循环必须在复制
 * 数据并清标志时进入临界区。请勿在 ISR 中对 `gsData` 做耗时处理。
 *===========================================================================*/

GraySensor_Data_t gsData = {0};

/*===========================================================================
 * 内部状态机与协议解析实现
 *===========================================================================*/

typedef enum
{
	GS_WAIT_75  = 0,  /**< 等待帧头 0x75              */
	GS_COLLECT  = 1,  /**< 收集 Data0 ~ Data15       */
	GS_TAIL     = 2   /**< 验证帧尾 0x11            */
} GS_State_t;

static GS_State_t gsState   = GS_WAIT_75;
static uint8_t    gsIdx     = 0;           /**< 数据收集索引 (0..15)      */
static uint8_t    gsBuf[16] = {0};         /**< Data0~Data15 缓存         */
static uint8_t    gsExpectedType = GRAYSENSOR_QUERY_ANALOG; /**< 最近一次有效查询所期望的回包类型。 */

/* 根据查询类型返回回包中的有效数据字节数。 */
static uint8_t GraySensor_GetPayloadLength(uint8_t type)
{
	return (type == GRAYSENSOR_QUERY_DIGITAL) ? 1U : 16U;
}

/* 根据查询类型返回对应的协议帧尾。 */
static uint8_t GraySensor_GetFrameTail(uint8_t type)
{
	return (type == GRAYSENSOR_QUERY_DIGITAL) ? 0x02U : 0x11U;
}

/**
 * @brief 发送查询帧到传感器（主上下文调用）
 * @note 该函数会调用串口发送助手，不要在 ISR 中调用
 */
void GraySensor_SendQuery(UART_HandleTypeDef *huart, uint8_t type)
{
	uint8_t cmd[2] = {0x57, 0x01};

	if (type != GRAYSENSOR_QUERY_DIGITAL && type != GRAYSENSOR_QUERY_ANALOG)
	{
		return; /* 查询类型无效时不发送命令，也不改变当前解析类型。 */
	}

	gsExpectedType = type;
	Serial_SendData(huart, cmd, 2);
}

/**
 * @brief 逐字节解析传感器回包（ISR-safe）
 * @param byte 单字节输入
 *
 * 解析流程：
 *  - 等待 0x75 帧头
 *  - 按查询类型收集 1 字节数字量或 16 字节模拟量数据
 *  - 校验对应尾字节；数字量直接更新位图，模拟量帧只更新模拟量数组，不更新数字量
 *
 * 约定：该函数应在中断上下文或串口逐字回调中被调用；绝不应调用会阻塞或
 * 引发 HAL 锁的操作。
 */
void GraySensor_ProcessByte(uint8_t byte)
{
	switch (gsState)
	{
	case GS_WAIT_75:
		if (byte == 0x75)
		{
			gsState = GS_COLLECT;
			gsIdx   = 0;
			memset(gsBuf, 0, sizeof(gsBuf));
		}
		else
		{
			gsState = GS_WAIT_75; /* 帧头异常，重新同步 */
		}
		break;

	case GS_COLLECT:
		gsBuf[gsIdx++] = byte;
		if (gsIdx >= GraySensor_GetPayloadLength(gsExpectedType))
		{
			gsState = GS_TAIL;
		}
		break;

	case GS_TAIL:
		/* 回到同步状态（无论帧尾是否正确） */
		gsState = GS_WAIT_75;

		if (byte == GraySensor_GetFrameTail(gsExpectedType))
		{
			if (gsExpectedType == GRAYSENSOR_QUERY_DIGITAL)
			{
				gsData.digital = gsBuf[0]; /* Data0 的 bit0~bit7 对应第 1~8 路。 */
			}
			else
			{
				/* 8 路模拟值：Data0-1, Data2-3, ..., Data14-15。 */
				for (uint8_t ch = 0; ch < 8; ch++)
				{
					gsData.analog[ch] = (uint16_t)(((uint16_t)gsBuf[ch * 2] << 8)
												  | gsBuf[ch * 2 + 1]);
				}
			}
		}

		/* 帧尾错误：静默丢弃，gsData 不更新 */
		break;
	}
}

/**
 * @brief  复位状态机（异常恢复时调用）
 */
void GraySensor_Reset(void)
{
	gsState    = GS_WAIT_75;
	gsIdx      = 0;
	gsExpectedType = GRAYSENSOR_QUERY_ANALOG;
	gsData.digital = 0;
	memset(gsBuf, 0, sizeof(gsBuf));
}

/**
 * @brief 从 gsData 读取指定通道的模拟量（原子读）
 * @param ch 通道索引 (0..7)
 * @return 该通道模拟值，ch 越界则返回 0
 */
uint16_t GraySensor_GetAnalog(uint8_t ch)
{
	uint16_t val = 0;
	if (ch >= 8) return 0;
	__disable_irq();
	val = gsData.analog[ch];
	__enable_irq();
	return val;
}
/**
 * @brief  读取指定通道的灰度数字量。
 * @param  ch  通道索引 (0..7)
 * @return 1=检测到黑线，0=白底或通道越界
 * @note   读取最近一次校验成功的原生数字量帧。
 */
uint8_t GraySensor_GetDigital(uint8_t ch)
{
	uint8_t val;

	if (ch >= 8)
	{
		return 0;
	}

	__disable_irq();
	val = (uint8_t)((gsData.digital >> ch) & 0x01U);
	__enable_irq();

	return val;
}

/**
 * @brief  读取最近一次校验成功的原生数字量帧。
 * @return bit0~bit7 对应通道 0~7，位为 1 表示该路检测到黑线。
 */
uint8_t GraySensor_GetDigitalByte(void)
{
	uint8_t val;

	__disable_irq();
	val = gsData.digital;
	__enable_irq();

	return val;
}
