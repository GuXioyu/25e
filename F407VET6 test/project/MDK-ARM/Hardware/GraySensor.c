/**
 * @file    GraySensor.c
 * @brief   灰度循迹传感器 —— 8路收发实现
 *
 * @note    协议（8路模拟量）：主机发送 2 字节，传感器回包 18 字节
 *          主机发送：| 0x57 | ID |
 *          主机接收：| 0x75 | Data0~15（16字节） | 0x11 |
 *
 *          Data0-1:   第 1 路模拟值（高8位在前）
 *          Data2-3:   第 2 路模拟值（高8位在前）
 *          ...
 *          Data14-15: 第 8 路模拟值（高8位在前）
 *          0x11:      8路模拟量帧尾
 */

#include "GraySensor.h"
#include "../User/My_Uart.h"
#include "My_Flash.h"

#ifndef __disable_irq
#define __disable_irq() ((void)0)
#endif

#ifndef __enable_irq
#define __enable_irq() ((void)0)
#endif

#define GRAYSENSOR_WHITE_THRESHOLD_GAP 100U

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

/**
 * @brief  根据当前 8 路模拟量和阈值生成数字量位图。
 * @param  threshold  黑线阈值
 * @return bit0~bit7 对应通道 0~7。
 * @note   白底黑线带回差判断：
 *         - 模拟量 < threshold：判定黑线，输出 1
 *         - 模拟量 > threshold + 100：判定白底，输出 0
 *         - 中间区间：保持上一次数字量状态，减少阈值附近抖动
 *         默认 threshold=700 时，即 <700 为黑 1，>800 为白 0。
 */
static uint8_t GraySensor_BuildDigital(uint16_t threshold)
{
	uint8_t digital = gsData.digital;
	uint32_t whiteThreshold = (uint32_t)threshold + GRAYSENSOR_WHITE_THRESHOLD_GAP;

	if (whiteThreshold > 65535U)
	{
		whiteThreshold = 65535U;
	}

	for (uint8_t ch = 0; ch < 8; ch++)
	{
		if (gsData.analog[ch] < threshold)
		{
			digital |= (uint8_t)(1U << ch);
		}
		else if (gsData.analog[ch] > whiteThreshold)
		{
			digital &= (uint8_t)~(uint8_t)(1U << ch);
		}
	}

	return digital;
}

/**
 * @brief 发送查询帧到传感器（主上下文调用）
 * @note 该函数会调用串口发送助手，不要在 ISR 中调用
 */
void GraySensor_SendQuery(UART_HandleTypeDef *huart)
{
	uint8_t cmd[2] = {0x57, 0x01};
	Serial_SendData(huart, cmd, 2);
}

/**
 * @brief 逐字节解析传感器回包（ISR-safe）
 * @param byte 单字节输入
 *
 * 解析流程：
 *  - 等待 0x75 帧头
 *  - 收集 16 字节模拟量数据（Data0..Data15）
 *  - 期望尾字节 0x11；若尾字正确则解析到 `gsData.analog`
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
		if (gsIdx >= 16)
		{
			gsState = GS_TAIL;
		}
		break;

	case GS_TAIL:
		/* 回到同步状态（无论帧尾是否正确） */
		gsState = GS_WAIT_75;

		if (byte == 0x11) /* 期望的帧尾：8 路模拟量 */
		{
			/* 解析字段，保持操作尽可能简单 */
			/* 8 路模拟值：Data0-1, Data2-3, ..., Data14-15 */
			for (uint8_t ch = 0; ch < 8; ch++)
			{
				gsData.analog[ch] = (uint16_t)(((uint16_t)gsBuf[ch * 2] << 8)
											  | gsBuf[ch * 2 + 1]);
			}
			gsData.digital = GraySensor_BuildDigital(myFlashData.gray_threshold);
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
 * @note   白底黑线规则：模拟量小于灰度阈值时输出 1。
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
 * @brief  读取 8 路灰度数字量位图。
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

/**
 * @brief  读取当前灰度阈值。
 */
uint16_t GraySensor_GetThreshold(void)
{
	return myFlashData.gray_threshold;
}

/**
 * @brief  修改 RAM 中的灰度阈值，并立即按新阈值刷新数字量。
 * @param  threshold  灰度阈值；白底黑线时，模拟量小于该值输出 1。
 */
void GraySensor_SetThreshold(uint16_t threshold)
{
	__disable_irq();
	myFlashData.gray_threshold = threshold;
	gsData.digital = GraySensor_BuildDigital(threshold);
	__enable_irq();
}

/**
 * @brief  从 My_Flash 读取灰度阈值，并刷新数字量。
 */
void GraySensor_LoadThreshold(void)
{
	My_Flash_Load();

	__disable_irq();
	gsData.digital = GraySensor_BuildDigital(myFlashData.gray_threshold);
	__enable_irq();
}

/**
 * @brief  将当前灰度阈值写入 My_Flash，实现掉电保存。
 * @return 1=保存成功，0=保存失败。
 */
uint8_t GraySensor_SaveThreshold(void)
{
	return My_Flash_Save();
}

/**
 * @brief  修改灰度阈值并立即写入 My_Flash，实现掉电保存。
 * @param  threshold  灰度阈值；白底黑线时，模拟量小于该值输出 1。
 * @return 1=修改并保存成功，0=保存失败。
 */
uint8_t GraySensor_SetAndSaveThreshold(uint16_t threshold)
{
	GraySensor_SetThreshold(threshold);
	return GraySensor_SaveThreshold();
}
