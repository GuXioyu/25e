/**
 * @file    My_Uart.c
 * @brief   多路串口底层驱动实现
 *
 * @note    本文件负责：
 *          - 维护 UART 句柄到数组下标的固定映射表
 *          - 提供中断接收字节缓存与 RX 重使能机制
 *          - 封装 HAL 阻塞式发送 + TC 超时等待
 *          - Serial_Init() 初始化后启动逐字节中断接收
 *
 *          中断回调 HAL_UART_RxCpltCallback 不在本文件中，
 *          而是在 My_Uart_Rx.c 中实现，方便单独修改协议逻辑。
 */

#include "stm32f4xx_hal.h"
#include "My_Uart.h"
#include "usart.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

/** @brief 每路串口最近一次中断接收的原始字节 */
volatile uint8_t Serial_RxData[SERIAL_RX_UART_COUNT];

/** @brief 每路串口接收完成标志（中断置位，上层读后清零） */
volatile uint8_t Serial_RxFlag[SERIAL_RX_UART_COUNT];

/** @brief 每路串口协议解析后的包体缓冲区 (volatile: ISR 与主循环共享) */
volatile char Serial_RxPacket[SERIAL_RX_UART_COUNT][100];

/*===========================================================================
 * 内部静态变量 —— 仅本文件可见
 *===========================================================================*/

/**
 * @brief 中断接收单字节缓存
 * @note  HAL_UART_Receive_IT 将接收到的字节写入此数组对应位置，
 *        协议层通过 Serial_GetRxByte() 读取
 */
static uint8_t Serial_RxByte[SERIAL_RX_UART_COUNT];

/**
 * @brief UART 句柄到数组下标的固定映射表
 * @note  通过 Serial_GetUartIndex() 做指针比较，O(n) 查表
 */
static UART_HandleTypeDef *const Serial_UartList[SERIAL_RX_UART_COUNT] =
{
	&huart1,
	&huart2,
	&huart3,
	&huart4,
	&huart5,
};

/*===========================================================================
 * 协议层专用函数
 *===========================================================================*/

/**
 * @brief  将 UART 句柄映射为数组下标
 * @param  huart  HAL 串口句柄指针
 * @retval 有效下标（0..4），未匹配则返回 SERIAL_RX_UART_COUNT
 */
uint8_t Serial_GetUartIndex(UART_HandleTypeDef *huart)
{
	for (uint8_t i = 0; i < SERIAL_RX_UART_COUNT; i++)
	{
		if (huart == Serial_UartList[i])
		{
			return i;
		}
	}
	return SERIAL_RX_UART_COUNT;
}

/**
 * @brief  读取指定下标的中断接收字节
 * @param  uartIndex  串口下标
 * @return 字节缓存中对应位置的值
 */
uint8_t Serial_GetRxByte(uint8_t uartIndex)
{
	return Serial_RxByte[uartIndex];
}

/**
 * @brief  通过寄存器直接操作重新使能 RX 中断
 * @param  huart      HAL 串口句柄指针
 * @param  uartIndex  串口下标
 * @note   绕过 HAL_UART_Receive_IT 的句柄锁（Lock）和 RxState 状态检查，
 *         直接设置 pRxBuffPtr / RxXferSize / RxXferCount / RxState 并
 *         置位 RXNEIE、PEIE、EIE，避免 HAL_BUSY 导致 RX 永久失效
 */
void Serial_ReArmRxIT(UART_HandleTypeDef *huart, uint8_t uartIndex)
{
	huart->pRxBuffPtr  = &Serial_RxByte[uartIndex];
	huart->RxXferSize  = 1U;
	huart->RxXferCount = 1U;
	huart->RxState     = HAL_UART_STATE_BUSY_RX;
	huart->ErrorCode   = HAL_UART_ERROR_NONE;

	/* 使能 RXNE（接收非空）、PE（校验错）、EIE（错误）中断 */
	SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
	SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
	SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
}

/*===========================================================================
 * 通用 API
 *===========================================================================*/

/**
 * @brief  初始化指定串口的接收状态，并挂起首字节接收
 * @param  huart  HAL 串口句柄指针
 * @note   清零对应下标的所有接收状态后，调用 HAL_UART_Receive_IT
 *         启动首次单字节中断接收；若 HAL 调用失败则死循环停机
 */
void Serial_Init(UART_HandleTypeDef *huart)
{
	uint8_t uartIndex = Serial_GetUartIndex(huart);
	if (uartIndex >= SERIAL_RX_UART_COUNT)
	{
		return;
	}

	/* 清零该路串口所有接收状态 */
	Serial_RxFlag[uartIndex]      = 0;
	Serial_RxData[uartIndex]      = 0;
	Serial_RxPacket[uartIndex][0] = '\0';

	/* 启动首次单字节中断接收，失败则停机 */
	if (HAL_UART_Receive_IT(huart, &Serial_RxByte[uartIndex], 1) != HAL_OK)
	{
		while (1) {}
	}
}

/**
 * @brief  读取并清除指定串口的接收完成标志
 * @param  huart  HAL 串口句柄指针
 * @retval 1 收到完整数据包
 * @retval 0 暂无新包
 */
uint8_t Serial_GetRxFlag(UART_HandleTypeDef *huart)
{
	uint8_t uartIndex = Serial_GetUartIndex(huart);
	if (uartIndex >= SERIAL_RX_UART_COUNT)
	{
		return 0;
	}

	if (Serial_RxFlag[uartIndex] != 0)
	{
		Serial_RxFlag[uartIndex] = 0; /* 读后自清，防止重复处理 */
		return 1;
	}

	return 0;
}

/**
 * @brief  获取指定串口解析后的数据包指针
 * @param  huart  HAL 串口句柄指针
 * @return 指向包体字符串的指针，失败返回 NULL
 */
volatile char *Serial_GetRxPacket(UART_HandleTypeDef *huart)
{
	uint8_t uartIndex = Serial_GetUartIndex(huart);
	if (uartIndex >= SERIAL_RX_UART_COUNT)
	{
		return NULL;
	}

	return Serial_RxPacket[uartIndex];
}

/**
 * @brief  格式化字符串发送到指定串口
 * @param  huart  HAL 串口句柄指针
 * @param  format 格式化字符串
 * @note   内部使用 vsnprintf 格式化到 128 字节栈缓冲区，
 *         再调用 Serial_SendData 阻塞发送
 */
void Serial_Printf(UART_HandleTypeDef *huart, char *format, ...)
{
	char string[128];
	va_list arg;

	if (Serial_GetUartIndex(huart) >= SERIAL_RX_UART_COUNT)
	{
		return;
	}

	va_start(arg, format);
	vsnprintf(string, sizeof(string), format, arg);
	va_end(arg);

	Serial_SendData(huart, (const uint8_t *)string, (uint16_t)strlen(string));
}

/**
 * @brief  向指定串口发送原始字节数据
 * @param  huart  HAL 串口句柄指针
 * @param  buffer 待发送数据缓冲区
 * @param  length 发送字节数
 * @retval 实际发送字节数，失败返回 0
 * @note   分两步：HAL_UART_Transmit 阻塞发送 + 等待 TC 标志确保移位寄存器为空
 */
uint16_t Serial_SendData(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t length)
{
	/* 参数校验 */
	if (buffer == NULL || length == 0)
	{
		return 0;
	}

	/* 句柄合法性 */
	if (Serial_GetUartIndex(huart) >= SERIAL_RX_UART_COUNT)
	{
		return 0;
	}

	/* 阻塞发送 */
	if (HAL_UART_Transmit(huart, (uint8_t *)buffer, length, 1000) != HAL_OK)
	{
		return 0;
	}

	/* 等待移位寄存器清空（TC），防止提前切方向（如 RS485） */
	if (Serial_WaitTransmitComplete(huart, 1000) == 0)
	{
		return 0;
	}

	return length;
}

/**
 * @brief  等待指定串口的 TC（发送完成）标志置位
 * @param  huart   HAL 串口句柄指针
 * @param  timeout 超时时间（ms）
 * @retval 1 发送完成
 * @retval 0 超时
 * @note   TC（Transmit Complete）标志表明数据已从移位寄存器完全移出，
 *         比 TXE（发送缓冲区空）更能保证数据已真正发送到线上
 */
uint8_t Serial_WaitTransmitComplete(UART_HandleTypeDef *huart, uint32_t timeout)
{
	uint32_t tickstart = HAL_GetTick();

	while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
	{
		if ((HAL_GetTick() - tickstart) >= timeout)
		{
			return 0;
		}
	}

	return 1;
}
