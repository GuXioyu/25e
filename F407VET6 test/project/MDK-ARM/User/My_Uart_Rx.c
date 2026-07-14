/**
 * @file    My_Uart_Rx.c
 * @brief   串口接收协议层 —— HAL_UART_RxCpltCallback 中断回调
 *
 * @note    本文件是项目中唯一的 HAL_UART_RxCpltCallback 实现，
 *          覆盖 HAL 库的 __weak 默认定义。
 *
 *          职责：
 *          - 从底层驱动（My_Uart.c）读取本次接收字节
 *          - 执行 '[' ... ']' 帧协议解析状态机
 *          - 收到完整包后置位 Serial_RxFlag 通知上层
 *
 *          若需改用其他协议（Modbus、自定义帧头尾等），
 *          只需修改本文件的状态机逻辑，My_Uart.c 无需变动。
 */

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "My_Uart.h"
#include "My_Uart_Rx.h"
#include "Motor.h"
#include "../Hardware/GraySensor.h"
#include "../Hardware/HWT101.h"

void Emm_V5_ProcessRx(uint8_t byte);

/**
 * @brief  UART 接收完成中断回调 —— 覆盖 HAL 弱定义
 * @param  huart  触发中断的 UART 句柄
 *
 * @note   协议格式：[包体内容]
 *          - '[' = 帧头，进入接收状态
 *          - ']' = 帧尾，封包并置位 Serial_RxFlag
 *          - 包体中遇到新的 '[' → 前一包异常，丢弃并重新同步
 *          - 包体超过 99 字节 → 整包丢弃，回到等待帧头
 *
 *          RX 重使能通过底层 Serial_ReArmRxIT() 寄存器直写完成，
 *          不经过 HAL_UART_Receive_IT，避免句柄锁导致死锁。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* 状态机变量 —— 每路串口独立，static 保持跨中断状态 */
	static uint8_t rxState[SERIAL_RX_UART_COUNT]  = {0};   /**< 0=等待'['  1=接收包体 */
	static uint8_t rxPktIdx[SERIAL_RX_UART_COUNT] = {0};   /**< 包体写入索引       */

	/* 获取触发中断的串口下标 */
	uint8_t uartIndex = Serial_GetUartIndex(huart);
	if (uartIndex >= SERIAL_RX_UART_COUNT)
	{
		return; /* 不支持的串口，忽略 */
	}

	/* 从底层缓存读取本次收到的字节 */
	uint8_t byte = Serial_GetRxByte(uartIndex);
	Serial_RxData[uartIndex] = byte;

	/* 绕过 HAL 锁重使能 RXNE 中断，保证下一字节能正常接收 */
	Serial_ReArmRxIT(huart, uartIndex);

	/* 指定串口的专用协议处理 */
//	if (huart == &huart1)while(1);
//	if (huart == &huart2)while(1);
//	if (huart == &huart3)while(1);
//	if (huart == &huart4)while(1);
//	if (huart == &huart5)while(1);
	
	if (huart == &huart3)
	{
		Motor_ProcessByte(uartIndex, byte);
		return;
	}

	if (huart == &huart4)
	{
		GraySensor_ProcessByte(byte);
		return;
	}

	if (huart == &huart5)
	{
		HWT101_ProcessByte(byte);
		return;
	}

	/* =================================================================
	 * 协议状态机：逐字节解析 '[' ... ']' 帧
	 * ================================================================= */
	if (rxState[uartIndex] == 0)
	{
		/* ---------- 状态 0：等待帧头 '[' ---------- */
		if (byte == '[')
		{
			rxState[uartIndex]  = 1;                   /* 进入包体接收状态 */
			rxPktIdx[uartIndex] = 0;                   /* 复位写入指针       */
			Serial_RxPacket[uartIndex][0] = '\0';      /* 清空旧数据         */
		}
		/* 非 '[' 字节直接丢弃 */
	}
	else
	{
		/* ---------- 状态 1：接收包体 ---------- */
		if (byte == ']')
		{
			/* 帧尾：封包，通知上层 */
			Serial_RxPacket[uartIndex][rxPktIdx[uartIndex]] = '\0';
			Serial_RxFlag[uartIndex] = 1;
			rxState[uartIndex] = 0;                    /* 回到等待帧头 */
		}
		else if (byte == '[')
		{
			/* 包体中遇到 '[' → 异常，丢弃旧数据重新同步 */
			rxPktIdx[uartIndex] = 0;
			Serial_RxPacket[uartIndex][0] = '\0';
			/* rxState 保持 1，继续接收 */
		}
		else
		{
			/* 普通数据字节：写入包体缓冲区 */
			if (rxPktIdx[uartIndex] < (uint8_t)(sizeof(Serial_RxPacket[uartIndex]) - 1U))
			{
				Serial_RxPacket[uartIndex][rxPktIdx[uartIndex]] = (char)byte;
				rxPktIdx[uartIndex]++;
				Serial_RxPacket[uartIndex][rxPktIdx[uartIndex]] = '\0'; /* 保持 '\0' 结尾 */
			}
			else
			{
				/* 包体超长（>=100 字节），整包丢弃 */
				rxState[uartIndex] = 0;
				Serial_RxPacket[uartIndex][0] = '\0';
			}
		}
	}
}

