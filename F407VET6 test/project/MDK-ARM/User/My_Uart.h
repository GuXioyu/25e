/**
 * @file    My_Uart.h
 * @brief   多路串口底层驱动 —— 通用收发 API 与协议层接口
 *
 * @note    业务代码仅需 #include "My_Uart.h" 即可使用全部收发功能。
 *          协议层（My_Uart_Rx.c）通过本文件的协议层专用接口操作硬件。
 *
 * @warning  需确保项目中只有一个 HAL_UART_RxCpltCallback 定义
 */

#ifndef __MY_UART_H
#define __MY_UART_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

#ifndef SERIAL_RX_UART_COUNT
#define SERIAL_RX_UART_COUNT 5    /**< 最大串口路数，可全局覆盖 */
#endif

/*===========================================================================
 * 全局变量（供 ISR 与主循环共享）
 *===========================================================================*/

/* 最近接收字节：ISR 写入，主循环可读取 */
extern volatile uint8_t Serial_RxData[SERIAL_RX_UART_COUNT];

/* 接收完成标志：ISR 置位，主循环读后清零 */
extern volatile uint8_t Serial_RxFlag[SERIAL_RX_UART_COUNT];

/* 解析后包体缓冲区：由 ISR 写入，主循环在读取并处理前应先通过
	Serial_GetRxFlag 确认并清零标志以保证同步。为保持 API 兼容，
	此处保留非 volatile 声明（避免改变返回类型）；访问时遵循
	读标志后处理的同步约定。 */
/* 解析后包体缓冲区：由 ISR 写入，主循环在读取并处理前应先通过
	Serial_GetRxFlag 确认并清零标志以保证同步。
	声明为 volatile 以便在中断/主循环间可见性正确。 */
extern volatile char    Serial_RxPacket[SERIAL_RX_UART_COUNT][100];

/*===========================================================================
 * 通用 API
 *===========================================================================*/

void     Serial_Init(UART_HandleTypeDef *huart);
uint8_t  Serial_GetRxFlag(UART_HandleTypeDef *huart);
volatile char *Serial_GetRxPacket(UART_HandleTypeDef *huart);
void     Serial_Printf(UART_HandleTypeDef *huart, char *format, ...);
uint16_t Serial_SendData(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t length);
uint8_t  Serial_WaitTransmitComplete(UART_HandleTypeDef *huart, uint32_t timeout);

/*===========================================================================
 * 协议层专用 —— My_Uart_Rx.c 的 HAL_UART_RxCpltCallback 调用
 *===========================================================================*/

uint8_t Serial_GetUartIndex(UART_HandleTypeDef *huart);
uint8_t Serial_GetRxByte(uint8_t uartIndex);
/* ISR-safe：可在中断上下文调用。实现必须为非阻塞且直接操作寄存器，
	不得调用会导致 HAL 锁或阻塞的 API（例如 HAL_UART_Receive_IT）。 */
void    Serial_ReArmRxIT(UART_HandleTypeDef *huart, uint8_t uartIndex);

#endif
