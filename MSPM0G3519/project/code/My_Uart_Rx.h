/**
 * @file    My_Uart_Rx.h
 * @brief   串口接收数据包解析接口。
 */

#ifndef __MY_UART_RX_H
#define __MY_UART_RX_H

#include "My_Uart.h"

/**
 * @brief  串口接收字节分发解析。
 * @param  huart  串口句柄指针。
 */
void Serial_RxDispatch(Serial_Port_t *huart);

#endif
