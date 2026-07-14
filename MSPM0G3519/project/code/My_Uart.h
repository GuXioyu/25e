/**
 * @file    My_Uart.h
 * @brief   基于逐飞 UART 驱动的串口兼容层接口。
 */

#ifndef __MY_UART_H
#define __MY_UART_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zf_driver_uart.h"

/* 默认串口数量与逐飞 UART 驱动中的 UART_NUM 保持一致。 */
#ifndef SERIAL_RX_UART_COUNT
#define SERIAL_RX_UART_COUNT UART_NUM
#endif

/**
 * @brief 串口句柄结构体，用于兼容移植过来的 HAL 风格代码。
 */
typedef struct
{
    /* 逐飞 UART 编号 */
    uart_index_enum uart;
    /* 串口波特率 */
    uint32_t baudrate;
    /* 串口发送引脚 */
    uart_tx_pin_enum tx_pin;
    /* 串口接收引脚 */
    uart_rx_pin_enum rx_pin;
} Serial_Port_t;

/* 兼容 HAL 风格命名，便于移植原来的 UART_HandleTypeDef 代码。 */
typedef Serial_Port_t UART_HandleTypeDef;

/* 外部可直接使用的串口句柄。 */
extern Serial_Port_t huart0;
extern Serial_Port_t huart1;
//extern Serial_Port_t huart2;
extern Serial_Port_t huart3;
extern Serial_Port_t huart4;
extern Serial_Port_t huart5;
extern Serial_Port_t huart6;
extern Serial_Port_t huart7;

/* 各串口最新接收到的 1 字节原始数据。 */
extern volatile uint8_t Serial_RxData[SERIAL_RX_UART_COUNT];
/* 各串口完整数据包接收完成标志。 */
extern volatile uint8_t Serial_RxFlag[SERIAL_RX_UART_COUNT];
/* 各串口接收数据包缓存。 */
extern volatile char Serial_RxPacket[SERIAL_RX_UART_COUNT][100];

/**
 * @brief  初始化串口并开启接收中断。
 * @param  huart  串口句柄指针。
 */
void Serial_Init(Serial_Port_t *huart);

/**
 * @brief  获取并清除接收完成标志。
 * @param  huart  串口句柄指针。
 * @return 1 表示收到完整数据包；0 表示未收到。
 */
uint8_t Serial_GetRxFlag(Serial_Port_t *huart);

/**
 * @brief  获取串口接收数据包缓存。
 * @param  huart  串口句柄指针。
 * @return 数据包缓存指针；非法句柄返回 NULL。
 */
volatile char *Serial_GetRxPacket(Serial_Port_t *huart);

/**
 * @brief  串口格式化发送。
 * @param  huart   串口句柄指针。
 * @param  format  printf 格式字符串。
 * @param  ...     可变参数。
 */
void Serial_Printf(Serial_Port_t *huart, const char *format, ...);

/**
 * @brief  串口发送指定长度数据。
 * @param  huart   串口句柄指针。
 * @param  buffer  待发送数据缓存。
 * @param  length  待发送数据长度。
 * @return 实际提交发送的数据长度；参数非法时返回 0。
 */
uint16_t Serial_SendData(Serial_Port_t *huart, const uint8_t *buffer, uint16_t length);

/**
 * @brief  等待串口发送完成。
 * @param  huart    串口句柄指针。
 * @param  timeout  超时时间，当前保留未使用。
 * @return 当前固定返回 1，作为兼容接口使用。
 */
uint8_t Serial_WaitTransmitComplete(Serial_Port_t *huart, uint32_t timeout);

/**
 * @brief  获取串口句柄对应的数组下标。
 * @param  huart  串口句柄指针。
 * @return 串口数组下标；非法句柄返回 SERIAL_RX_UART_COUNT。
 */
uint8_t Serial_GetUartIndex(Serial_Port_t *huart);

/**
 * @brief  获取指定串口最新接收字节。
 * @param  uartIndex  串口数组下标。
 * @return 最新接收字节；下标非法时返回 0。
 */
uint8_t Serial_GetRxByte(uint8_t uartIndex);

/**
 * @brief  重新开启串口接收中断。
 * @param  huart      串口句柄指针。
 * @param  uartIndex  兼容保留参数，当前未使用。
 */
void Serial_ReArmRxIT(Serial_Port_t *huart, uint8_t uartIndex);

#endif
