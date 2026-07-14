/**
 * @file    My_Uart.c
 * @brief   基于逐飞 UART 驱动的串口兼容层。
 * @note    本模块封装串口初始化、发送、接收缓存和接收中断回调。
 */

#include "My_Uart.h"
#include "My_Uart_Rx.h"

/* 串口句柄配置：串口编号、波特率、发送引脚、接收引脚、接收数据用途。 */
/* huart4,huart5，huart6底层函数写错 */
Serial_Port_t huart0 = {UART_0, 115200, 	UART0_TX_A10, UART0_RX_A11	};
Serial_Port_t huart1 = {UART_1, 115200, 	UART1_TX_A8,  UART1_RX_A9	};
//Serial_Port_t huart2 = {UART_2, 115200, 	UART2_TX_B15, UART2_RX_B16};
Serial_Port_t huart3 = {UART_3, 115200, 	UART3_TX_A14, UART3_RX_A13	};
Serial_Port_t huart4 = {UART_4, (115200/2), UART4_TX_B10, UART4_RX_B11	};
Serial_Port_t huart5 = {UART_5, (115200/2), UART5_TX_A1,  UART5_RX_A0	};
Serial_Port_t huart6 = {UART_6, (115200/2), UART6_TX_B22, UART6_RX_B21	};
Serial_Port_t huart7 = {UART_7, 115200, 	UART7_TX_B2,  UART7_RX_B3	};

/* 各串口最新接收到的 1 字节原始数据。 */
volatile uint8_t Serial_RxData[SERIAL_RX_UART_COUNT];
/* 各串口完整数据包接收完成标志。 */
volatile uint8_t Serial_RxFlag[SERIAL_RX_UART_COUNT];
/* 字符串数据包缓存，由 My_Uart_Rx.c 按 '[' 和 ']' 解析填充。 */
volatile char Serial_RxPacket[SERIAL_RX_UART_COUNT][100];

/* 中断回调中临时保存的最新接收字节，供接收解析函数读取。 */
static uint8_t Serial_RxByte[SERIAL_RX_UART_COUNT];

/* 串口句柄列表，用于把句柄指针转换为数组下标。 */
static Serial_Port_t *const Serial_UartList[SERIAL_RX_UART_COUNT] =
{
	&huart0,
	&huart1,
	//&huart2,
	&huart3,
	&huart4,
	&huart5,
	&huart6,
	&huart7,
};

/**
 * @brief  逐飞 UART 接收中断回调函数。
 * @param  event  串口中断事件。
 * @param  ptr    初始化时传入的用户参数，这里为串口句柄。
 */
static void Serial_ZfUartCallback(uint32 event, void *ptr)
{
    Serial_Port_t *huart = (Serial_Port_t *)ptr;
    uint8_t uartIndex;
    uint8_t byte;

    if ((huart == NULL) || (event != UART_INTERRUPT_STATE_RX))
    {
        return;
    }

    uartIndex = Serial_GetUartIndex(huart);
    if (uartIndex >= SERIAL_RX_UART_COUNT)
    {
        return;
    }

    while (uart_query_byte(huart->uart, &byte))
    {
        Serial_RxByte[uartIndex] = byte;
        Serial_RxDispatch(huart);
    }
}

/**
 * @brief  获取串口句柄对应的数组下标。
 * @param  huart  串口句柄指针。
 * @return 串口数组下标；非法句柄返回 SERIAL_RX_UART_COUNT。
 */
uint8_t Serial_GetUartIndex(Serial_Port_t *huart)
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
 * @brief  获取指定串口最新接收字节。
 * @param  uartIndex  串口数组下标。
 * @return 最新接收字节；下标非法时返回 0。
 */
uint8_t Serial_GetRxByte(uint8_t uartIndex)
{
    if (uartIndex >= SERIAL_RX_UART_COUNT)
    {
        return 0;
    }

    return Serial_RxByte[uartIndex];
}

/**
 * @brief  重新开启串口接收中断。
 * @param  huart      串口句柄指针。
 * @param  uartIndex  兼容保留参数，当前未使用。
 */
void Serial_ReArmRxIT(Serial_Port_t *huart, uint8_t uartIndex)
{
    (void)uartIndex;

    if (huart != NULL)
    {
        uart_set_interrupt_config(huart->uart, UART_INTERRUPT_CONFIG_RX_ENABLE);
    }
}

/**
 * @brief  初始化串口并开启接收中断。
 * @param  huart  串口句柄指针。
 */
void Serial_Init(Serial_Port_t *huart)
{
    uint8_t uartIndex = Serial_GetUartIndex(huart);
    if (uartIndex >= SERIAL_RX_UART_COUNT)
    {
        return;
    }

    Serial_RxData[uartIndex] = 0;
    Serial_RxFlag[uartIndex] = 0;
    Serial_RxByte[uartIndex] = 0;
    Serial_RxPacket[uartIndex][0] = '\0';

    uart_init(huart->uart, huart->baudrate, huart->tx_pin, huart->rx_pin);
    uart_set_callback(huart->uart, Serial_ZfUartCallback, huart);
    uart_set_interrupt_config(huart->uart, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

/**
 * @brief  获取并清除接收完成标志。
 * @param  huart  串口句柄指针。
 * @return 1 表示收到完整数据包；0 表示未收到。
 */
uint8_t Serial_GetRxFlag(Serial_Port_t *huart)
{
    uint8_t uartIndex = Serial_GetUartIndex(huart);
    if (uartIndex >= SERIAL_RX_UART_COUNT)
    {
        return 0;
    }

    if (Serial_RxFlag[uartIndex] != 0)
    {
        Serial_RxFlag[uartIndex] = 0;
        return 1;
    }

    return 0;
}

/**
 * @brief  获取串口接收数据包缓存。
 * @param  huart  串口句柄指针。
 * @return 数据包缓存指针；非法句柄返回 NULL。
 */
volatile char *Serial_GetRxPacket(Serial_Port_t *huart)
{
    uint8_t uartIndex = Serial_GetUartIndex(huart);
    if (uartIndex >= SERIAL_RX_UART_COUNT)
    {
        return NULL;
    }

    return Serial_RxPacket[uartIndex];
}

/**
 * @brief  串口格式化发送。
 * @param  huart   串口句柄指针。
 * @param  format  printf 格式字符串。
 */
void Serial_Printf(Serial_Port_t *huart, const char *format, ...)
{
    char string[128];
    va_list arg;

    if ((huart == NULL) || (format == NULL))
    {
        return;
    }

    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);
    va_end(arg);

    Serial_SendData(huart, (const uint8_t *)string, (uint16_t)strlen(string));
}

/**
 * @brief  串口发送指定长度数据。
 * @param  huart   串口句柄指针。
 * @param  buffer  待发送数据缓存。
 * @param  length  待发送数据长度。
 * @return 实际提交发送的数据长度；参数非法时返回 0。
 */
uint16_t Serial_SendData(Serial_Port_t *huart, const uint8_t *buffer, uint16_t length)
{
    if ((huart == NULL) || (buffer == NULL) || (length == 0))
    {
        return 0;
    }

    if (Serial_GetUartIndex(huart) >= SERIAL_RX_UART_COUNT)
    {
        return 0;
    }

    uart_write_buffer(huart->uart, buffer, length);
    return length;
}

/**
 * @brief  等待串口发送完成。
 * @param  huart    串口句柄指针。
 * @param  timeout  超时时间，当前保留未使用。
 * @return 当前固定返回 1，作为兼容接口使用。
 */
uint8_t Serial_WaitTransmitComplete(Serial_Port_t *huart, uint32_t timeout)
{
    (void)huart;
    (void)timeout;

    return 1;
}
