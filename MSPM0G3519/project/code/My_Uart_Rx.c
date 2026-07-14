/**
 * @file    My_Uart_Rx.c
 * @brief   串口接收数据包解析模块。
 * @note    按字节解析串口接收数据，以 '[' 作为起始符，以 ']' 作为结束符。
 */

#include "My_Uart_Rx.h"
#include "GraySensor.h"
#include "HWT101.h"
#include "Motor.h"

/**
 * @brief  串口接收字节分发解析。
 * @param  huart  串口句柄指针。
 */
void Serial_RxDispatch(Serial_Port_t *huart)
{
    /* rxState: 0 表示等待包头；1 表示正在接收包内容。 */
    static uint8_t rxState[SERIAL_RX_UART_COUNT];
    /* rxPktIdx: 当前串口接收到 Serial_RxPacket 的字符下标。 */
    static uint8_t rxPktIdx[SERIAL_RX_UART_COUNT];
    uint8_t uartIndex;
    uint8_t byte;

    uartIndex = Serial_GetUartIndex(huart);
    if (uartIndex >= SERIAL_RX_UART_COUNT)
    {
        return;
    }

    byte = Serial_GetRxByte(uartIndex);
    Serial_RxData[uartIndex] = byte;

    /* ========== 外部解析 ========== */
    

    if (rxState[uartIndex] == 0)
    {
        if (byte == '[')
        {
            rxState[uartIndex] = 1;
            rxPktIdx[uartIndex] = 0;
            Serial_RxPacket[uartIndex][0] = '\0';
        }
    }
    else
    {
        if (byte == ']')
        {
            Serial_RxPacket[uartIndex][rxPktIdx[uartIndex]] = '\0';
            Serial_RxFlag[uartIndex] = 1;
            rxState[uartIndex] = 0;
        }
        else if (byte == '[')
        {
            rxPktIdx[uartIndex] = 0;
            Serial_RxPacket[uartIndex][0] = '\0';
        }
        else
        {
            if (rxPktIdx[uartIndex] < (uint8_t)(sizeof(Serial_RxPacket[uartIndex]) - 1U))
            {
                Serial_RxPacket[uartIndex][rxPktIdx[uartIndex]] = (char)byte;
                rxPktIdx[uartIndex]++;
                Serial_RxPacket[uartIndex][rxPktIdx[uartIndex]] = '\0';
            }
            else
            {
                rxState[uartIndex] = 0;
                Serial_RxPacket[uartIndex][0] = '\0';
            }
        }
    }
}
