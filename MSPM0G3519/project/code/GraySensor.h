/**
 * @file    GraySensor.h
 * @brief   灰度循迹传感器 —— 8路模式收发处理
 *
 * @note    用法：
 *          1. 调用 `GraySensor_SendQuery(&huartX, type)` 发送查询帧（固定 0x57 0x01）。
 *          2. 在串口接收回调中调用 `GraySensor_ProcessByte(byte)`，或将其注册为
 *             `Uart_RxUserCallback` 以逐字节解析回包。
 *          3. 主循环可直接调用 `GraySensor_GetAnalog(ch)` 读取最近一次解析成功的模拟量。
 *
 * @note   线程/中断注意事项：
 *         - `GraySensor_ProcessByte` 设计为 ISR-safe（仅做字节状态机和内存写入，非阻塞），
 *           因此可以在中断上下文调用。
 *         - 主循环读取模拟量时使用 `GraySensor_GetAnalog(ch)`，函数内部会进入临界区。
 */

#ifndef __GRAYSENSOR_H
#define __GRAYSENSOR_H

#include <stdint.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "My_Uart.h"

/* 灰度传感器查询类型：数字量回包为 1 字节，模拟量回包为 16 字节。 */
#define GRAYSENSOR_QUERY_DIGITAL 1U
#define GRAYSENSOR_QUERY_ANALOG  2U

/*===========================================================================
 * 8 路灰度数据结构体
 *===========================================================================*/

typedef struct
{
	uint16_t analog[8];      /**< Data0-15: 8路模拟值           */
	uint8_t digital;         /**< 8路数字量位图，bit0 对应第 0 路 */
} GraySensor_Data_t;

/*===========================================================================
 * 全局数据实例
 *
 * 说明：`gsData` 存放最近解析成功的一帧数据。ISR 在解析完成时会把
 * `gsData` 写入；主循环通过 `GraySensor_GetAnalog(ch)` 读取最近一次模拟量。
 *===========================================================================*/

/**
 * @brief  发送查询帧到灰度传感器（ID 固定 0x01）
 * @param  huart  HAL 串口句柄
 * @param  type   1=数字量，2=模拟量；无效值不发送。
 * @note   主机发送 2 字节：0x57 0x01
 */
void GraySensor_SendQuery(UART_HandleTypeDef *huart, uint8_t type);

/**
 * @brief  逐字节处理灰度传感器回包（可注册为 Uart_RxUserCallback）
 * @param  byte       接收到的单字节
 * @note   内部状态机按最近一次查询自动解析数字量回包（0x75 + Data0 + 0x02）
 *         或模拟量回包（0x75 + 16 数据 + 0x11 帧尾）。数字量帧更新数字量，
 *         模拟量帧只更新模拟量，不更新数字量。
 * @warning ISR-safe: 此函数设计可在中断上下文调用，必须保持非阻塞、
 *          只做轻量状态机与内存写入，严禁调用阻塞或会引发 HAL 锁的 API。
 */
void GraySensor_ProcessByte(uint8_t byte);

/**
 * @brief  复位灰度传感器状态机（用于异常恢复）
 */
void GraySensor_Reset(void);

/**
 * @brief 从 gsData 原子读指定通道的模拟量
 * @param ch 通道索引 (0..7)
 * @return 该通道模拟值，ch 越界则返回 0
 */
uint16_t GraySensor_GetAnalog(uint8_t ch);

/**
 * @brief  读取最近一次校验成功的原生数字量帧中指定通道的数字量。
 * @param  ch  通道索引 (0..7)
 * @return 1=检测到黑线，0=白底或通道越界
 */
uint8_t GraySensor_GetDigital(uint8_t ch);

/**
 * @brief  读取最近一次校验成功的原生数字量帧。
 * @return bit0~bit7 对应通道 0~7，位为 1 表示该路检测到黑线。
 */
uint8_t GraySensor_GetDigitalByte(void);

#endif
