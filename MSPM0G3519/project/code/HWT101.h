/**
 * @file    HWT101.h
 * @brief   HWT101 姿态传感器数据解析接口
 *
 * @note    串口接收回调中调用 `HWT101_ProcessByte(byte)` 逐字节喂入数据。
 *          解析成功后，主循环可通过 Get 函数读取最近一次数据。
 */

#ifndef __HWT101_H
#define __HWT101_H

#include "zf_common_headfile.h"
#include "My_Uart.h"
#include <stdint.h>

/*===========================================================================
 * 数据结构
 *===========================================================================*/

typedef struct
{
	int16_t  rawGyroZ;  /**< 角速度 Z 原始值 */
	float    gyroZ;     /**< 角速度 Z，单位 deg/s */
	int16_t  rawYaw;    /**< 偏航角 Z 原始值 */
	float    yaw;       /**< 偏航角 Z，单位 deg */
	uint16_t version;   /**< 版本号 */
} HWT101_Data_t;

/** @brief HWT101 最新解析数据 */
extern HWT101_Data_t hwt101Data;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief  逐字节处理 HWT101 数据帧
 * @param  byte  串口收到的单字节
 */
void HWT101_ProcessByte(uint8_t byte);

/**
 * @brief  复位 HWT101 解析状态机
 */
void HWT101_Reset(void);

/**
 * @brief  读取角速度 Z 原始值
 */
int16_t  HWT101_GetRawGyroZ(void);

/**
 * @brief  读取角速度 Z，单位 deg/s
 */
float    HWT101_GetGyroZ(void);

/**
 * @brief  读取偏航角 Z 原始值
 */
int16_t  HWT101_GetRawYaw(void);

/**
 * @brief  读取偏航角 Z，单位 deg
 */
float    HWT101_GetYaw(void);

/**
 * @brief  读取版本号
 */
uint16_t HWT101_GetVersion(void);

/**
 * @brief  设置 Z 轴角度零点并保存
 * @param  huart  HWT101 所连接的 UART 句柄
 * @note   主循环上下文调用，不要在中断中调用。
 */
void HWT101_SetYawZero(UART_HandleTypeDef *huart);

#endif
