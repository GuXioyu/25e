/**
 * @file Cam.h
 * @brief 摄像头坐标模块接口与坐标类型定义。
 */

#ifndef __CAM_H
#define __CAM_H

#include "zf_common_headfile.h"
#include "My_Uart.h"

#define CAM_TARGERT		1		//目标坐标类型
#define CAM_LASER		2		//激光坐标类型
#define CAM_PRELASER	3		//激光坐标类型

#define CAM_X			1		//目标坐标类型
#define CAM_Y			2		//激光坐标类型

void Cam_ReadXY(void);    //读取图像坐标并更新全局坐标
uint8_t Cam_GetFlag(void);    //读取并清除一次新坐标标志
uint16_t Cam_GetXY(uint8_t Axis, uint8_t Type);    //读取指定类型与轴的已保存坐标


#endif /* __CAM_H */
