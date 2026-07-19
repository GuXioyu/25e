/**
 * @file Cam.c
 * @brief 摄像头目标坐标帧接收与读取模块。
 */

#include "Cam.h"

#define CAM_HUART	huart4		//摄像头坐标帧使用的串口句柄

uint16_t cam_target_x;			//目标 X 坐标，单位：像素
uint16_t cam_target_y;			//目标 Y 坐标，单位：像素
uint16_t cam_laser_x;			//激光 X 坐标预留值，单位：像素
uint16_t cam_laser_y;			//激光 Y 坐标预留值，单位：像素

uint8_t cam_flag;				//新数据标志位

/**
 * @brief  从 UART4 读取并解析 `cam,x,y` 格式的坐标帧
 * @param  无
 * @return 无
 * @note   数据格式为[cam,x,y]，例如[cam,320,180]
 */
void Cam_ReadXY(void)    //读取图像坐标并更新全局坐标
{
	//仅在串口模块通知收到完整帧时开始解析
	if (Serial_GetRxFlag(&CAM_HUART) == 0)return; //检测 UART4 是否收到完整坐标帧
	
    char *Tag = strtok((char *)Serial_GetRxPacket(&CAM_HUART), ",");    //解析数据帧标签
	//仅处理标签为 cam 的视觉坐标帧
    if (Tag != NULL && strcmp(Tag, "cam") == 0)    		//确认当前帧为视觉坐标帧
    {
        char *Value1 = strtok(NULL, ", ");   			//解析 X 坐标字符串
        char *Value2 = strtok(NULL, ", ");    			//解析 Y 坐标字符串
		//字段完整时分别更新目标坐标
        if (Value1 != NULL && Value2 != NULL)    		//确认坐标字段完整
        {
            int IntValue1 = atoi(Value1);  				//转换 X 坐标数值
            cam_target_x = IntValue1;          			//更新全局 X 坐标

            int IntValue2 = atoi(Value2);    			//转换 Y 坐标数值
            cam_target_y = IntValue2;            		//更新全局 Y 坐标
			
			cam_flag = 1;								//置新数据标志位
        }
    }
}
	
/**
 * @brief 读取并清除一次新坐标标志
 * @param 无
 * @return 1 表示存在新坐标；0 表示没有新坐标
 */
uint8_t Cam_GetFlag(void)    //读取并清除一次新坐标标志
{
	//没有新坐标时直接通知调用方读取失败
	if (cam_flag == 0)return 0;
	
	//成功读取后消费本次新坐标通知
	cam_flag = 0;
	return 1;
}

/**
 * @brief 读取指定类型与轴的已保存坐标
 * @param Axis 坐标轴，1 表示 X，2 表示 Y
 * @param Type 坐标类型，取 CAM_TARGERT 或 CAM_LASER
 * @return 对应坐标；轴或类型无效时返回 0
 */
uint16_t Cam_GetXY(uint8_t Axis, uint8_t Type)    //读取指定类型与轴的已保存坐标
{
	//根据坐标类型返回指定轴的目标或激光坐标
	if (Type == CAM_TARGERT)
	{
		if (Axis == 1)return cam_target_x;
		if (Axis == 2)return cam_target_y;
	}
	
	if (Type == CAM_LASER)
	{
		if (Axis == 1)return cam_laser_x;
		if (Axis == 2)return cam_laser_y;
	}
	
	//拒绝无效轴或未定义坐标类型
	return 0;
}
