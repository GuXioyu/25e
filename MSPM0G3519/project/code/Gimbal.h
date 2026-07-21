#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* Gimbal_GetSpeed() 的电机选择参数 */
#define GIMBAL_SPEED_YAW    (1U)
#define GIMBAL_SPEED_PITCH 	(2U)
#define GIMBAL_ANGLE_YAW    (3U)
#define GIMBAL_ANGLE_PITCH 	(4U)

/* ==================== Timer ==================== */
void Gimbal_Timer(void);//1 ms 定时器节拍函数，每 10 ms 请求一次云台处理
/* ==================== Parent ==================== */
uint8_t Gimbal_Aim1(uint8_t mode);//执行定点瞄准状态机
void Gimbal_Aim2(void);//执行运动过程中的 Yaw 瞄准控制
void Gimbal_Aim3(void);//执行预留的动态瞄准控制
/* ==================== Child ==================== */
void Gimbal_ResetAim1(void); //复位定点瞄准状态机
static uint16_t Gimbal_GetAbsError(uint16_t Target, uint16_t Actual); // 计算图像坐标绝对误差
static float Gimbal_GetStepAngle(uint16_t Target, uint16_t Actual, uint8_t FineAdjustEnable); // 生成图像误差对应的相对转角
static void Gimbal_UpdateAimOk(uint16_t Error, uint16_t ErrorOk, uint8_t *Count, uint8_t *Ok); // 更新单轴瞄准稳定状态
/* ==================== 通信 ==================== */
//读取并清除一次云台结果标志
uint8_t Gimbal_GetFlag(void);//获取一次尚未读取的云台结果标志
float Gimbal_GetSpeed(uint8_t motor);//获取指定轴的云台目标速度
//更新当前帧的目标与激光图像坐标
void Gimbal_GetImage(uint16_t target_x, uint16_t target_y, uint16_t laser_x, uint16_t laser_y);
void Tx(void);
//执行 Yaw 角速度补偿测试
void Gimbal_tast(void);

#endif /* GIMBAL_H */
