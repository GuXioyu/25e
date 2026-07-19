#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* Gimbal_GetSpeed() 的电机选择参数 */
#define GIMBAL_MOTOR_YAW    (1U)
#define GIMBAL_MOTOR_PITCH (2U)

/* ==================== Init ==================== */
void Gimbal_Init(void);//初始化云台 PID 控制器
/* ==================== Timer ==================== */
void Gimbal_Timer(void);//1 ms 定时器节拍函数，每 10 ms 请求一次云台处理
/* ==================== Parent ==================== */
uint8_t Gimbal_Aim1(uint8_t mode);
void Gimbal_Aim2(void);
void Gimbal_Aim3(void);
/* ==================== Child ==================== */


/* ==================== 通信 ==================== */
uint8_t Gimbal_GetFlag(void);//获取一次尚未读取的云台结果标志
float Gimbal_GetSpeed(uint8_t motor);//获取指定轴的云台目标速度



#endif /* GIMBAL_H */