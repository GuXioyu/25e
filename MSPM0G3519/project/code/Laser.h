#ifndef __LASER_H
#define __LASER_H

#include "zf_common_headfile.h"

/* 激光控制引脚：A21 */
#define LASER_PIN          A21
/* 激光开启电平：高电平使能 */
#define LASER_ON_LEVEL     1U
/* 激光关闭电平：与开启电平相反 */
#define LASER_OFF_LEVEL    0U

/* 初始化激光引脚 */
void Laser_Init(void);
/* 使能激光 */
void Laser_Enable(void);
/* 失能激光 */
void Laser_Disable(void);
/* 反转激光状态 */
void Laser_Toggle(void);

#endif /* __LASER_H */