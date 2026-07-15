#ifndef __BUZZER_H
#define __BUZZER_H

#include "zf_common_headfile.h"

/* 蜂鸣器控制引脚：PA22 */
#define BUZZER_PIN             A22
/* 蜂鸣器开启电平：高电平使能 */
#define BUZZER_ON_LEVEL        1U
/* 蜂鸣器关闭电平：与开启电平相反 */
#define BUZZER_OFF_LEVEL       0U

/* 初始化蜂鸣器引脚 */
void Buzzer_Init(void);
/* 使能蜂鸣器 */
void Buzzer_Enable(void);
/* 失能蜂鸣器 */
void Buzzer_Disable(void);
/* 反转蜂鸣器状态 */
void Buzzer_Toggle(void);

#endif /* __BUZZER_H */
