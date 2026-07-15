#include "Buzzer.h"

/* 蜂鸣器当前状态缓存：0=关闭，1=开启 */
static uint8_t s_buzzer_state = BUZZER_OFF_LEVEL;

/* 设置蜂鸣器输出电平 */
static void Buzzer_WriteLevel(uint8_t level)
{
	gpio_set_level(BUZZER_PIN, level);
}

/* 初始化蜂鸣器 */
void Buzzer_Init(void)
{
	gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
	s_buzzer_state = BUZZER_OFF_LEVEL;
	Buzzer_WriteLevel(BUZZER_OFF_LEVEL);
}

/* 使能蜂鸣器 */
void Buzzer_Enable(void)
{
	s_buzzer_state = BUZZER_ON_LEVEL;
	Buzzer_WriteLevel(BUZZER_ON_LEVEL);
}

/* 失能蜂鸣器 */
void Buzzer_Disable(void)
{
	s_buzzer_state = BUZZER_OFF_LEVEL;
	Buzzer_WriteLevel(BUZZER_OFF_LEVEL);
}

/* 反转蜂鸣器状态 */
void Buzzer_Toggle(void)
{
	if (s_buzzer_state == BUZZER_ON_LEVEL)
	{
		Buzzer_Disable();
	}
	else
	{
		Buzzer_Enable();
	}
}
