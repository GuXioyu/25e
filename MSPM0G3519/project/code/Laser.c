#include "Laser.h"

/* 激光当前状态缓存：0=关闭，1=开启 */
static uint8_t s_laser_state = LASER_OFF_LEVEL;

/* 设置激光输出电平 */
static void Laser_WriteLevel(uint8_t level)
{
	gpio_set_level(LASER_PIN, level);
}

/* 初始化激光 */
void Laser_Init(void)
{
	gpio_init(LASER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
	s_laser_state = LASER_OFF_LEVEL;
	Laser_WriteLevel(LASER_OFF_LEVEL);
}

/* 使能激光 */
void Laser_Enable(void)
{
	s_laser_state = LASER_ON_LEVEL;
	Laser_WriteLevel(LASER_ON_LEVEL);
}

/* 失能激光 */
void Laser_Disable(void)
{
	s_laser_state = LASER_OFF_LEVEL;
	Laser_WriteLevel(LASER_OFF_LEVEL);
}

/* 反转激光状态 */
void Laser_Toggle(void)
{
	if (s_laser_state == LASER_ON_LEVEL)
	{
		Laser_Disable();
	}
	else
	{
		Laser_Enable();
	}
}
