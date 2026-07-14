#include <stdint.h>
#include <stdio.h>

#include "../Hardware/GraySensor.h"
#include "../My_Flash.h"

uint16_t Serial_SendData(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t length)
{
	(void)huart;
	(void)buffer;
	return length;
}

static void feed_gray_frame(const uint16_t analog[8])
{
	GraySensor_ProcessByte(0x75);
	for (uint8_t ch = 0; ch < 8; ch++)
	{
		GraySensor_ProcessByte((uint8_t)(analog[ch] >> 8));
		GraySensor_ProcessByte((uint8_t)analog[ch]);
	}
	GraySensor_ProcessByte(0x11);
}

int main(void)
{
	const uint16_t analog[8] = {699, 700, 701, 800, 801, 0, 65535, 42};
	const uint16_t middle[8] = {750, 750, 750, 750, 750, 750, 750, 750};
	const uint16_t white[8] = {801, 801, 801, 801, 801, 801, 801, 801};
	const uint16_t black[8] = {699, 699, 699, 699, 699, 699, 699, 699};

	My_Flash_Load();
	if (GraySensor_GetThreshold() != 700)
	{
		printf("default threshold failed: %u\n", GraySensor_GetThreshold());
		return 1;
	}

	GraySensor_SetThreshold(700);
	GraySensor_Reset();
	feed_gray_frame(analog);

	if (GraySensor_GetDigitalByte() != 0xA1)
	{
		printf("digital byte failed: 0x%02X\n", GraySensor_GetDigitalByte());
		return 1;
	}

	if (GraySensor_GetDigital(0) != 1 ||
		GraySensor_GetDigital(1) != 0 ||
		GraySensor_GetDigital(2) != 0 ||
		GraySensor_GetDigital(4) != 0 ||
		GraySensor_GetDigital(5) != 1 ||
		GraySensor_GetDigital(7) != 1 ||
		GraySensor_GetDigital(8) != 0)
	{
		printf("digital channel failed\n");
		return 1;
	}

	feed_gray_frame(middle);
	if (GraySensor_GetDigitalByte() != 0xA1)
	{
		printf("digital hysteresis hold failed: 0x%02X\n", GraySensor_GetDigitalByte());
		return 1;
	}

	feed_gray_frame(white);
	if (GraySensor_GetDigitalByte() != 0x00)
	{
		printf("digital white threshold failed: 0x%02X\n", GraySensor_GetDigitalByte());
		return 1;
	}

	feed_gray_frame(black);
	if (GraySensor_GetDigitalByte() != 0xFF)
	{
		printf("digital black threshold failed: 0x%02X\n", GraySensor_GetDigitalByte());
		return 1;
	}

	if (GraySensor_SetAndSaveThreshold(1234) == 0)
	{
		printf("threshold set and save failed\n");
		return 1;
	}

	GraySensor_SetThreshold(42);
	GraySensor_LoadThreshold();
	if (GraySensor_GetThreshold() != 1234)
	{
		printf("threshold load failed: %u\n", GraySensor_GetThreshold());
		return 1;
	}

	printf("ok\n");
	return 0;
}
