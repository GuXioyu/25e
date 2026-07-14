#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../Hardware/HWT101.h"

struct __UART_HandleTypeDef
{
	int dummy;
};

static uint8_t txLog[3][5];
static uint16_t txLenLog[3];
static uint8_t txCount;
static uint32_t delayLog[2];
static uint8_t delayCount;

uint16_t Serial_SendData(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t length)
{
	(void)huart;

	if (txCount < 3)
	{
		txLenLog[txCount] = length;
		for (uint8_t i = 0; i < length && i < 5; i++)
		{
			txLog[txCount][i] = buffer[i];
		}
	}

	txCount++;
	return length;
}

void HAL_Delay(uint32_t delay)
{
	if (delayCount < 2)
	{
		delayLog[delayCount] = delay;
	}
	delayCount++;
}

static int nearly_equal(float a, float b)
{
	float diff = a - b;
	if (diff < 0.0f)
	{
		diff = -diff;
	}
	return diff < 0.001f;
}

static uint8_t sum11(const uint8_t frame[11])
{
	uint16_t sum = 0;
	for (uint8_t i = 0; i < 10; i++)
	{
		sum += frame[i];
	}
	return (uint8_t)sum;
}

int main(void)
{
	uint8_t gyro[11] = {0x55, 0x52, 0x00, 0x00, 0x34, 0x12, 0x00, 0x10, 0x00, 0x00, 0x00};
	uint8_t angle[11] = {0x55, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x01, 0x00};
	UART_HandleTypeDef hwtUart = {0};
	const uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
	const uint8_t zero[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
	const uint8_t save[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

	gyro[10] = sum11(gyro);
	angle[10] = sum11(angle);

	HWT101_Reset();
	for (uint8_t i = 0; i < 11; i++)
	{
		HWT101_ProcessByte(gyro[i]);
	}

	if (HWT101_GetRawGyroZ() != 0x1000)
	{
		printf("raw gyro z failed: %d\n", HWT101_GetRawGyroZ());
		return 1;
	}
	if (!nearly_equal(HWT101_GetGyroZ(), 250.0f))
	{
		printf("gyro z failed: %.3f\n", HWT101_GetGyroZ());
		return 1;
	}

	for (uint8_t i = 0; i < 11; i++)
	{
		HWT101_ProcessByte(angle[i]);
	}

	if (HWT101_GetRawYaw() != 0x4000)
	{
		printf("raw yaw failed: %d\n", HWT101_GetRawYaw());
		return 1;
	}
	if (!nearly_equal(HWT101_GetYaw(), 90.0f))
	{
		printf("yaw failed: %.3f\n", HWT101_GetYaw());
		return 1;
	}
	if (HWT101_GetVersion() != 0x0102)
	{
		printf("version failed: %u\n", HWT101_GetVersion());
		return 1;
	}

	HWT101_SetYawZero(&hwtUart);

	if (txCount != 3)
	{
		printf("set yaw zero tx count failed: %u\n", txCount);
		return 1;
	}
	if (delayCount != 2 || delayLog[0] != 200U || delayLog[1] != 500U)
	{
		printf("set yaw zero delay failed: %u %lu %lu\n",
			   delayCount,
			   (unsigned long)delayLog[0],
			   (unsigned long)delayLog[1]);
		return 1;
	}
	for (uint8_t row = 0; row < 3; row++)
	{
		const uint8_t *expected = (row == 0) ? unlock : ((row == 1) ? zero : save);
		if (txLenLog[row] != 5)
		{
			printf("set yaw zero tx length failed: %u\n", txLenLog[row]);
			return 1;
		}
		for (uint8_t col = 0; col < 5; col++)
		{
			if (txLog[row][col] != expected[col])
			{
				printf("set yaw zero command failed: row=%u col=%u value=0x%02X\n",
					   row,
					   col,
					   txLog[row][col]);
				return 1;
			}
		}
	}

	printf("ok\n");
	return 0;
}
