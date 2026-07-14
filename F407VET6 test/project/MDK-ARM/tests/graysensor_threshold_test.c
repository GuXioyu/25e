#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../Hardware/GraySensor.h"

static uint8_t last_tx[2];      /* 保存最近一次查询帧，供测试校验。 */
static uint16_t last_tx_length; /* 保存最近一次查询帧的实际长度。 */

uint16_t Serial_SendData(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t length)
{
	uint16_t copy_length = (length > sizeof(last_tx)) ? sizeof(last_tx) : length;

	(void)huart;
	memcpy(last_tx, buffer, copy_length); /* 捕获查询帧，不访问真实串口。 */
	last_tx_length = length;
	return length;
}

/* 按数字量协议输入帧头、一个数据字节和帧尾。 */
static void feed_digital_frame(uint8_t header, uint8_t data, uint8_t tail)
{
	GraySensor_ProcessByte(header);
	GraySensor_ProcessByte(data);
	GraySensor_ProcessByte(tail);
}

/* 按模拟量协议输入帧头、16 个数据字节和帧尾。 */
static void feed_analog_frame(uint8_t header, const uint16_t analog[8], uint8_t tail)
{
	uint8_t ch;

	GraySensor_ProcessByte(header);
	for (ch = 0; ch < 8; ch++)
	{
		GraySensor_ProcessByte((uint8_t)(analog[ch] >> 8));
		GraySensor_ProcessByte((uint8_t)analog[ch]);
	}
	GraySensor_ProcessByte(tail);
}

int main(void)
{
	const uint16_t analog[8] = {101, 102, 103, 104, 105, 106, 107, 108};

	GraySensor_SendQuery(NULL, GRAYSENSOR_QUERY_DIGITAL);
	if (last_tx_length != 2 || last_tx[0] != 0x57 || last_tx[1] != 0x01)
	{
		printf("digital query frame failed\n");
		return 1;
	}

	/* Data0 的 bit0~bit7 必须保持对应第 0~7 路数字量。 */
	feed_digital_frame(0x75, 0xA5, 0x02);
	if (GraySensor_GetDigitalByte() != 0xA5 ||
		GraySensor_GetDigital(0) != 1 ||
		GraySensor_GetDigital(1) != 0 ||
		GraySensor_GetDigital(2) != 1 ||
		GraySensor_GetDigital(7) != 1)
	{
		printf("digital frame bit order failed\n");
		return 1;
	}

	/* 错误帧尾必须丢弃数字量数据，保留上一帧的有效结果。 */
	feed_digital_frame(0x75, 0x3C, 0x03);
	if (GraySensor_GetDigitalByte() != 0xA5)
	{
		printf("invalid digital tail updated data\n");
		return 1;
	}

	GraySensor_SendQuery(NULL, GRAYSENSOR_QUERY_ANALOG);
	if (last_tx_length != 2 || last_tx[0] != 0x57 || last_tx[1] != 0x01)
	{
		printf("analog query frame failed\n");
		return 1;
	}

	feed_analog_frame(0x75, analog, 0x11);
	if (GraySensor_GetAnalog(0) != 101 || GraySensor_GetAnalog(7) != 108)
	{
		printf("analog frame parse failed\n");
		return 1;
	}

	/* 模拟量帧只能更新模拟量，不能覆盖原生数字量数据。 */
	if (GraySensor_GetDigitalByte() != 0xA5)
	{
		printf("analog frame changed native digital data\n");
		return 1;
	}

	printf("ok\n");
	return 0;
}
