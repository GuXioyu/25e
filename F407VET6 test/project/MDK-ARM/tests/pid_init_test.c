#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../User/PID.h"

static int nearly_equal(float a, float b)
{
	float diff = a - b;
	if (diff < 0.0f)
	{
		diff = -diff;
	}
	return diff < 0.001f;
}

int main(void)
{
	PID_t pid;
	memset(&pid, 0xA5, sizeof(pid));

	PID_Init(&pid, 1.2f, 0.3f, 0.04f, -100.0f, 100.0f, -20.0f, 20.0f);

	if (!nearly_equal(pid.kp, 1.2f) ||
		!nearly_equal(pid.ki, 0.3f) ||
		!nearly_equal(pid.kd, 0.04f))
	{
		printf("pid gains init failed\n");
		return 1;
	}

	if (!nearly_equal(pid.outmin, -100.0f) ||
		!nearly_equal(pid.outmax, 100.0f))
	{
		printf("pid output limit init failed\n");
		return 1;
	}

	if (pid.intmode != (uint8_t)(PID_INTMODE_DATA_NORMAL | PID_INTMODE_LIMIT))
	{
		printf("pid intmode init failed: %u\n", pid.intmode);
		return 1;
	}

	if (pid.difmode != PID_DIFMODE_NORMAL)
	{
		printf("pid difmode init failed: %u\n", pid.difmode);
		return 1;
	}

	if (!nearly_equal(pid.target, 0.0f) ||
		!nearly_equal(pid.actual_current, 0.0f) ||
		!nearly_equal(pid.actual_past, 0.0f) ||
		!nearly_equal(pid.out, 0.0f) ||
		!nearly_equal(pid.error_current, 0.0f) ||
		!nearly_equal(pid.error_past, 0.0f) ||
		!nearly_equal(pid.error_int, 0.0f) ||
		!nearly_equal(pid.out_offset, 0.0f) ||
		!nearly_equal(pid.out_deadzone, 0.0f))
	{
		printf("pid runtime state init failed\n");
		return 1;
	}

	if (!nearly_equal(pid.error_intmin, -20.0f) ||
		!nearly_equal(pid.error_intmax, 20.0f))
	{
		printf("pid integral limit init failed\n");
		return 1;
	}

	printf("ok\n");
	return 0;
}
