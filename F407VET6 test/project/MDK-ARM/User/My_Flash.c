#include "My_Flash.h"

#include <string.h>

#if defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#endif

/*
 * STM32F407VET6 为 512KB Flash。
 * Sector 7 地址范围：0x08060000 ~ 0x0807FFFF，位于片内 Flash 末尾。
 * 本工程把它作为参数保存区使用；如果后续程序体积接近该区域，需要调整链接脚本或换扇区。
 */
#define MY_FLASH_MAGIC          0x464C5348UL  /* ASCII: "FLSH" */
#define MY_FLASH_CHECK_XOR      0xA5A55A5AUL
#define MY_FLASH_SAVE_ADDRESS   0x08060000UL

#if defined(STM32F407xx)
#define MY_FLASH_SAVE_SECTOR    FLASH_SECTOR_7
#endif

MyFlash_Data_t myFlashData = {0};

#if !defined(STM32F407xx)
/* 主机测试环境没有真实 Flash，用静态变量模拟掉电保存区。 */
static MyFlash_Data_t myFlashEmuStorage = {0};
#endif

static uint32_t My_Flash_CalcChecksum(const MyFlash_Data_t *data)
{
	return data->magic ^
		   (uint32_t)data->gray_threshold ^
		   ((uint32_t)data->reserved << 16) ^
		   MY_FLASH_CHECK_XOR;
}

static void My_Flash_Finalize(MyFlash_Data_t *data)
{
	data->magic = MY_FLASH_MAGIC;
	data->checksum = My_Flash_CalcChecksum(data);
}

static uint8_t My_Flash_IsValid(const MyFlash_Data_t *data)
{
	if (data->magic != MY_FLASH_MAGIC)
	{
		return 0;
	}

	return (My_Flash_CalcChecksum(data) == data->checksum) ? 1U : 0U;
}

/**
 * @brief  恢复所有掉电参数的默认值。
 * @note   这里只改 RAM 中的 myFlashData，不会立刻写入 Flash。
 */
void My_Flash_SetDefaults(void)
{
	memset(&myFlashData, 0, sizeof(myFlashData));
	myFlashData.gray_threshold = MY_FLASH_DEFAULT_GRAY_THRESHOLD;
	My_Flash_Finalize(&myFlashData);
}

/**
 * @brief  从 Flash 参数区读取配置；无有效数据时使用默认值。
 */
void My_Flash_Load(void)
{
#if defined(STM32F407xx)
	const MyFlash_Data_t *stored = (const MyFlash_Data_t *)MY_FLASH_SAVE_ADDRESS;
#else
	const MyFlash_Data_t *stored = &myFlashEmuStorage;
#endif

	if (My_Flash_IsValid(stored))
	{
		myFlashData = *stored;
	}
	else
	{
		My_Flash_SetDefaults();
	}
}

/**
 * @brief  把 RAM 中的 myFlashData 写入 Flash，实现掉电保存。
 * @retval 1 保存成功
 * @retval 0 保存失败
 * @note   STM32 Flash 写入前必须擦除整个扇区，调用时不要放在中断里。
 */
uint8_t My_Flash_Save(void)
{
	MyFlash_Data_t dataToSave = myFlashData;
	My_Flash_Finalize(&dataToSave);

#if defined(STM32F407xx)
	HAL_StatusTypeDef status;
	uint32_t sectorError = 0;
	FLASH_EraseInitTypeDef eraseInit;
	const uint32_t *word = (const uint32_t *)&dataToSave;
	uint32_t address = MY_FLASH_SAVE_ADDRESS;

	HAL_FLASH_Unlock();

	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Sector = MY_FLASH_SAVE_SECTOR;
	eraseInit.NbSectors = 1;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
	if (status == HAL_OK)
	{
		for (uint32_t i = 0; i < (uint32_t)(sizeof(MyFlash_Data_t) / sizeof(uint32_t)); i++)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word[i]);
			if (status != HAL_OK)
			{
				break;
			}
			address += sizeof(uint32_t);
		}
	}

	HAL_FLASH_Lock();

	if (status != HAL_OK)
	{
		return 0;
	}
#else
	myFlashEmuStorage = dataToSave;
#endif

	myFlashData = dataToSave;
	return 1;
}
