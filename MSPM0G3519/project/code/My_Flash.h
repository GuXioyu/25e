#ifndef __MY_FLASH_H
#define __MY_FLASH_H

#include <stdint.h>

/*
 * 统一掉电参数存储区。
 *
 * 以后新增需要掉电保存的配置项，统一追加到 MyFlash_Data_t 中，
 * 不要让各个业务模块直接操作 Flash 地址，便于集中管理和迁移。
 */

#define MY_FLASH_DEFAULT_GRAY_THRESHOLD 700U

typedef struct
{
	uint32_t magic;          /* 数据有效标记，用于判断 Flash 中是否已有合法参数 */
	uint16_t gray_threshold; /* 黑线阈值：模拟量小于该值判黑；白底阈值为该值 + 100 */
	uint16_t reserved;       /* 预留对齐位，后续可扩展为其他 16 位参数 */
	uint32_t checksum;       /* 简单校验，防止掉电写入中断后误用损坏数据 */
} MyFlash_Data_t;

extern MyFlash_Data_t myFlashData;

void My_Flash_SetDefaults(void);
void My_Flash_Load(void);
uint8_t My_Flash_Save(void);

#endif
