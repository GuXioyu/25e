#include "My_Flash.h"

#include <string.h>

#include "zf_common_interrupt.h"

/*
 * MSPM0G3519 main flash ends at 0x0007FFFF.
 * This module owns the last 12 KB: 0x0007D000 - 0x0007FFFF.
 * The parameter block is stored in the last 1 KB page: 0x0007FC00.
 */
#define MY_FLASH_MAGIC                 0x464C5348UL  /* ASCII: "FLSH" */
#define MY_FLASH_CHECK_XOR             0xA5A55A5AUL
#define MY_FLASH_BASE_ADDR             0x0007D000UL
#define MY_FLASH_PAGE_SIZE             0x00000400UL
#define MY_FLASH_SECTION_PAGE_COUNT    2U
#define MY_FLASH_SECTION_COUNT         6U
#define MY_FLASH_SAVE_SECTION          (MY_FLASH_SECTION_COUNT - 1U)
#define MY_FLASH_SAVE_PAGE             (MY_FLASH_SECTION_PAGE_COUNT - 1U)
#define MY_FLASH_SAVE_ADDR             (MY_FLASH_BASE_ADDR + \
                                        (MY_FLASH_SAVE_SECTION * MY_FLASH_SECTION_PAGE_COUNT + \
                                         MY_FLASH_SAVE_PAGE) * MY_FLASH_PAGE_SIZE)
#define MY_FLASH_OPERATION_TIME_OUT    0x0FFFUL
#define MY_FLASH_WORD_COUNT            ((sizeof(MyFlash_Data_t) + sizeof(uint32_t) - 1U) / sizeof(uint32_t))
#define MY_FLASH_PAGE_WORD_CAP         (MY_FLASH_PAGE_SIZE / (sizeof(uint32_t) * 2U))

typedef char my_flash_size_check[(sizeof(MyFlash_Data_t) <= MY_FLASH_PAGE_SIZE) ? 1 : -1];
typedef char my_flash_word_check[(MY_FLASH_WORD_COUNT <= MY_FLASH_PAGE_WORD_CAP) ? 1 : -1];

MyFlash_Data_t myFlashData = {0};

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

static void My_Flash_CopyFromWords(MyFlash_Data_t *data, const uint32_t *words)
{
    memset(data, 0, sizeof(*data));
    memcpy(data, words, sizeof(*data));
}

static void My_Flash_CopyToWords(uint32_t *words, const MyFlash_Data_t *data)
{
    memset(words, 0xFF, MY_FLASH_WORD_COUNT * sizeof(words[0]));
    memcpy(words, data, sizeof(*data));
}

static uint8_t My_Flash_PageHasData(void)
{
    uint32_t offset;

    for (offset = 0; offset < MY_FLASH_PAGE_SIZE; offset += 8U)
    {
        if (0xFFFFFFFFUL != (*(volatile uint32_t *)(MY_FLASH_SAVE_ADDR + offset)))
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t My_Flash_ErasePage(void)
{
    volatile uint32_t timeOut = MY_FLASH_OPERATION_TIME_OUT;
    uint32_t status;
    uint32_t primask = interrupt_global_disable();

    DL_FlashCTL_unprotectSector(FLASHCTL, MY_FLASH_SAVE_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);

    FLASHCTL->GEN.CMDTYPE = (uint32_t)DL_FLASHCTL_COMMAND_SIZE_SECTOR |
                            (uint32_t)DL_FLASHCTL_COMMAND_TYPE_ERASE;
    FLASHCTL->GEN.CMDADDR = MY_FLASH_SAVE_ADDR;
    FLASHCTL->GEN.CMDEXEC = FLASHCTL_CMDEXEC_VAL_EXECUTE;

    do
    {
        timeOut--;
        status = FLASHCTL->GEN.STATCMD;
    } while ((DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS == status) && (0U != timeOut));

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    interrupt_global_enable(primask);

    return ((DL_FLASHCTL_COMMAND_STATUS_PASSED == status) && (0U != timeOut)) ? 0U : 1U;
}

static void My_Flash_ReadWords(uint32_t *words, uint16_t len)
{
    uint16_t index;

    for (index = 0U; index < len; index++)
    {
        words[index] = *(volatile uint32_t *)(MY_FLASH_SAVE_ADDR + ((uint32_t)index * 8U));
    }
}

static uint8_t My_Flash_WriteWords(const uint32_t *words, uint16_t len)
{
    uint16_t index;
    uint8_t returnState = 0U;
    uint32_t flashAddr = MY_FLASH_SAVE_ADDR;
    uint32_t primask;

    if (My_Flash_PageHasData())
    {
        if (My_Flash_ErasePage())
        {
            return 1U;
        }
    }

    primask = interrupt_global_disable();

    for (index = 0U; index < len; index++)
    {
        volatile uint32_t timeOut = MY_FLASH_OPERATION_TIME_OUT;
        uint32_t status;

        DL_FlashCTL_unprotectSector(FLASHCTL, MY_FLASH_SAVE_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);

        FLASHCTL->GEN.CMDTYPE = (uint32_t)DL_FLASHCTL_COMMAND_SIZE_ONE_WORD |
                                (uint32_t)DL_FLASHCTL_COMMAND_TYPE_PROGRAM;
        FLASHCTL->GEN.CMDBYTEN = DL_FLASHCTL_PROGRAM_32_WITH_ECC;
        FLASHCTL->GEN.CMDADDR = flashAddr;
        FLASHCTL->GEN.CMDDATA0 = words[index];
        FLASHCTL->GEN.CMDEXEC = FLASHCTL_CMDEXEC_VAL_EXECUTE;

        do
        {
            timeOut--;
            status = FLASHCTL->GEN.STATCMD;
        } while ((DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS == status) && (0U != timeOut));

        if ((DL_FLASHCTL_COMMAND_STATUS_PASSED != status) || (0U == timeOut))
        {
            returnState = 1U;
            break;
        }

        DL_FlashCTL_executeClearStatus(FLASHCTL);
        flashAddr += 8U;
    }

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    interrupt_global_enable(primask);

    return returnState;
}

void My_Flash_SetDefaults(void)
{
    memset(&myFlashData, 0, sizeof(myFlashData));
    myFlashData.gray_threshold = MY_FLASH_DEFAULT_GRAY_THRESHOLD;
    My_Flash_Finalize(&myFlashData);
}

void My_Flash_Load(void)
{
    uint32_t flashWords[MY_FLASH_WORD_COUNT];
    MyFlash_Data_t stored;

    memset(flashWords, 0xFF, sizeof(flashWords));
    My_Flash_ReadWords(flashWords, MY_FLASH_WORD_COUNT);
    My_Flash_CopyFromWords(&stored, flashWords);

    if (My_Flash_IsValid(&stored))
    {
        myFlashData = stored;
    }
    else
    {
        My_Flash_SetDefaults();
    }
}

uint8_t My_Flash_Save(void)
{
    uint32_t flashWords[MY_FLASH_WORD_COUNT];
    MyFlash_Data_t dataToSave = myFlashData;

    My_Flash_Finalize(&dataToSave);
    My_Flash_CopyToWords(flashWords, &dataToSave);

    if (My_Flash_WriteWords(flashWords, MY_FLASH_WORD_COUNT))
    {
        return 0;
    }

    myFlashData = dataToSave;
    return 1;
}
