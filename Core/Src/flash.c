#include "flash.h"
#include "stm_log.h"

HAL_StatusTypeDef Flash_WriteWord(uint32_t start_address, uint32_t data)
{
    if (Flash_ReadAddress(start_address) != -1)
        return HAL_ERROR;

    /* Struct for erase process */
    // FLASH_EraseInitTypeDef EraseInitStruct;
    // EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    // EraseInitStruct.PageAddress = start_address;
    // EraseInitStruct.NbPages = (end_address - start_address) / FLASH_PAGE_SIZE;
    // uint32_t PageError = 0xFFFFFFFF;

    // /* Erase Flash based on initialize struct */
    // HAL_StatusTypeDef err = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    // if (err != HAL_OK) {
    //     return flash_status;
    // }
    STM_LOGV("Flash", "write data to 0x%x: 0x%x - dec: %d", start_address, data, data);
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address, data);
    HAL_FLASH_Lock();
    FLASH_READ(start_address);
    return err;
}

int Flash_ReadAddress(uint32_t flash_address)
{
    return *(volatile int*)(flash_address);
}

int8_t Flash_GetAddressPage(uint32_t flash_address)
{
    uint32_t currentSector;
    currentSector = flash_address / ADDR_FLASH_PAGE_0;
    if (IS_PAGE_IN_RANGE(currentSector))
        return currentSector;
    else {
        return -1;
    }
}

HAL_StatusTypeDef Flash_ErasePage(uint32_t start_address, uint32_t nb_of_delete_pages)
{
    FLASH_EraseInitTypeDef EraseInitStruct = { 0 };
    uint32_t PageError = 0;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = start_address;
    EraseInitStruct.NbPages = nb_of_delete_pages;

    HAL_StatusTypeDef err = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (PageError != 0xFFFFFFFF) {
        STM_LOGE("Flash", "PageError != 0xFFFFFFFF, %d", PageError);
    }

    HAL_FLASH_Lock();
    return err;
}
