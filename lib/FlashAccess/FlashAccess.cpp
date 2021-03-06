
/*
Utils meant to replace Ben Katz's original FlashWriter/PreferenceWriter libraries (which cannot be easily ported as they use a 
non-HAL flash library not available on STM32G474RET6). 

Purpose: facilitate reading/writing callibration data between flash memory and global variables declared in user_config.h
Maybe replace with EEPROM emulation or implement wear-leveling in the future. (10 000 write/erase cycles guaranteed per datasheet 
- probably good enough if recallibration occurs infrequently enough)
*/


//#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx.h"
#include "FlashAccess.h"
#include "user_config.h"


#define PAGELEN 2048
#define BANK2_START 0x0804000
#define RESERVED_PAGE  123      //max prog. upload reduced by 10k in platformio.ini, reserving pages 123 to 127 of bank 2
#define RESERVED_ADDR BANK2_START + PAGELEN*RESERVED_PAGE

#define FLOATSCOUNT 64
#define INTSCOUNT 256

#define FLOATS_ADDR RESERVED_ADDR
#define INTS_ADDR RESERVED_ADDR + FLOATSCOUNT*sizeof(float)


/**
  * @brief  Loads floats and ints from flash memory into global arrays
  */
void loadFromFlash(){
    for(int i = 0;i<FLOATSCOUNT;i++){
        __float_reg[i] = *((float*)(FLOATS_ADDR + i*sizeof(float)));
    }
    for(int i = 0;i<INTSCOUNT;i++){
        __int_reg[i] = *((int*)(INTS_ADDR + i*sizeof(float)));
    }
}

/**
  * @brief  Erase used page of flash memory to prepare for reprogramming
  * @retval uint32_t PageError. 0xFFFFFFFF means no problem
  */
int eraseReservedFlash(){
    FLASH_EraseInitTypeDef eraseStruct;
    eraseStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseStruct.Banks = FLASH_BANK_2;
    eraseStruct.Page = RESERVED_PAGE;
    eraseStruct.NbPages = 1;
    uint32_t error;
    HAL_FLASHEx_Erase(&eraseStruct, &error);
    return error;
}


/**
  * @brief  Saves floats and ints from global arrays into flash memory.
  * This should be done sparingly as it causes wear of flash memory and shortens it's lifespan.
  * Should it become necessary to perform this often, consider implementing some form of wear leveling.
  * @retval Zero when OK, nonzero when an error was encountered
  */
int saveToFlash(){
    unsigned int eraseError = eraseReservedFlash();
    if(eraseError!=0xFFFFFFFF) return 1;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if(status!=HAL_OK) return 2;
    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    if(status!=HAL_OK) return 3;


    for(int i=0;i<FLOATSCOUNT;i=i+2){
        uint64_t doubleWord = *((uint64_t*) __float_reg + i); //read two floats from array as one uint64
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLOATS_ADDR + i*sizeof(float), doubleWord);
        if(status!=HAL_OK) return 4;
    }
    for(int i=0;i<INTSCOUNT;i=i+2){
        uint64_t doubleWord = *((uint64_t*) __int_reg + i);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLOATS_ADDR + i*sizeof(int), doubleWord);
        if(status!=HAL_OK) return 5;
    }
    HAL_FLASH_Lock();
    return 0;
}








