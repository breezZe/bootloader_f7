#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "main.h"

#define STM32_FLASH_BASE 0x08000000 // STM32Flash Start Address
#define FLASH_WAITETIME  50000      // Wait Time


#if defined(DUAL_BANK)
    #define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) // 64 Kbytes 
    #define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) // 128 Kbytes 
    #define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) // 128 Kbytes 
    #define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) // 128 Kbytes 
    
    #define ADDR_FLASH_SECTOR_12    ((uint32_t)0x08080000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_13    ((uint32_t)0x08084000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_14    ((uint32_t)0x08088000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_15    ((uint32_t)0x0808C000) // 16 Kbytes 
    #define ADDR_FLASH_SECTOR_16    ((uint32_t)0x08090000) // 64 Kbytes 
    #define ADDR_FLASH_SECTOR_17    ((uint32_t)0x080A0000) // 128 Kbytes 
    #define ADDR_FLASH_SECTOR_18    ((uint32_t)0x080C0000) // 128 Kbytes 
    #define ADDR_FLASH_SECTOR_19    ((uint32_t)0x080E0000) // 128 Kbytes 
#else
    #define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) // 32 Kbytes  
    #define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) // 32 Kbytes  
    #define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) // 32 Kbytes  
    #define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) // 32 Kbytes  
    #define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) // 128 Kbytes  
    #define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) // 256 Kbytes  
    #define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000) // 256 Kbytes  
    #define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000) // 256 Kbytes  
#endif 

/** 
*  uint32_t STMFLASH_ReadWord(uint32_t faddr)
*  @brief  Read a word from address faddr. 
**/
uint32_t STMFLASH_ReadWord(uint32_t faddr);
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);
void iap_write_appbin(uint32_t appxaddr,uint8_t *appbuf,uint32_t appsize);
void iap_load_app(uint32_t appxaddr);
#endif
