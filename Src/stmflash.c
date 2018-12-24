#include "stmflash.h"
#include "main.h"
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t *)faddr; 
}

uint16_t STMFLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	return FLASH_SECTOR_7;	
}

void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
  FLASH_EraseInitTypeDef FlashEraseInit;
  HAL_StatusTypeDef FlashStatus=HAL_OK;
  uint32_t SectorError=0;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)
		return;
    
 	HAL_FLASH_Unlock(); 
	addrx=WriteAddr;	
	endaddr=WriteAddr+NumToWrite*4;
    
	if(addrx<0X1FF00000)
	{
		while(addrx<endaddr)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)
			{   
					FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 
					FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx); 
					FlashEraseInit.NbSectors=1;        
					FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3; 
				
					if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
					{
						break;	
					}
					
					SCB_CleanInvalidateDCache();                         
			}
			else 
			{
				addrx+=4;
			}
			FLASH_WaitForLastOperation(FLASH_WAITETIME);             
		}
	}
  FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);         
	if(FlashStatus==HAL_OK)
	{
		while(WriteAddr<endaddr)
		{
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)
			{ 
				break;
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	HAL_FLASH_Lock(); 
} 

void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);
		ReadAddr+=4;
	}
}


void   (*jump2app)(void);
typedef  void (*iapfun)(void);

void iap_load_app(uint32_t appxaddr)
{
	if(((*(uint32_t*)appxaddr)&0x2FF00000)==0x20000000)
	{ 
		jump2app=(iapfun)*(uint32_t*)(appxaddr+4);
		__set_MSP(*( uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);	
		jump2app();
	}
}
uint32_t iapbuf[512];

void iap_write_appbin(uint32_t appxaddr,uint8_t *appbuf,uint32_t appsize)
{
	uint32_t t;
	uint16_t i=0;
	uint32_t temp;
	uint32_t fwaddr=appxaddr;
	uint8_t *dfu=appbuf;
	for(t=0;t<appsize;t+=4)
	{						   
		temp=(uint32_t)dfu[3]<<24;   
		temp|=(uint32_t)dfu[2]<<16;    
		temp|=(uint32_t)dfu[1]<<8;
		temp|=(uint32_t)dfu[0];	  
		dfu+=4;
		iapbuf[i++]=temp;	    
		if(i==512)
		{
			i=0; 
			STMFLASH_Write(fwaddr,iapbuf,512);
			fwaddr+=2048;
		}
	} 
	if(i)STMFLASH_Write(fwaddr,iapbuf,i);  
}
