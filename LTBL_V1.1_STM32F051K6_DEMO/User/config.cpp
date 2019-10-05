
#include "config.h"
#include "stm32f0xx_flash.h"

static uint32_t DataCount = 0;
static uint32_t DataAddr[32];
static uint32_t DataLength[32];

void Config_RegesterData(void *pData, uint32_t length)
{
	DataAddr[DataCount] = (uint32_t)pData;
	DataLength[DataCount] = length;
	if(DataCount < 32)
	{
		DataCount ++;
	}
}
uint8_t Config_LoadData()
{
	uint16_t *dproc = (uint16_t *)(CONFIG_FlashStartAddr + sizeof(CONFIG_FlashVerifyCode));
	if(*(uint32_t *)CONFIG_FlashStartAddr == CONFIG_FlashVerifyCode)
	{
		uint32_t i = 0, j = 0;
		for(; i < DataCount; i++)
		{
			for(j = 0; j < (DataLength[i] / 2) + (DataLength[i] & 1); j++)
			{
				((uint16_t *)(DataAddr[i]))[j] = *dproc ++;
			}
		}
		return 1;
	}
	return 0;
}
void Config_SaveData()
{
	uint16_t *dproc = (uint16_t *)(CONFIG_FlashStartAddr);
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);
	FLASH_ErasePage(CONFIG_FlashStartAddr);
	FLASH_ProgramHalfWord((uint32_t)(dproc++), (CONFIG_FlashVerifyCode) & 0xffff);
	FLASH_ProgramHalfWord((uint32_t)(dproc++), (CONFIG_FlashVerifyCode >> 16) & 0xffff);
	if(*(uint32_t *)CONFIG_FlashStartAddr == CONFIG_FlashVerifyCode)
	{
		uint32_t i = 0, j = 0;
		for(; i < DataCount; i++)
		{
			for(j = 0; j < (DataLength[i] / 2) + (DataLength[i] & 1); j++)
			{
				FLASH_ProgramHalfWord((uint32_t)(dproc ++), ((uint16_t *)(DataAddr[i]))[j]);
			}
		}
	}
	FLASH_Lock();
}

