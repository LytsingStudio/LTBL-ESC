
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stdint.h"
#include "ltbl.h"

#define CONFIG_SignalLoseBeep		NO
#define CONFIG_CalibrateHoldMS	3000
#define CONFIG_FlashStartAddr		(0x08000000+1024*30)
#define CONFIG_FlashVerifyCode	0xfea2561f

void Config_RegesterData(void *pData, uint32_t length);
uint8_t Config_LoadData(void);
void Config_SaveData(void);
uint16_t Identification_GetBrakeThrottle(int16_t thr, uint32_t commIntv);

#endif

