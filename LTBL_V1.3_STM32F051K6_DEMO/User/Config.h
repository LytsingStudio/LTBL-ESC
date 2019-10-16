
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stdint.h"

#define YES				1
#define NO				0

#define ThrottleCtrlType_Duty			0
#define ThrottleCtrlType_Force		1
#define ThrottleCtrlType_Speed		2

#define LowRPMProtect_Startup1Intv1		1000
#define LowRPMProtect_Startup1Intv2		600

/* 低速保护 - 低速判定起始值 */
#define LowRPMProtect_StartIntv				200
/* 低速保护 - 低速判定最大值 */
#define LowRPMProtect_EndIntv					3000
/* 低速保护 - 油门限制最大值 */
#define LowRPMProtect_StartThr				1024
/* 低速保护 - 油门限制最小值 */
#define LowRPMProtect_EndThr					200
	
/* 是否开启特殊启动模式1 */
#define CONFIG_UseStartup1						NO
/* 是否开启高速模式：高速模式将禁用换相后空闲事件处理，立即进入下次过零扫描 */
#define CONFIG_HighSpeedMode					NO
/* 是否开启信号丢失鸣叫：信号丢失后将会每 3 秒鸣叫一次 */
#define CONFIG_SignalLoseBeep					NO
/* 进入校准模式的高油门持续时间：当油门置于最高值一定事件后进入校准模式 */
#define CONFIG_CalibrateHoldMS				3000
/* 指示用于保存设置的 Flash 起始地址 */
#define CONFIG_FlashStartAddr					(0x08000000+1024*30)
/* 指示用于校验 Flash 区域的标记 */
#define CONFIG_FlashVerifyCode				0xfea2561f
/* 指示当前油门控制模式
 * ThrottleCtrlType_Duty  : 油门控制 PWM 占空比
 * ThrottleCtrlType_Force : 油门控制电机的扭矩
 * ThrottleCtrlType_Speed : 油门控制电机的速度
 */
#define CONFIG_ThrottleCtrlType				ThrottleCtrlType_Duty
/* 是否启用线性刹车：启用后，校准电机后会自动使用线性刹车 */
#define CONFIG_EnableLinearBrake			YES

/* 油门控制扭矩模式下允许的最大 正向/刹车 扭矩 相当于该值正脉宽完全堵转时的扭矩 */
#define CONFIG_ThrottleCtrlType_Force_Forware_MAX			200
#define CONFIG_ThrottleCtrlType_Force_Brake_MAX				140

/* parameter for skateboard 2 */
//#define CONFIG_ThrottleCtrlType_Force_Forware_MAX			200
//#define CONFIG_ThrottleCtrlType_Force_Brake_MAX				300
/* config parameter used for my skateboard */
//#define CONFIG_ThrottleCtrlType_Force_Forware_MAX			200
//#define CONFIG_ThrottleCtrlType_Force_Brake_MAX				140




void Config_RegesterData(void *pData, uint32_t length);
uint8_t Config_LoadData(void);
void Config_SaveData(void);
uint16_t Identification_GetBrakeThrottle(int16_t thr, uint32_t commIntv);
uint16_t Identification_GetCommonThrottle(uint32_t commIntv);
 
#endif

