
#include "identification.h"

#include "arduino.h"

#include "ltbl.h"
#include "ltbl_signal.h"

static uint32_t identStage = 0;
static uint32_t lastCapturedMS = 0;

static LTBL_CommEventHandler_TypeDef lastCommHandler = 0;
static LTBL_SIGNAL_PWM_CapturedEventHandler_TypeDef lastPWMHandler = 0;
static LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef lastDshotHandler = 0;

void Identification_Abort()
{
	LTBL_DISABLEIT;
	identStage = 0;
	LTBL_SetPWMMode(LTBL_Default_PWM_Mode);
	LTBL_UpdateThrottle(0);
	LTBL_AttachCommEvent(lastCommHandler);
	LTBL_PWM_AttachCaptureEvent(lastPWMHandler);
	LTBL_DSHOT_AttachCaptureEvent(lastDshotHandler);
	LTBL_ENABLEIT;
}

#define Identification_TestStepCount			10
#define Identification_TestDurationMS			8000

uint32_t Identification_SpeedTable1[Identification_TestStepCount] = { 0 };
uint32_t Identification_SpeedTable2[Identification_TestStepCount] = { 0 };

static void ltblPWMSignalCaptured(int32_t thr)
{
	lastCapturedMS = millis();
	if(thr)
	{
		Identification_Abort();
	}
}
static void ltblDshotSignalCaptured(int32_t thr, uint8_t *pInfo)
{
	ltblPWMSignalCaptured(thr);
}
static void commHandler()
{
	static uint32_t stage = 0;
	static uint32_t beginMS = 0;
	if(millis() - lastCapturedMS > 100)
	{
		Identification_Abort();
	}
	if(identStage)
	{
		if(identStage == 1)
		{
			stage = 0;
			beginMS = millis();
			identStage = 2;
		}
		else if(identStage == 2)
		{
			uint32_t nowIntv = (millis() - beginMS);
			int16_t testThr = nowIntv * LTBL_PWM_RESOLUTION / Identification_TestDurationMS;
			uint32_t crtStage = nowIntv * Identification_TestStepCount / Identification_TestDurationMS;
			if(crtStage != stage)
			{
				stage = crtStage;
				Identification_SpeedTable1[crtStage - 1] = 1e6 / LTBL_GetAvgCommInterval();
			}
			if(testThr > LTBL_PWM_RESOLUTION)
			{
				testThr = LTBL_PWM_RESOLUTION;
				beginMS = millis();
				identStage = 3;
			}
			LTBL_UpdateThrottle(testThr);
		}
		else if(identStage == 3)
		{
			uint32_t nowIntv = (millis() - beginMS);
			int16_t testThr = LTBL_PWM_RESOLUTION - nowIntv * LTBL_PWM_RESOLUTION / Identification_TestDurationMS;
			uint32_t crtStage = nowIntv * Identification_TestStepCount / Identification_TestDurationMS;
			if(crtStage != stage)
			{
				stage = crtStage;
				if(crtStage < Identification_TestStepCount)
				{
					Identification_SpeedTable2[Identification_TestStepCount - crtStage - 1] = 1e6 / LTBL_GetAvgCommInterval();
				}
			}
			if(testThr <= 0)
			{
				testThr = 0;
				beginMS = millis();
				identStage = 4;
			}
			LTBL_UpdateThrottle(testThr);
		}
		else
		{
			Identification_Abort();
			Config_SaveData();
			LTBL_Stop();
		}
	}
	else
	{
		LTBL_UpdateThrottle(0);
	}
}

void Identification_Init()
{
	Config_RegesterData(Identification_SpeedTable1, sizeof(Identification_SpeedTable1));
	Config_RegesterData(Identification_SpeedTable2, sizeof(Identification_SpeedTable2));
}
uint16_t Identification_GetCommonThrottle(uint32_t commIntv)
{
	int index = 0;
	uint32_t spd = 1e6 / commIntv;
	uint32_t dataSpd1 = 0;
	uint32_t dataSpd2 = 0;
	uint32_t Thr = 0;
	for(; index < Identification_TestStepCount; index++)
	{
		dataSpd1 = dataSpd2;
		dataSpd2 = (Identification_SpeedTable1[index] + Identification_SpeedTable2[index]) / 2;
		if(dataSpd2 >= spd)
		{
			break;
		}
	}
	if(index == 0)
	{
		uint32_t pct = 10000 * (spd) / (dataSpd2);
		uint32_t thr2 = LTBL_PWM_RESOLUTION / Identification_TestStepCount;
		Thr = thr2 * pct / 10000;
	}
	else if(index >= Identification_TestStepCount)
	{
		Thr = LTBL_PWM_RESOLUTION;
	}
	else
	{
		uint32_t pct = 10000 * (spd - dataSpd1) / (dataSpd2 - dataSpd1);
		uint32_t thr1 = LTBL_PWM_RESOLUTION / Identification_TestStepCount + (index - 1) * LTBL_PWM_RESOLUTION / Identification_TestStepCount;
		uint32_t thr2 = LTBL_PWM_RESOLUTION / Identification_TestStepCount + index * LTBL_PWM_RESOLUTION / Identification_TestStepCount;
		Thr = (thr1 + (thr2 - thr1) * pct / 10000);
	}
	return Thr;
}
uint16_t Identification_GetBrakeThrottle(int16_t thr, uint32_t commIntv)
{
	if(Identification_SpeedTable1[Identification_TestStepCount - 1] > 50 &&
		Identification_SpeedTable1[Identification_TestStepCount - 1] <= 100000 &&
		Identification_SpeedTable2[Identification_TestStepCount - 1] > 50 &&
		Identification_SpeedTable2[Identification_TestStepCount - 1] <= 100000
		)
	{
		uint32_t startThr = LTBL_PWM_RESOLUTION - Identification_GetCommonThrottle(commIntv);
		startThr = startThr * Identification_BrakeLinearPercent / Identification_BrakeLinearPercent_MAX;
		uint32_t trueThr = thr * (LTBL_PWM_RESOLUTION - startThr) / LTBL_PWM_RESOLUTION + startThr;
		return trueThr;
	}
	else
	{
		return thr;
	}
}

void Identification_Begin()
{
	lastCommHandler = LTBL_GetCommEventHandler();
	lastPWMHandler = LTBL_PWM_GetCaptureEventHandler();
	lastDshotHandler = LTBL_DSHOT_GetCaptureEventHandler();
	
	lastCapturedMS = millis();
	LTBL_AttachCommEvent(commHandler);
	LTBL_PWM_AttachCaptureEvent(ltblPWMSignalCaptured);
	LTBL_DSHOT_AttachCaptureEvent(ltblDshotSignalCaptured);
	
	identStage = 1;
	
	LTBL_SetPWMMode(LTBL_PWM_MODE_DAMPED);
	LTBL_Run();
}

