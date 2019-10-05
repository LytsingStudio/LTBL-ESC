
#include "arduino.h"
#include "hardwareSerial.h"

#include "stm32f0xx_conf.h" 
#include "stm32f0xx_rcc.h"

#include "ltbl.h"
#include "ltbl_signal.h"

#include "identification.h"

#include "config.h"

extern "C"
{
	void HardFault_Handler()
	{
		/* Reset when hardfault occured */
		NVIC_SystemReset();
	}
}

static uint8_t  IsCalibrated = 0;
static uint64_t lastCapMS = millis();
static uint64_t lastWorkMS = millis();

void InternalClocks_Init();
void Motor_Commucated();
void Motor_PlayStartupMusic(void);
void Motor_PlaySignalResetMusic(void);
void Motor_PlaySignalDetectMusic(void);
void Motor_PlayCalibratePrepareMusic(void);
void Motor_PlayCalibrateBeginMusic(void);
void Motor_PlayCalibrateAbortMusic(void);
void Motor_PlayLoadFailedMusic(void);
void Signal_PWM_Captured(int32_t thr);
void Signal_DSHOT_Captured(int32_t thr, uint8_t *info);

void Motor_BeginCalibrate(void);

int main()
{
	/* Use internal clock source, running @ 48MHz */
	InternalClocks_Init();
	
	/* Initialize system tick */
	Delay_Init();
	
	/* Initialize LTBL Lib */
	LTBL_Init();
	
	/* Initialize applications */
	Identification_Init();
	
	/* Play startup music */
	Motor_PlayStartupMusic();
	
	/* Initialize LTBL signal libs */
	do
	{
		/* Check protocol type */
		LTBL_SIGNAL_TYPES sigType = LTBL_Signal_GetSignalType();
		if(sigType == LTBL_SIGNAL_TYPE_PWM)
		{
			/* Use PWM Protocol */
			LTBL_PWM_Init();
			Motor_PlaySignalDetectMusic();
			break;
		}
		else if(sigType == LTBL_SIGNAL_TYPE_DSHOT)
		{
			/* Use DSHOT Protocol */
			LTBL_DSHOT_Init();
			Motor_PlaySignalDetectMusic();
			break;
		}
		else
		{
			/* Unknown Protocol, Beep and try again */
			#if( CONFIG_SignalLoseBeep == YES )
			LTBL_Tone(1000, 300, LTBL_TONE_VOLUME_RECOMM);
			#endif
		}
	} while(1);
	
	do
	{
		/* Check throttle state */
		uint32_t state = LTBL_SIGNAL_GetThrottleState();
		static uint32_t lastNotResetMS = millis();
		if(state == LTBL_SIGNAL_STATE_RESET)
		{
			/* throttle is zero, allow continue */
			break;
		}
		else if(state == LTBL_PWM_RESOLUTION)
		{
			if(millis() - lastNotResetMS > CONFIG_CalibrateHoldMS)
			{
				Motor_BeginCalibrate();
				lastNotResetMS = millis();
			}
		}
		else
		{
			lastNotResetMS = millis();
		}
	} while(1);
	
	Motor_PlaySignalResetMusic();
	
	/* Reset signal state timer */
	lastCapMS = millis();
	if(Config_LoadData())
	{
		IsCalibrated = 1;
	}
	else
	{
		IsCalibrated = 0;
		delay(500);
		Motor_PlayLoadFailedMusic();
	}
	
	/* Attach events */
	LTBL_PWM_AttachCaptureEvent(Signal_PWM_Captured);
	LTBL_DSHOT_AttachCaptureEvent(Signal_DSHOT_Captured);
	LTBL_AttachCommEvent(Motor_Commucated);	
	
	/* Motor begin to run */
	while(1)
	{
		LTBL_Run();
	}
}

/* -----------------------------------------------------------------*/

void InternalClocks_Init()
{
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	while(RCC_GetSYSCLKSource());
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLKConfig(RCC_HCLK_Div1);
	RCC_PLLCmd(DISABLE);
	RCC_HSICmd(ENABLE);
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_12);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) ;
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while(RCC_GetSYSCLKSource()!=0x08);
	
	pinMode(PB6, OUTPUT);
	pinMode(PB5, OUTPUT);
	digitalWrite_HIGH(PB5);
}
void Motor_Commucated()
{
	/* Set throttle to zero when cannot capture signal  */
	if(millis() - lastCapMS > LTBL_SIGNAL_DETECT_LOSETIME)
	{
		LTBL_UpdateThrottle(0);
		if(millis() - lastCapMS > LTBL_SIGNAL_DETECT_TIMEOUT)
		{
			NVIC_SystemReset();
		}
	}
	/* Output comm signal used for debug or speed detecting */
	togglePin(PB6);
	if(millis() - lastWorkMS < 10000)
	{
		digitalWrite_HIGH(PB5);
	}
	else
	{
		digitalWrite_LOW(PB5);
	}
}
void Motor_PlayStartupMusic()
{
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(4000, 200, LTBL_TONE_VOLUME_RECOMM);
	delay(30);
}
void Motor_PlaySignalResetMusic()
{
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(4000, 120, LTBL_TONE_VOLUME_RECOMM);
}
void Motor_PlaySignalDetectMusic()
{
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(500);
}
void Signal_PWM_Captured(int32_t thr)
{
	#define LowRPMProtect_StartIntv		200
	#define LowRPMProtect_EndIntv			3000
	#define LowRPMProtect_StartThr		1024
	#define LowRPMProtect_EndThr			200
	
	lastCapMS = millis();
	int interval = LTBL_GetAvgCommInterval();
	if(thr > 0)
	{
		int limit = 1024;
		if(interval > LowRPMProtect_StartIntv)
		{
			int nowSpeed = 1e6 / interval;
			int startSpd = 1e6 / LowRPMProtect_StartIntv;
			int endSpd = 1e6 / LowRPMProtect_EndIntv;
			limit = map(constrain(nowSpeed, endSpd, startSpd),
			startSpd,
			endSpd,
			LowRPMProtect_StartThr,
			LowRPMProtect_EndThr
			);
			if(thr > limit)
			{
				thr = limit;
			}
		}
		lastWorkMS = millis();
		LTBL_SetMode(LTBL_Mode_Normal);
		LTBL_UpdateThrottle(thr);
	}
	else if(thr < 0)
	{
		if(IsCalibrated)
		{
			LTBL_SetMode(LTBL_Mode_Brake);
			LTBL_UpdateThrottle(Identification_GetBrakeThrottle(-thr, interval));
		}
		else
		{
			LTBL_SetMode(LTBL_Mode_Brake);
			LTBL_UpdateThrottle(-thr);
		}
	}
	else
	{
		LTBL_SetMode(LTBL_Mode_Free);
		LTBL_UpdateThrottle(0);
	}
}
void Signal_DSHOT_Captured(int32_t thr, uint8_t *info)
{
	Signal_PWM_Captured(thr);
}
void Motor_PlayCalibratePrepareMusic()
{
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(4000, 200, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(4000, 200, LTBL_TONE_VOLUME_RECOMM);
	delay(30);
}
void Motor_PlayCalibrateBeginMusic()
{
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
}
void Motor_PlayCalibrateAbortMusic()
{
	LTBL_Tone(3000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
}
void Motor_PlayLoadFailedMusic()
{
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
	LTBL_Tone(2000, 120, LTBL_TONE_VOLUME_RECOMM);
	delay(25);
}
void Motor_BeginCalibrate()
{
	Motor_PlayCalibratePrepareMusic();
	uint32_t lastNotResetMS = millis();
	do
	{
		/* Check throttle state */
		uint32_t state = LTBL_SIGNAL_GetThrottleState();
		if(state == LTBL_SIGNAL_STATE_RESET)
		{
			/* throttle is zero, allow continue */
			break;
		}
		else if(state == LTBL_PWM_RESOLUTION)
		{
			if(millis() - lastNotResetMS > CONFIG_CalibrateHoldMS)
			{
				Motor_PlayCalibrateAbortMusic();
				return;
			}
		}
		else
		{
			lastNotResetMS = millis();
		}
	} while(1);
	Motor_PlayCalibrateBeginMusic();
	Identification_Begin();
	Motor_PlayCalibrateAbortMusic();
}

