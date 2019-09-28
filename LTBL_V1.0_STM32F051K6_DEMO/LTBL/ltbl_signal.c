
#include "ltbl_signal.h"

static volatile uint32_t ltblPWMSignalLastCapMS = 0;
static volatile uint32_t ltblDshotSignalLastCapMS = 0;
static volatile int32_t ltblSignalCrtThrottle = 0;

static void ltblPWMSignalCaptured(int32_t thr)
{
	ltblSignalCrtThrottle = thr;
	ltblPWMSignalLastCapMS = LTBL_SIGNAL_SYSTEM_MS;
}
static void ltblDshotSignalCaptured(int32_t thr, uint8_t *pInfo)
{
	ltblSignalCrtThrottle = thr;
	ltblDshotSignalLastCapMS = LTBL_SIGNAL_SYSTEM_MS;
}

LTBL_SIGNAL_TYPES LTBL_Signal_GetSignalType()
{
	uint8_t success = 0;
	volatile uint32_t nowMS = LTBL_SIGNAL_SYSTEM_MS;
	volatile uint32_t tryMS = 0;
	LTBL_PWM_AttachCaptureEvent(ltblPWMSignalCaptured);
	LTBL_DSHOT_AttachCaptureEvent(ltblDshotSignalCaptured);
	while(LTBL_SIGNAL_SYSTEM_MS - nowMS < LTBL_SIGNAL_DETECT_TIMEOUT)
	{
		/* Try dshot protocol */
		LTBL_DSHOT_Init();
		success = 1;
		tryMS = LTBL_SIGNAL_SYSTEM_MS;
		ltblDshotSignalLastCapMS = LTBL_SIGNAL_SYSTEM_MS;
		while(LTBL_SIGNAL_SYSTEM_MS - tryMS < LTBL_SIGNAL_DETECT_STABTIME)
		{
			if(LTBL_SIGNAL_SYSTEM_MS - ltblDshotSignalLastCapMS > LTBL_SIGNAL_DETECT_LOSETIME)
			{
				success = 0;
				break;
			}
		}
		LTBL_DSHOT_Dispose();
		if(success)
		{
			LTBL_PWM_AttachCaptureEvent(0);
			LTBL_DSHOT_AttachCaptureEvent(0);
			return LTBL_SIGNAL_TYPE_DSHOT;
		}
		/* Try pwm protocol */
		LTBL_PWM_Init();
		success = 1;
		tryMS = LTBL_SIGNAL_SYSTEM_MS;
		ltblPWMSignalLastCapMS = LTBL_SIGNAL_SYSTEM_MS;
		while(LTBL_SIGNAL_SYSTEM_MS - tryMS < LTBL_SIGNAL_DETECT_STABTIME)
		{
			if(LTBL_SIGNAL_SYSTEM_MS - ltblPWMSignalLastCapMS > LTBL_SIGNAL_DETECT_LOSETIME)
			{
				success = 0;
				break;
			}
		}
		LTBL_PWM_Dispose();
		if(success)
		{
			LTBL_PWM_AttachCaptureEvent(0);
			LTBL_DSHOT_AttachCaptureEvent(0);
			return LTBL_SIGNAL_TYPE_PWM;
		}
	}
	LTBL_PWM_AttachCaptureEvent(0);
	LTBL_DSHOT_AttachCaptureEvent(0);
	return LTBL_SIGNAL_TYPE_Unknown;
}

LTBL_SIGNAL_STATES LTBL_SIGNAL_GetThrottleState()
{
	uint8_t errCode = 0;
	volatile uint32_t nowMS = LTBL_SIGNAL_SYSTEM_MS;
	volatile uint32_t tryMS = 0;
	ltblPWMSignalLastCapMS = LTBL_SIGNAL_SYSTEM_MS;
	ltblDshotSignalLastCapMS = LTBL_SIGNAL_SYSTEM_MS;
	LTBL_PWM_AttachCaptureEvent(ltblPWMSignalCaptured);
	LTBL_DSHOT_AttachCaptureEvent(ltblDshotSignalCaptured);
	while(LTBL_SIGNAL_SYSTEM_MS - nowMS < LTBL_SIGNAL_DETECT_STABTIME)
	{
		if(LTBL_SIGNAL_SYSTEM_MS - ltblPWMSignalLastCapMS > LTBL_SIGNAL_DETECT_LOSETIME &&
			LTBL_SIGNAL_SYSTEM_MS - ltblDshotSignalLastCapMS > LTBL_SIGNAL_DETECT_LOSETIME)
		{
			errCode = 1;
			break;
		}
		if(ltblSignalCrtThrottle != 0)
		{
			errCode = 2;
			break;
		}
	}
	LTBL_PWM_AttachCaptureEvent(0);
	LTBL_DSHOT_AttachCaptureEvent(0);
	if(errCode == 1)
	{
		return LTBL_SIGNAL_STATE_LOSE;
	}
	else if(errCode == 2)
	{
		return LTBL_SIGNAL_STATE_NOTZERO;
	}
	else
	{
		return LTBL_SIGNAL_STATE_RESET;
	}
}

#if( LTBL_SIGNAL_PWM_HANDLER == LTBL_SIGNAL_DSHOT_HANDLER)
void LTBL_SIGNAL_PWM_HANDLER()
{
	LTBL_DSHOT_Handler();
	LTBL_PWM_Handler();
}
#else
void LTBL_SIGNAL_PWM_HANDLER()
{
	LTBL_PWM_Handler();
}
void LTBL_SIGNAL_DSHOT_HANDLER()
{
	LTBL_DSHOT_Handler();
}
#endif
