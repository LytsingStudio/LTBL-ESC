
#ifndef __LTBL_SIGNAL_H__
#define __LTBL_SIGNAL_H__

/* User support */
#include "delay.h"


#include "stdint.h"
#include "ltbl_pwm.h"
#include "ltbl_dshot.h"

typedef enum
{
	LTBL_SIGNAL_TYPE_Unknown,
	LTBL_SIGNAL_TYPE_PWM,
	LTBL_SIGNAL_TYPE_DSHOT
} LTBL_SIGNAL_TYPES;

typedef enum
{
	LTBL_SIGNAL_STATE_RESET,
	LTBL_SIGNAL_STATE_LOSE,
	LTBL_SIGNAL_STATE_NOTZERO
} LTBL_SIGNAL_STATES;

#define LTBL_SIGNAL_SYSTEM_MS					System_ms

#define LTBL_SIGNAL_DETECT_TIMEOUT		3000
#define LTBL_SIGNAL_DETECT_STABTIME		500
#define LTBL_SIGNAL_DETECT_LOSETIME		100

#ifdef __cplusplus
extern "C"
{
#endif
	
LTBL_SIGNAL_TYPES LTBL_Signal_GetSignalType(void);
uint32_t LTBL_SIGNAL_GetThrottleState(void);

#ifdef __cplusplus
}
#endif

#endif


