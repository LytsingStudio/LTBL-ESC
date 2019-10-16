
#ifndef __LTBL_H__
#define __LTBL_H__

/* User Support */
#include "stdio.h"
#include "stdarg.h"
#include "arduino.h"
#include "delay.h"
#include "math.h"

/* Periphal Device Driver */
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_dma.h"
#include "stm32f0xx_adc.h"

typedef enum
{
	LTBL_PWM_MODE_SINGLE,
	LTBL_PWM_MODE_DAMPED
} LTBL_PWM_Modes;

typedef enum
{
	LTBL_Mode_Free,					/* Free MOSFETs and do nothing */
	LTBL_Mode_Normal,				/* Motor rotate forward */
	LTBL_Mode_Brake,				/* Motor brake */
	LTBL_Mode_Reverse,			/* Motor rotate reversely */
	LTBL_Mode_Startup1,			/* Startup Mode 1 */
	LTBL_Mode_NULL					/* DON NOT USE */
} LTBL_Modes_TypeDef;

typedef void (*LTBL_OperationStrategy_TypeDef)();
typedef void (*LTBL_ThrottleStrategy_TypeDef)(uint16_t);
typedef void(*LTBL_CommEventHandler_TypeDef)(); 

#define YES	1
#define NO	0

/* Enable parameter check will decrease running efficiency
 * [!] if you can ensure parameter is ok, set this macro to NO
 */
#define LTBL_ParameterCheck_Enable		NO
#define LTBL_Default_PWM_Mode					LTBL_PWM_MODE_SINGLE

/* step count & phase count */
#define STEP									6
typedef enum
{
	STEP_0,
	STEP_1,
	STEP_2,
	STEP_3,
	STEP_4,
	STEP_5,
	STEP_NULL
} LTBL_Steps_TypeDef;
#define PHASE									3
typedef enum
{
	PHASE_U,
	PHASE_V,
	PHASE_W
} LTBL_Phases_TypeDef;
#define PHASE_FREE						3

/* Output Pins Definition */
#define LTBL_RCCPER_HMOS_U		(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_HMOS_V		(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_HMOS_W		(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_LMOS_U		(RCC_AHBPeriph_GPIOB)
#define LTBL_RCCPER_LMOS_V		(RCC_AHBPeriph_GPIOB)
#define LTBL_RCCPER_LMOS_W		(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_DET_U			(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_DET_V			(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_DET_W			(RCC_AHBPeriph_GPIOA)
#define LTBL_RCCPER_DET_MID		(RCC_AHBPeriph_GPIOA)

#define LTBL_GPIO_HMOS_U		(GPIOA)
#define LTBL_GPIO_HMOS_V		(GPIOA)
#define LTBL_GPIO_HMOS_W		(GPIOA)
#define LTBL_PIN_HMOS_U			(GPIO_Pin_10)
#define LTBL_PIN_HMOS_V			(GPIO_Pin_9)
#define LTBL_PIN_HMOS_W			(GPIO_Pin_8)

#define LTBL_GPIO_LMOS_U		(GPIOB)
#define LTBL_GPIO_LMOS_V		(GPIOB)
#define LTBL_GPIO_LMOS_W		(GPIOA)
#define LTBL_PIN_LMOS_U			(GPIO_Pin_1)
#define LTBL_PIN_LMOS_V			(GPIO_Pin_0)
#define LTBL_PIN_LMOS_W			(GPIO_Pin_7)

#define LTBL_GPIO_DET_U			(GPIOA)
#define LTBL_GPIO_DET_V			(GPIOA)
#define LTBL_GPIO_DET_W			(GPIOA)
#define LTBL_GPIO_DET_MID		(GPIOA)
#define LTBL_PIN_DET_U			(GPIO_Pin_2)
#define LTBL_PIN_DET_V			(GPIO_Pin_4)
#define LTBL_PIN_DET_W			(GPIO_Pin_5)
#define LTBL_PIN_DET_MID		(GPIO_Pin_1)

#define LTBL_AF_CH_UVW			GPIO_AF_2

#define LTBL_COMP_CH_U			0x06
#define LTBL_COMP_CH_V			0x04
#define LTBL_COMP_CH_W			0x05

#define LTBL_PWM_CH_U				3
#define LTBL_PWM_CH_V				2
#define LTBL_PWM_CH_W				1

#define LTBL_PWMCCR_U				(LTBL_TIM->CCR3)
#define LTBL_PWMCCR_V				(LTBL_TIM->CCR2)
#define LTBL_PWMCCR_W				(LTBL_TIM->CCR1)

#define LTBL_PWM_RESOLUTION		1024
#define LTBL_PWM_PRESCALER		1
#define LTBL_PWM_DEADTIME			50

#define LTBL_ADC_RCCPER			(RCC_APB2Periph_ADC1)
#define LTBL_ADC						(ADC1)
#define LTBL_ADC_CH_U				(ADC_Channel_2)
#define LTBL_ADC_CH_V				(ADC_Channel_4)
#define LTBL_ADC_CH_W				(ADC_Channel_5)

#define LTBL_TIM_RCCPER			(RCC_APB2Periph_TIM1)
#define LTBL_REF_TIM_RCCPER	(RCC_APB1Periph_TIM3)
#define LTBL_TIM						(TIM1)
#define LTBL_REF_TIM				(TIM3)

#define LTBL_REF_TIM_PRESC	479

#define LTBL_STAB_STEP_MAX	360
#define LTBL_STAB_STEP			36
#define LTBL_STAB_TICK_MAX	(LTBL_START_TICK_MAX>>1)

/* macros */
#define LTBL_RESET_HMOS_U		(LTBL_GPIO_HMOS_U->ODR &= (uint16_t)~(LTBL_PIN_HMOS_U))
#define LTBL_RESET_HMOS_V		(LTBL_GPIO_HMOS_V->ODR &= (uint16_t)~(LTBL_PIN_HMOS_V))
#define LTBL_RESET_HMOS_W		(LTBL_GPIO_HMOS_W->ODR &= (uint16_t)~(LTBL_PIN_HMOS_W))
#define LTBL_RESET_LMOS_U		(LTBL_GPIO_LMOS_U->ODR &= (uint16_t)~(LTBL_PIN_LMOS_U))
#define LTBL_RESET_LMOS_V		(LTBL_GPIO_LMOS_V->ODR &= (uint16_t)~(LTBL_PIN_LMOS_V))
#define LTBL_RESET_LMOS_W		(LTBL_GPIO_LMOS_W->ODR &= (uint16_t)~(LTBL_PIN_LMOS_W))
#define LTBL_RESET_ALL			LTBL_RESET_HMOS_U;\
														LTBL_RESET_HMOS_V;\
														LTBL_RESET_HMOS_W;\
														LTBL_RESET_LMOS_U;\
														LTBL_RESET_LMOS_V;\
														LTBL_RESET_LMOS_W;
#define LTBL_RESET_LMOS_ALL	LTBL_RESET_LMOS_U;\
														LTBL_RESET_LMOS_V;\
														LTBL_RESET_LMOS_W;
#define LTBL_RESET_HMOS_ALL	LTBL_RESET_HMOS_U;\
														LTBL_RESET_HMOS_V;\
														LTBL_RESET_HMOS_W;

#define LTBL_SET_HMOS_U			(LTBL_GPIO_HMOS_U->ODR |= (uint16_t)(LTBL_PIN_HMOS_U))
#define LTBL_SET_HMOS_V			(LTBL_GPIO_HMOS_V->ODR |= (uint16_t)(LTBL_PIN_HMOS_V))
#define LTBL_SET_HMOS_W			(LTBL_GPIO_HMOS_W->ODR |= (uint16_t)(LTBL_PIN_HMOS_W))
#define LTBL_SET_LMOS_U			(LTBL_GPIO_LMOS_U->ODR |= (uint16_t)(LTBL_PIN_LMOS_U))
#define LTBL_SET_LMOS_V			(LTBL_GPIO_LMOS_V->ODR |= (uint16_t)(LTBL_PIN_LMOS_V))
#define LTBL_SET_LMOS_W			(LTBL_GPIO_LMOS_W->ODR |= (uint16_t)(LTBL_PIN_LMOS_W))
#define LTBL_SET_LMOS_ALL		LTBL_SET_LMOS_U;\
														LTBL_SET_LMOS_V;\
														LTBL_SET_LMOS_W;

#define LTBL_GetCompStatus()	(!(COMP->CSR & COMP_CSR_COMP2OUT))

#define LTBL_ENABLE_COMP_OUTPUT
#define LTBL_COMP_OUTPUT_HIGH		
#define LTBL_COMP_OUTPUT_LOW		

#define LTBL_SYSTEM_MS			System_ms

#define LTBL_DISABLEIT			__set_PRIMASK(1)
#define LTBL_ENABLEIT				__set_PRIMASK(0);//LTBL_TIM->EGR = 1;

#define LTBL_TONE_FREQ_MIN				100
#define LTBL_TONE_FREQ_MAX				8000
#define LTBL_TONE_VOLUME_MIN			0
#define LTBL_TONE_VOLUME_MAX			50
#define LTBL_TONE_VOLUME_RECOMM		10

#define LTBL_DEMAGWAIT_ENABLE			YES
#define LTBL_MAGFILTER_ENABLE			YES

#define LTBL_ZEROFILTER_INC		1
#define LTBL_ZEROFILTER_DEC		1
#define LTBL_ZEROFILTER_MAX		500
#define LTBL_MAGFILTER_INC		1
#define LTBL_MAGFILTER_DEC		1
#define LTBL_MAGFILTER_VAL		0

#define LTBL_LOW_SPEED_TICK		200
#define LTBL_HIGH_SPEED_TICK	3
#define LTBL_START_TICK_MAX		2000
#define LTBL_MAG_TICK_MAX			3000
#define LTBL_START1_TICK_MAX	60000

#ifdef __cplusplus
extern "C"
{
#endif

void LTBL_Init(void);
void LTBL_Run(void);
void LTBL_Stop(void);
void LTBL_SetMode(LTBL_Modes_TypeDef mode);
void LTBL_SetPWMMode(LTBL_PWM_Modes mode);
void LTBL_UpdateThrottle(uint16_t thr);
void LTBL_AttachCommEvent(LTBL_CommEventHandler_TypeDef commEvent);
LTBL_CommEventHandler_TypeDef LTBL_GetCommEventHandler(void);
uint32_t LTBL_GetStabilityStep(void);
uint32_t LTBL_GetAvgCommInterval(void);
void LTBL_Tone(uint32_t freq, uint32_t duration, uint32_t volume);

/* INNER FUNC USED FOR DEBUG */
void ltblStep0Normal(void);
void ltblStep1Normal(void);
void ltblStep2Normal(void);
void ltblStep3Normal(void);
void ltblStep4Normal(void);
void ltblStep5Normal(void);

#ifdef __cplusplus
}
#endif

#endif
