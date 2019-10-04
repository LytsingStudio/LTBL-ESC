
#ifndef __LTBL_PWM_H__
#define __LTBL_PWM_H__

#include "stdint.h"

#define YES	1
#define NO	0

#define LTBL_SIGNAL_USE_PWM			YES

#define LTBL_SIGNAL_PWM_GPIO_RCCPER		RCC_AHBPeriph_GPIOA
#define LTBL_SIGNAL_PWM_GPIO					GPIOA
#define LTBL_SIGNAL_PWM_PIN						GPIO_Pin_15
#define LTBL_SIGNAL_PWM_AF_CH					GPIO_AF_2

#define LTBL_SIGNAL_PWM_TIM_ID			2
#define LTBL_SIGNAL_PWM_TIM_CH			1
#define LTBL_SIGNAL_PWM_TIM_IT			(TIM_IT_Update << LTBL_SIGNAL_PWM_TIM_CH)

#if( LTBL_SIGNAL_PWM_TIM_CH == 1 )
#define LTBL_SIGNAL_PWM_TIMCCR		LTBL_SIGNAL_PWM_TIM->CCR1
#elif( LTBL_SIGNAL_PWM_TIM_CH == 2 )
#define LTBL_SIGNAL_PWM_TIMCCR		LTBL_SIGNAL_PWM_TIM->CCR2
#elif( LTBL_SIGNAL_PWM_TIM_CH == 3 )
#define LTBL_SIGNAL_PWM_TIMCCR		LTBL_SIGNAL_PWM_TIM->CCR3
#elif( LTBL_SIGNAL_PWM_TIM_CH == 4 )
#define LTBL_SIGNAL_PWM_TIMCCR		LTBL_SIGNAL_PWM_TIM->CCR4
#endif

#if( LTBL_SIGNAL_PWM_TIM_ID == 1 )
#define LTBL_SIGNAL_PWM_TIM				TIM1
#define LTBL_SIGNAL_PWM_IRQn			TIM1_CC_IRQn
#define LTBL_SIGNAL_PWM_HANDLER		TIM1_CC_IRQHandler
#elif( LTBL_SIGNAL_PWM_TIM_ID == 2 )
#define LTBL_SIGNAL_PWM_TIM				TIM2
#define LTBL_SIGNAL_PWM_IRQn			TIM2_IRQn
#define LTBL_SIGNAL_PWM_HANDLER		TIM2_IRQHandler
#elif( LTBL_SIGNAL_PWM_TIM_ID == 3 )
#define LTBL_SIGNAL_PWM_TIM				TIM3
#define LTBL_SIGNAL_PWM_IRQn			TIM3_IRQn
#define LTBL_SIGNAL_PWM_HANDLER		TIM3_IRQHandler
#endif

#if( LTBL_SIGNAL_PWM_TIM_CH ==  1)
#define LTBL_SIGNAL_PWM_TIM_Channel		TIM_Channel_1
#elif( LTBL_SIGNAL_PWM_TIM_CH ==  2)
#define LTBL_SIGNAL_PWM_TIM_Channel		TIM_Channel_2
#elif( LTBL_SIGNAL_PWM_TIM_CH ==  3)
#define LTBL_SIGNAL_PWM_TIM_Channel		TIM_Channel_3
#elif( LTBL_SIGNAL_PWM_TIM_CH ==  4)
#define LTBL_SIGNAL_PWM_TIM_Channel		TIM_Channel_4
#endif

#define LTBL_SIGNAL_PWM_MIN							1000
#define LTBL_SIGNAL_PWM_MID							1500
#define LTBL_SIGNAL_PWM_MAX							2000
#define LTBL_SIGNAL_PWM_DEAD_RANGE			50
#define LTBL_SIGNAL_PWM_VALID_RANGE			500

/* DON'T set this item to YES, or serious problems may arise ! */
#define LTBL_SIGNAL_PWM_SetZeroWhenOutOfRange 	NO

#define LTBL_SIGNAL_PWM_RESOLUTION			1024

#ifdef __cplusplus
extern "C"
{
#endif

void LTBL_PWM_Init(void);
void LTBL_PWM_Dispose(void);
void LTBL_PWM_AttachCaptureEvent(void(*cap)(int32_t));
void LTBL_PWM_Handler(void);

#ifdef __cplusplus
}
#endif

#endif

