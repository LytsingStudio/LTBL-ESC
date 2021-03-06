
#ifndef __LTBL_DSHOT_H__
#define __LTBL_DSHOT_H__

/* User Support */
#include "arduino.h"

#include "stdint.h"

/* Hardware DSHOT Decode */

/* Periphal Occupied : 
 * 1 Timer
 * 3 Timer I/O Channel
 */
 
typedef void(*LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef)(int32_t, uint8_t *);

#define YES	1
#define NO	0

#define LTBL_SIGNAL_USE_DSHOT					YES

#define LTBL_SIGNAL_USE_DSHOT150			NO
#define LTBL_SIGNAL_USE_DSHOT300			NO
#define LTBL_SIGNAL_USE_DSHOT600			YES
#define LTBL_SIGNAL_USE_DSHOT1200			NO
#define LTBL_SIGNAL_USE_DSHOT_EX			NO

#if( LTBL_SIGNAL_USE_DSHOT150 + LTBL_SIGNAL_USE_DSHOT300 + LTBL_SIGNAL_USE_DSHOT600 + LTBL_SIGNAL_USE_DSHOT1200 + LTBL_SIGNAL_USE_DSHOT_EX > 1 )
#error Only one protocol can be selected.
#endif

#define LTBL_SIGNAL_DSHOT_GPIO_RES_RCCPER			RCC_AHBPeriph_GPIOA
#define LTBL_SIGNAL_DSHOT_GPIO_CAP_RCCPER			RCC_AHBPeriph_GPIOB
#define LTBL_SIGNAL_DSHOT_GPIO_RES						GPIOA
#define LTBL_SIGNAL_DSHOT_GPIO_CAP						GPIOB
#define LTBL_SIGNAL_DSHOT_PIN_RES							GPIO_Pin_15
#define LTBL_SIGNAL_DSHOT_PIN_CAP							GPIO_Pin_3

#define LTBL_SIGNAL_DSHOT_AF_CH								GPIO_AF_2

#define LTBL_SIGNAL_DSHOT_TIM_ID					2
#define LTBL_SIGNAL_DSHOT_TIM_RES_CH			2
#define LTBL_SIGNAL_DSHOT_TIM_CAP_CH			1
#define LTBL_SIGNAL_DSHOT_TIM_CUT_CH			3
#define LTBL_SIGNAL_DSHOT_TIM_IT			(TIM_IT_Update << LTBL_SIGNAL_DSHOT_TIM_CH)

#if( LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 1 )
#define LTBL_SIGNAL_DSHOT_TIM_CAP_CCR		LTBL_SIGNAL_DSHOT_TIM->CCR1
#elif( LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 2 )
#define LTBL_SIGNAL_DSHOT_TIM_CAP_CCR		LTBL_SIGNAL_DSHOT_TIM->CCR2
#elif( LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 3 )
#define LTBL_SIGNAL_DSHOT_TIM_CAP_CCR		LTBL_SIGNAL_DSHOT_TIM->CCR3
#elif( LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 4 )
#define LTBL_SIGNAL_DSHOT_TIM_CAP_CCR		LTBL_SIGNAL_DSHOT_TIM->CCR4
#endif

#if( LTBL_SIGNAL_DSHOT_TIM_ID == 1 )
#define LTBL_SIGNAL_DSHOT_TIM				TIM1
#define LTBL_SIGNAL_DSHOT_IRQn			TIM1_CC_IRQn
#define LTBL_SIGNAL_DSHOT_HANDLER		TIM1_CC_IRQHandler
#elif( LTBL_SIGNAL_DSHOT_TIM_ID == 2 )
#define LTBL_SIGNAL_DSHOT_TIM				TIM2
#define LTBL_SIGNAL_DSHOT_IRQn			TIM2_IRQn
#define LTBL_SIGNAL_DSHOT_HANDLER		TIM2_IRQHandler
#elif( LTBL_SIGNAL_DSHOT_TIM_ID == 3 )
#define LTBL_SIGNAL_DSHOT_TIM				TIM3
#define LTBL_SIGNAL_DSHOT_IRQn			TIM3_IRQn
#define LTBL_SIGNAL_DSHOT_HANDLER		TIM3_IRQHandler
#endif

#if( LTBL_SIGNAL_DSHOT_TIM_RES_CH ==  LTBL_SIGNAL_DSHOT_TIM_CAP_CH)
#error Reset channel cannot be same as capture channel.
#endif

#if( LTBL_SIGNAL_DSHOT_TIM_RES_CH ==  1)
#define LTBL_SIGNAL_DSHOT_TIM_RES_Channel		TIM_Channel_1
#define LTBL_SIGNAL_DSHOT_TIM_RES_TIXFPX		TIM_TS_TI1FP1
#elif( LTBL_SIGNAL_DSHOT_TIM_RES_CH ==  2)
#define LTBL_SIGNAL_DSHOT_TIM_RES_Channel		TIM_Channel_2
#define LTBL_SIGNAL_DSHOT_TIM_RES_TIXFPX		TIM_TS_TI2FP2
#elif( LTBL_SIGNAL_DSHOT_TIM_RES_CH ==  3)
#define LTBL_SIGNAL_DSHOT_TIM_RES_Channel		TIM_Channel_3
#error Channel 3 cannot be config as input of slave mode, Please choose other channel.
#elif( LTBL_SIGNAL_DSHOT_TIM_RES_CH ==  4)
#define LTBL_SIGNAL_DSHOT_TIM_RES_Channel		TIM_Channel_4
#error Channel 3 cannot be config as input of slave mode, Please choose other channel.
#endif

#if( LTBL_SIGNAL_DSHOT_TIM_CAP_CH ==  1)
#define LTBL_SIGNAL_DSHOT_TIM_CAP_Channel		TIM_Channel_1
#define LTBL_SIGNAL_DSHOT_TIMCCR							LTBL_SIGNAL_DSHOT_TIM->CCR1
#elif( LTBL_SIGNAL_DSHOT_TIM_CAP_CH ==  2)
#define LTBL_SIGNAL_DSHOT_TIM_CAP_Channel		TIM_Channel_2
#define LTBL_SIGNAL_DSHOT_TIMCCR							LTBL_SIGNAL_DSHOT_TIM->CCR2
#elif( LTBL_SIGNAL_DSHOT_TIM_CAP_CH ==  3)
#define LTBL_SIGNAL_DSHOT_TIM_CAP_Channel		TIM_Channel_3
#define LTBL_SIGNAL_DSHOT_TIMCCR							LTBL_SIGNAL_DSHOT_TIM->CCR3
#elif( LTBL_SIGNAL_DSHOT_TIM_CAP_CH ==  4)
#define LTBL_SIGNAL_DSHOT_TIM_CAP_Channel		TIM_Channel_4
#define LTBL_SIGNAL_DSHOT_TIMCCR							LTBL_SIGNAL_DSHOT_TIM->CCR4
#endif

#if( LTBL_SIGNAL_DSHOT_TIM_CUT_CH ==  1)
#define LTBL_SIGNAL_DSHOT_TIM_CUT_Channel		TIM_Channel_1
#define LTBL_SIGNAL_DSHOT_TIM_CUT_IT				TIM_IT_CC1
#elif( LTBL_SIGNAL_DSHOT_TIM_CUT_CH ==  2)
#define LTBL_SIGNAL_DSHOT_TIM_CUT_Channel		TIM_Channel_2
#define LTBL_SIGNAL_DSHOT_TIM_CUT_IT				TIM_IT_CC2
#elif( LTBL_SIGNAL_DSHOT_TIM_CUT_CH ==  3)
#define LTBL_SIGNAL_DSHOT_TIM_CUT_Channel		TIM_Channel_3
#define LTBL_SIGNAL_DSHOT_TIM_CUT_IT				TIM_IT_CC3
#elif( LTBL_SIGNAL_DSHOT_TIM_CUT_CH ==  4)
#define LTBL_SIGNAL_DSHOT_TIM_CUT_Channel		TIM_Channel_4
#define LTBL_SIGNAL_DSHOT_TIM_CUT_IT				TIM_IT_CC4
#endif

/* DON'T set this item to YES, or serious problems may arise ! */
#define LTBL_SIGNAL_DSHOT_SetZeroWhenOutOfRange 	NO

#if( LTBL_SIGNAL_USE_DSHOT600 == YES )
#define LTBL_SIGNAL_DSHOT_PULSE_MIN				300
#define LTBL_SIGNAL_DSHOT_PULSE_MAX				1600
#define LTBL_SIGNAL_DSHOT_FrameBitCount		16
#define LTBL_SIGNAL_DSHOT_BufferLength		64
#define LTBL_SIGNAL_DSHOT_ThresholdNS			800
#define LTBL_SIGNAL_DSHOT_FrameTimeoutNS	2000
#define LTBL_SIGNAL_DSHOT_TYPE_FUNC		LTBL_SIGNAL_DSHOT_StructType1_ASM
#endif

#define LTBL_SIGNAL_DSHOT_RESOLUTION_P		10

#ifdef __cplusplus
extern "C"
{
#endif

void LTBL_DSHOT_Init(void);
void LTBL_DSHOT_Dispose(void);
void LTBL_DSHOT_Handler(void);
void LTBL_DSHOT_AttachCaptureEvent(LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef cap);
LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef LTBL_DSHOT_GetCaptureEventHandler(void);
	
#ifdef __cplusplus
}
#endif

#endif
