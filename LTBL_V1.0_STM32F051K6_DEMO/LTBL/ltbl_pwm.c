
#include "stm32f0xx.h"
#include "ltbl_pwm.h"

/**
  * 表示 PWM 是否处于就绪态
  */
static volatile uint8_t ltblPWMInit = 0;
static void (*LTBL_PWM_Captured)(int32_t throttle) = 0;

/**
* @brief  初始化运行所需的变量，配置 IO、定时器输入捕获等，以便使用 PWM 油门信号解码器
  * @param  None
  * @retval None
  */
void LTBL_PWM_Init()
{
	uint32_t fmhz = 0;
	uint32_t pin = LTBL_SIGNAL_PWM_PIN;
	uint8_t pinSource = (uint8_t)-1;
	
	GPIO_InitTypeDef ioConfig;
	TIM_TimeBaseInitTypeDef timConfig;
	TIM_ICInitTypeDef icConfig;
	
	ltblPWMInit = YES;
	
	SystemCoreClockUpdate();
	fmhz = SystemCoreClock / 1e6;
	
	RCC_AHBPeriphClockCmd(LTBL_SIGNAL_PWM_GPIO_RCCPER, ENABLE);
	
	if(LTBL_SIGNAL_PWM_TIM == TIM1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	if(LTBL_SIGNAL_PWM_TIM == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	if(LTBL_SIGNAL_PWM_TIM == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	ioConfig.GPIO_Mode = GPIO_Mode_AF;
	ioConfig.GPIO_OType = GPIO_OType_PP;
	ioConfig.GPIO_Pin = LTBL_SIGNAL_PWM_PIN;
	ioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;
	ioConfig.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LTBL_SIGNAL_PWM_GPIO, &ioConfig);
	while(pin) { pinSource ++; pin >>= 1; }
	GPIO_PinAFConfig(LTBL_SIGNAL_PWM_GPIO, pinSource, LTBL_SIGNAL_PWM_AF_CH);
	
	TIM_TimeBaseStructInit(&timConfig);
	timConfig.TIM_ClockDivision = TIM_CKD_DIV1;
	timConfig.TIM_CounterMode = TIM_CounterMode_Up;
	timConfig.TIM_Period = 0xffff;
	timConfig.TIM_Prescaler = fmhz - 1;
	TIM_TimeBaseInit(LTBL_SIGNAL_PWM_TIM, &timConfig);
	
	TIM_ICStructInit(&icConfig);
	icConfig.TIM_Channel = LTBL_SIGNAL_PWM_TIM_Channel;
	icConfig.TIM_ICFilter = 0x0f;
	icConfig.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	icConfig.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(LTBL_SIGNAL_PWM_TIM, &icConfig);
	
	TIM_ITConfig(LTBL_SIGNAL_PWM_TIM, LTBL_SIGNAL_PWM_TIM_IT, ENABLE);
	TIM_ClearITPendingBit(LTBL_SIGNAL_PWM_TIM, LTBL_SIGNAL_PWM_TIM_IT);
	NVIC_EnableIRQ(LTBL_SIGNAL_PWM_IRQn);
	
	TIM_Cmd(LTBL_SIGNAL_PWM_TIM, ENABLE);
}
/**
  * @brief  释放 PWM 解析所占用的所有外设资源
  * @param  None
  * @retval None
  */
void LTBL_PWM_Dispose()
{
	ltblPWMInit = NO;
	TIM_Cmd(LTBL_SIGNAL_PWM_TIM, DISABLE);
	NVIC_DisableIRQ(LTBL_SIGNAL_PWM_IRQn);
}
/**
  * @brief  为输入捕获后事件安装指定的事件处理器
	* @param  cap : 指定的事件处理器
  * @retval None
  */
void LTBL_PWM_AttachCaptureEvent(void(*cap)(int32_t))
{
	LTBL_PWM_Captured = cap;
}

/**
  * 由指定脉宽值计算得的油门值
  */
static volatile int32_t ltblThrottle = 0;
/**
  * 最近一次捕获的脉宽，单位 us
  */
static volatile int32_t ltblPWMWidth = 0;
/**
  * 最近一次输入捕获触发时定时器的值
  */
static volatile uint32_t ltblPWMLastVal = (uint32_t)-1;
#if( LTBL_SIGNAL_USE_PWM == YES )
void LTBL_PWM_Handler()
{
	static uint8_t captureStatus = 0;
	uint32_t captureVal = LTBL_SIGNAL_PWM_TIMCCR;
	TIM_ClearITPendingBit(LTBL_SIGNAL_PWM_TIM, LTBL_SIGNAL_PWM_TIM_IT);
	if(!ltblPWMInit) { return; }
	if(ltblPWMLastVal == (uint32_t)-1)
	{
		ltblPWMLastVal = captureVal;
	}
	if(LTBL_SIGNAL_PWM_GPIO->IDR & LTBL_SIGNAL_PWM_PIN)
	{
		ltblPWMLastVal = captureVal;
		captureStatus = 1;
	}
	else
	{
		/* had capture high level */
		if(captureStatus)
		{
			captureStatus = 0;
			if(captureVal >= ltblPWMLastVal)
			{
				ltblPWMWidth = captureVal - ltblPWMLastVal;
			}
			else
			{
				ltblPWMWidth = captureVal + 65536 - ltblPWMLastVal;
			}
		}
		else
		{
			/* cannot capture high level, input pulse is too narrow ! */
			ltblPWMWidth = 0;
		}
		//ltblPWMWidth = 1573;
		if(ltblPWMWidth < LTBL_SIGNAL_PWM_MIN - LTBL_SIGNAL_PWM_VALID_RANGE)
		{
			#if LTBL_SIGNAL_PWM_SetZeroWhenOutOfRange == YES
			ltblThrottle = 0;
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
			#endif
		}
		else if(ltblPWMWidth < LTBL_SIGNAL_PWM_MIN)
		{
			ltblThrottle = -LTBL_SIGNAL_PWM_RESOLUTION;
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
		}
		else if(ltblPWMWidth < LTBL_SIGNAL_PWM_MID - LTBL_SIGNAL_PWM_DEAD_RANGE)
		{
			ltblThrottle = -LTBL_SIGNAL_PWM_RESOLUTION + LTBL_SIGNAL_PWM_RESOLUTION * 
					(ltblPWMWidth - LTBL_SIGNAL_PWM_MIN) / 
					(LTBL_SIGNAL_PWM_MID - LTBL_SIGNAL_PWM_DEAD_RANGE - LTBL_SIGNAL_PWM_MIN);
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
		}
		else if(ltblPWMWidth < LTBL_SIGNAL_PWM_MID + LTBL_SIGNAL_PWM_DEAD_RANGE)
		{
			ltblThrottle = 0;
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
		}
		else if(ltblPWMWidth < LTBL_SIGNAL_PWM_MAX)
		{
			ltblThrottle = LTBL_SIGNAL_PWM_RESOLUTION * 
					(ltblPWMWidth - (LTBL_SIGNAL_PWM_MID + LTBL_SIGNAL_PWM_DEAD_RANGE)) / 
					(LTBL_SIGNAL_PWM_MAX - (LTBL_SIGNAL_PWM_MID + LTBL_SIGNAL_PWM_DEAD_RANGE));
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
		}
		else if(ltblPWMWidth <= LTBL_SIGNAL_PWM_MAX + LTBL_SIGNAL_PWM_VALID_RANGE)
		{
			ltblThrottle = LTBL_SIGNAL_PWM_RESOLUTION;
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
		}
		else
		{
			#if LTBL_SIGNAL_PWM_SetZeroWhenOutOfRange == YES
			ltblThrottle = 0;
			if(LTBL_PWM_Captured)
			{
				LTBL_PWM_Captured(ltblThrottle);
			}
			#endif
		}
	}
}
#endif
