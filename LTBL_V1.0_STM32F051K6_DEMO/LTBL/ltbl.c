
#include "ltbl.h"

/**
  * 换相后立即激发的事件
  */
void (*LTBL_CommEvent)() = 0;

#define ltblGetComp				LTBL_GetCompStatus
#define ltblSetComp(ch)		((COMP->CSR = 0x810001 | (ch << 4) | (ch << 20)),(LTBL_ADC->CHSELR = ch))

static const uint32_t ltblRccPeriphals[] = 
{
	LTBL_RCCPER_HMOS_U,
	LTBL_RCCPER_HMOS_V,
	LTBL_RCCPER_HMOS_W,
	LTBL_RCCPER_LMOS_U,
	LTBL_RCCPER_LMOS_V,
	LTBL_RCCPER_LMOS_W,
	LTBL_RCCPER_DET_U,
	LTBL_RCCPER_DET_V,
	LTBL_RCCPER_DET_W,
	LTBL_RCCPER_DET_MID,
	LTBL_ADC_RCCPER,
	LTBL_TIM_RCCPER,
	LTBL_REF_TIM_RCCPER
};
static const GPIO_TypeDef* ltblGPIOx[] =
{
	LTBL_GPIO_HMOS_U,
	LTBL_GPIO_HMOS_V,
	LTBL_GPIO_HMOS_W,
	LTBL_GPIO_LMOS_U,
	LTBL_GPIO_LMOS_V,
	LTBL_GPIO_LMOS_W,
	LTBL_GPIO_DET_U,
	LTBL_GPIO_DET_V,
	LTBL_GPIO_DET_W,
	LTBL_GPIO_DET_MID
};
static const uint32_t ltblPin[] = 
{
	LTBL_PIN_HMOS_U,
	LTBL_PIN_HMOS_V,
	LTBL_PIN_HMOS_W,
	LTBL_PIN_LMOS_U,
	LTBL_PIN_LMOS_V,
	LTBL_PIN_LMOS_W,
	LTBL_PIN_DET_U,
	LTBL_PIN_DET_V,
	LTBL_PIN_DET_W,
	LTBL_PIN_DET_MID
};

/**
  * 最近一个更新的油门值
  */
static uint16_t ltblLastThrottle = 0;
/**
  * 当前步
  * 		U		V		W
  * 0		+		-		.
  * 1		+		.		-
  * 2		.		+		-
  * 3		-		+		.
  * 4		-		.		+
  * 5		.		-		+
  */
static uint16_t ltblStep= 0;

/**
  * 参考 定时器地址
  */
const uint32_t *ltblRefCntAddr = (uint32_t *)(LTBL_REF_TIM);
/**
  * 当前比较器地址
  */
const uint32_t *ltblCompAddr = (uint32_t *)&(COMP->CSR);
/**
  * 当前比较器输出位掩码
  */
const uint32_t ltblCompMask = COMP_CSR_COMP2OUT;
/**
  * 过零滤波器值
  */
volatile uint32_t ltblFilterVal = 0;
/**
  * 用于输出 MOSFET 控制信号的引脚的 PinSource 枚举
  * 函数 LTBL_Init 将填充该值
  */
static uint32_t ltblPinSource[6];

/**
  * 每步的 CCER 寄存器配置
  * 函数 ltblGetStepFloatCCER 可以生成该项的值
  */
static uint16_t ltblStepFloatCCER[STEP_NULL + 1];
/**
  * 最近 6 步换相所用的时间间隔，单位 基准定时器 Ticks
  */
volatile uint32_t stepTicks[STEP];
/**
  * 最近 1 步换相所用的时间间隔
  */
volatile uint32_t lastTicks = 0;
/**
  * 最近 6 步换相所用的平均时间间隔，单位 基准定时器 Ticks
  */
volatile uint32_t avgStepTicks = 0;
/**
  * 最近稳定换相次数
  */
volatile uint32_t stabilityStep = 0;



/**
  * @brief  使能对应通道的 输出捕获功能
  * @param  ch: 指定使能的通道，取值 1 - 4
  * @param  config: 指定初始化配置
  * @retval None
  */
static void ltblEnablePWM(uint32_t ch, TIM_OCInitTypeDef *config)
{
	if(ch == 1)
	{
		TIM_OC1Init(LTBL_TIM, config);
		TIM_OC1PreloadConfig(LTBL_TIM, TIM_OCPreload_Disable);
		LTBL_TIM->CCR1 = 0;
	}
	else if(ch == 2)
	{
		TIM_OC2Init(LTBL_TIM, config);
		TIM_OC2PreloadConfig(LTBL_TIM, TIM_OCPreload_Disable);
		LTBL_TIM->CCR2 = 0;
	}
	else if(ch == 3)
	{
		TIM_OC3Init(LTBL_TIM, config);
		TIM_OC3PreloadConfig(LTBL_TIM, TIM_OCPreload_Disable);
		LTBL_TIM->CCR3 = 0;
	}
	else if(ch == 4)
	{
		TIM_OC4Init(LTBL_TIM, config);
		TIM_OC4PreloadConfig(LTBL_TIM, TIM_OCPreload_Disable);
		LTBL_TIM->CCR4 = 0;
	}
}
/**
  * @brief  生成 CCER 寄存器的值
  * @param  pwmMode: 指定 PWM 方式，取值自 LTBL_PWM_Modes 枚举
  * @retval None
  */
static void ltblGetStepFloatCCER(uint8_t pwmMode)
{
	#define ltblGetStepFloatCCERMast_Stop \
	(TIM_OutputState_Disable|TIM_OutputNState_Disable|TIM_OCPolarity_Low|TIM_OCNPolarity_Low)
	
	#define ltblGetStepFloatCCERMast_Single \
	(TIM_OutputState_Enable|TIM_OutputNState_Disable|TIM_OCPolarity_Low|TIM_OCNPolarity_Low)
	
	#define ltblGetStepFloatCCERMast_Damped \
	(TIM_OutputState_Enable|TIM_OutputNState_Enable|TIM_OCPolarity_Low|TIM_OCNPolarity_Low)
	
	if(pwmMode == LTBL_PWM_MODE_SINGLE)
	{
		ltblStepFloatCCER[STEP_0] =  (ltblGetStepFloatCCERMast_Single << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Stop		<< ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_1] =  (ltblGetStepFloatCCERMast_Single << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_2] =  (ltblGetStepFloatCCERMast_Stop   << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Single << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_3] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Single << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_4] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Single << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_5] =  (ltblGetStepFloatCCERMast_Stop   << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Single << ((LTBL_PWM_CH_W - 1) << 2));
	}
	else if(pwmMode == LTBL_PWM_MODE_DAMPED)
	{
		ltblStepFloatCCER[STEP_0] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Stop		<< ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_1] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_2] =  (ltblGetStepFloatCCERMast_Stop   << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_3] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_4] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblStepFloatCCER[STEP_5] =  (ltblGetStepFloatCCERMast_Stop   << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
	}
	ltblStepFloatCCER[STEP_NULL] = (ltblGetStepFloatCCERMast_Stop << ((LTBL_PWM_CH_U - 1) << 2)) | 
																 (ltblGetStepFloatCCERMast_Stop << ((LTBL_PWM_CH_V - 1) << 2)) |
																 (ltblGetStepFloatCCERMast_Stop << ((LTBL_PWM_CH_W - 1) << 2));
}
/**
  * @brief  将用于输出 MOSFET 控制信号的引脚配置为通用推挽输出模式
  * 				以便于 beep 功能和其他特殊控制功能运行
  * @param  None
  * @retval None
  */
static void ltblPinToPP()
{
	int i;
	for(i = 0; i < STEP; i++)
	{
		GPIO_InitTypeDef ioConfig;
		GPIO_ResetBits((GPIO_TypeDef *)ltblGPIOx[i], ltblPin[i]);
		ioConfig.GPIO_Mode = GPIO_Mode_OUT;
		ioConfig.GPIO_OType = GPIO_OType_PP;
		ioConfig.GPIO_Pin = ltblPin[i];
		ioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;
		ioConfig.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init((GPIO_TypeDef *)ltblGPIOx[i], &ioConfig);
	}
}
/**
  * @brief  将用于输出 MOSFET 控制信号的引脚配置为复用推挽输出模式
  * 				以便于 PWM 输出
  * @param  None
  * @retval None
  */
static void ltblPinToAF()
{
	int i;
	for(i = 0; i < STEP; i++)
	{
		GPIO_InitTypeDef ioConfig;
		GPIO_ResetBits((GPIO_TypeDef *)ltblGPIOx[i], ltblPin[i]);
		ioConfig.GPIO_Mode = GPIO_Mode_AF;
		ioConfig.GPIO_OType = GPIO_OType_PP;
		ioConfig.GPIO_Pin = ltblPin[i];
		ioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;
		ioConfig.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init((GPIO_TypeDef *)ltblGPIOx[i], &ioConfig);
		GPIO_PinAFConfig((GPIO_TypeDef *)ltblGPIOx[i], ltblPinSource[i], LTBL_AF_CH_UVW);
	}
}
/**
  * @brief  初始化运行所需的变量，并配置和启动相应外设、配置 IO 输出模式等
  * @param  None
  * @retval None
  */
void LTBL_Init()
{
	int i = 0;
	/* enable periphal clocks */
	{
		for(i = 0; i < STEP + PHASE; i++)
		{
			RCC_AHBPeriphClockCmd(ltblRccPeriphals[i], ENABLE);
		}
		if(LTBL_ADC == ADC1)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//		else if(LTBL_ADC == ADC2)
//			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
		if(LTBL_TIM == TIM1)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		if(LTBL_TIM == TIM2)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		if(LTBL_TIM == TIM3)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
		if(LTBL_REF_TIM == TIM1)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		if(LTBL_REF_TIM == TIM2)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		if(LTBL_REF_TIM == TIM3)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	}
	/* config output IO */
	for(i = 0; i < STEP; i++)
	{
		uint8_t pinSource = (uint8_t)-1;
		uint32_t pin = ltblPin[i];
		while(pin) { pinSource ++; pin >>= 1; }
		ltblPinSource[i] = pinSource;
		GPIO_PinAFConfig((GPIO_TypeDef *)ltblGPIOx[i], pinSource, LTBL_AF_CH_UVW);
	}
	/* config output IO as PP mode */
	ltblPinToAF();
	for(i = STEP; i < sizeof(ltblGPIOx) / sizeof(GPIO_TypeDef *); i++)
	{
		GPIO_InitTypeDef ioConfig;
		GPIO_ResetBits((GPIO_TypeDef *)ltblGPIOx[i], ltblPin[i]);
		ioConfig.GPIO_Mode = GPIO_Mode_IN;
		ioConfig.GPIO_OType = GPIO_OType_PP;
		ioConfig.GPIO_Pin = ltblPin[i];
		ioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;
		ioConfig.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init((GPIO_TypeDef *)ltblGPIOx[i], &ioConfig);
	}
	/* Enable ADC*/
	{
		ADC_InitTypeDef adConfig;
		adConfig.ADC_ContinuousConvMode = DISABLE;
		adConfig.ADC_DataAlign = ADC_DataAlign_Right;
		adConfig.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T15_TRGO;
		adConfig.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		adConfig.ADC_Resolution = ADC_Resolution_12b;
		adConfig.ADC_ScanDirection = ADC_ScanDirection_Upward;
		ADC_Cmd(LTBL_ADC, DISABLE);
		ADC_Init(LTBL_ADC, &adConfig);
		ADC_Cmd(LTBL_ADC, ENABLE);
		LTBL_ADC->CR |= ADC_CR_ADCAL;
		while(LTBL_ADC->CR & ADC_CR_ADCAL);
		ADC_Cmd(LTBL_ADC, ENABLE);
	}
	/* Enable TIM */
	{
		/* timer counter */
		TIM_TimeBaseInitTypeDef timConfig;
		TIM_OCInitTypeDef pwmConfig;
		TIM_TimeBaseStructInit(&timConfig);
		TIM_OCStructInit(&pwmConfig);
		timConfig.TIM_ClockDivision = TIM_CKD_DIV1;
		timConfig.TIM_CounterMode = TIM_CounterMode_Up;
		timConfig.TIM_Period = LTBL_PWM_RESOLUTION - 1;
		timConfig.TIM_Prescaler = LTBL_PWM_PRESCALER;
		TIM_TimeBaseInit(LTBL_TIM, &timConfig);
		/* output compare */
		pwmConfig.TIM_OCMode = TIM_OCMode_PWM2;
		pwmConfig.TIM_OutputState = TIM_OutputState_Enable;
		pwmConfig.TIM_OutputNState = TIM_OutputNState_Enable;
		pwmConfig.TIM_OCPolarity = TIM_OCPolarity_Low;
		pwmConfig.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		ltblEnablePWM(LTBL_PWM_CH_U, &pwmConfig);
		ltblEnablePWM(LTBL_PWM_CH_V, &pwmConfig);
		ltblEnablePWM(LTBL_PWM_CH_W, &pwmConfig);
		/* dead time */
		LTBL_TIM->BDTR = (LTBL_TIM->BDTR & ~TIM_BDTR_DTG) | LTBL_PWM_DEADTIME;
		/* enable counter */
		TIM_Cmd(LTBL_TIM, ENABLE);
		TIM_CtrlPWMOutputs(LTBL_TIM, ENABLE);
	}
	/* Enable Ref timer */
	{
		TIM_TimeBaseInitTypeDef timConfig;
		TIM_OCInitTypeDef pwmConfig;
		TIM_TimeBaseStructInit(&timConfig);
		TIM_OCStructInit(&pwmConfig);
		timConfig.TIM_ClockDivision = TIM_CKD_DIV1;
		timConfig.TIM_CounterMode = TIM_CounterMode_Up;
		timConfig.TIM_Period = 0xffff;
		timConfig.TIM_Prescaler = LTBL_REF_TIM_PRESC;
		TIM_TimeBaseInit(LTBL_REF_TIM, &timConfig);
		TIM_Cmd(LTBL_REF_TIM, ENABLE);
	}
	/* Comparetor */
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
		COMP->CSR |= 0x810001 /*0x813001*/;
	}
	ltblGetStepFloatCCER(LTBL_Default_PWM_Mode);
}
/**
  * @brief  更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void LTBL_UpdateThrottle(uint16_t thr)
{
	uint8_t stepTyp = ltblStep >> 1;
	if(thr > LTBL_PWM_RESOLUTION)
	{
		thr = LTBL_PWM_RESOLUTION;
	}
	ltblLastThrottle = thr;
	if(stepTyp == 0) { LTBL_PWMCCR_U = thr; }
	else if(stepTyp == 1) { LTBL_PWMCCR_V = thr; }
	else if(stepTyp == 2){ LTBL_PWMCCR_W = thr; }
}
/**
  * @brief  为换相后事件安装指定的事件处理器
  * @param  commEvent: 指定的事件处理器
  * @retval None
  */
void LTBL_AttachCommEvent(void(*commEvent)())
{
	LTBL_CommEvent = commEvent;
}
/**
  * @brief  获取稳定换相次数
  * @param  None
  * @retval 稳定换相次数
  */
uint32_t LTBL_GetStabilityStep()
{
	return stabilityStep;
}
/**
  * @brief  获取平均换相间隔
  * @param  None
  * @retval 平均换相间隔 单位 基准定时器 Ticks
  */
uint32_t LTBL_GetAvgCommInterval()
{
	return avgStepTicks;
}
/**
  * @brief  函数 ltblStepXNormal 将换相到对应步 X
  * @param  None
  * @retval None
  */
void ltblStep0Normal()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_0];
	LTBL_PWMCCR_U = ltblLastThrottle;
	LTBL_PWMCCR_V = 0;
	ltblSetComp(LTBL_COMP_CH_W);
	ltblStep = STEP_0;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep1Normal()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_1];
	LTBL_PWMCCR_U = ltblLastThrottle;
	LTBL_PWMCCR_W = 0;
	ltblSetComp(LTBL_COMP_CH_V);
	ltblStep = STEP_1;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep2Normal()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_2];
	LTBL_PWMCCR_V = ltblLastThrottle;
	LTBL_PWMCCR_W = 0;
	ltblSetComp(LTBL_COMP_CH_U);
	ltblStep = STEP_2;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep3Normal()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_3];
	LTBL_PWMCCR_V = ltblLastThrottle;
	LTBL_PWMCCR_U = 0;
	ltblSetComp(LTBL_COMP_CH_W);
	ltblStep = STEP_3;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep4Normal()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_4];
	LTBL_PWMCCR_W = ltblLastThrottle;
	LTBL_PWMCCR_U = 0;
	ltblSetComp(LTBL_COMP_CH_V);
	ltblStep = STEP_4;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep5Normal()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_5];
	LTBL_PWMCCR_W = ltblLastThrottle;
	LTBL_PWMCCR_V = 0;
	ltblSetComp(LTBL_COMP_CH_U);
	ltblStep = STEP_5;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}

#define LTBL_ASM_OFFSET_REF_TIM_EGR		0x14
#define LTBL_ASM_OFFSET_REF_TIM_CNT		0x24
/**
  * @brief  等待比较器为 H
  * @param  timeout: 允许等待过零的最大定时器 Ticks
  * @param  verTimeout: 允许等待消磁的最大定时器 Ticks
  * @retval 等待所用的定时器 Ticks
  */
__asm uint32_t ltblWaitH(uint32_t timeout, uint32_t verTimeout, uint32_t verMin)
{
	extern ltblRefCntAddr			/* 基准定时器 */
	extern ltblCompAddr				/* 比较器地址 */
	extern ltblCompMask				/* 比较器掩码 */
	extern ltblFilterVal			/* 滤波器值 */
		
	; r0 = timeout
	; r1 = verTimeout
	; r2 = LTBL_COMP_Mask
	; r3 = ?
	; r4 = ver
	; r5 = pas
	; r6 = LTBL_REF_TIM
	; r7 = LTBL_COMP
	
	push {r4-r7}
	push {r2}
	
	ldr  r2, =ltblCompMask
	ldr  r2, [r2]
	ldr  r7, =ltblCompAddr
	ldr  r7, [r7]
	
	movs r4, #0x01
	ldr  r6, =ltblRefCntAddr										; r6 : LTBL_REF_TIM
	ldr  r6, [r6]										
	str  r4, [r6,#LTBL_ASM_OFFSET_REF_TIM_EGR]	; LTBL_REF_TIM(r6)->EGR = 0x01
	movs r4, #0x00															; r4 : ver = 0
	pop  {r5}
	
ltblWaitH_ver_loop
	ldr  r3, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	cmp  r3, r1																	; if(nowTicks > verTimeout)
	bgt  ltblWaitH_ver_finished									; 
	cmp  r4, #LTBL_MAGFILTER_VAL								;	if(ver > val)										
	bgt  ltblWaitH_ver_finished
	ldr  r3, [r7]
	ands r3, r2																	; r3 = COMP->CSR & MASK
	cmp  r3, #0x00
	beq  ltblWaitH_ver_getH
	
ltblWaitH_ver_getL
	adds r4, #LTBL_MAGFILTER_INC
	b    ltblWaitH_ver_loop
	
ltblWaitH_ver_getH
	cmp  r4, #LTBL_MAGFILTER_DEC
	bgt  ltblWaitH_ver_getH_red
	movs r4, #0x00
	b    ltblWaitH_ver_loop
ltblWaitH_ver_getH_red
	subs r4, #LTBL_MAGFILTER_DEC
	b    ltblWaitH_ver_loop
ltblWaitH_ver_finished
	ldr  r3, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	cmp  r3, r5																	; if(nowTicks > verMin)
	bgt  ltblWaitH_pas_begin
	b    ltblWaitH_ver_finished
		
	; r0 = timeout
	; r1 = verTimeout
	; r2 = LTBL_COMP_Mask
	; r3 = ?
	; r4 = targetPas
	; r5 = pas
	; r6 = LTBL_REF_TIM
	; r7 = LTBL_COMP

ltblWaitH_pas_begin
	movs r5, #0x00
	ldr  r4, =ltblFilterVal
	ldr  r4, [r4]

ltblWaitH_pas_loop	
	ldr  r3, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	cmp  r3, r0																	; if(nowTicks > timeout)
	bgt  ltblWaitH_finished											; 
	cmp  r5, r4																	;	if(ver > val)										
	bgt  ltblWaitH_finished
	ldr  r3, [r7]
	ands r3, r2																	; r3 = COMP->CSR & MASK
	cmp  r3, #0x00
	beq  ltblWaitH_pas_getH
	
ltblWaitH_pas_getL
	cmp  r5, #LTBL_ZEROFILTER_DEC
	bgt  ltblWaitH_pas_getL_red
	movs r5, #0x00
	b    ltblWaitH_pas_loop
ltblWaitH_pas_getL_red
	subs r5, #LTBL_ZEROFILTER_DEC
	b    ltblWaitH_pas_loop

ltblWaitH_pas_getH
	adds r5, #LTBL_ZEROFILTER_INC
	b    ltblWaitH_pas_loop

ltblWaitH_finished
	ldr  r0, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	pop  {r4-r7}
	bx   lr
}

__asm uint32_t ltblWaitL(uint32_t timeout, uint32_t verTimeout, uint32_t verMin)
{
	extern ltblRefCntAddr			/* 基准定时器 */
	extern ltblCompAddr				/* 比较器地址 */
	extern ltblCompMask				/* 比较器掩码 */
	extern ltblFilterVal			/* 滤波器值 */
		
	; r0 = timeout
	; r1 = verTimeout
	; r2 = LTBL_COMP_Mask
	; r3 = ?
	; r4 = ver
	; r5 = pas
	; r6 = LTBL_REF_TIM
	; r7 = LTBL_COMP
	
	push {r4-r7}
	push {r2}
	
	ldr  r2, =ltblCompMask
	ldr  r2, [r2]
	ldr  r7, =ltblCompAddr
	ldr  r7, [r7]
	
	movs r4, #0x01
	ldr  r6, =ltblRefCntAddr										; r6 : LTBL_REF_TIM
	ldr  r6, [r6]										
	str  r4, [r6,#LTBL_ASM_OFFSET_REF_TIM_EGR]	; LTBL_REF_TIM(r6)->EGR = 0x01
	movs r4, #0x00															; r4 : ver = 0
	pop  {r5}															; r5 : pas = 0
	
ltblWaitL_ver_loop
	ldr  r3, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	cmp  r3, r1																	; if(nowTicks > verTimeout)
	bgt  ltblWaitL_ver_finished									; 
	cmp  r4, #LTBL_MAGFILTER_VAL								;	if(ver > val)										
	bgt  ltblWaitL_ver_finished
	ldr  r3, [r7]
	ands r3, r2																	; r3 = COMP->CSR & MASK
	cmp  r3, #0x00
	bne  ltblWaitL_ver_getH
	
ltblWaitL_ver_getL
	adds r4, #LTBL_MAGFILTER_INC
	b    ltblWaitL_ver_loop
	
ltblWaitL_ver_getH
	cmp  r4, #LTBL_MAGFILTER_DEC
	bgt  ltblWaitL_ver_getH_red
	movs r4, #0x00
	b    ltblWaitL_ver_loop
ltblWaitL_ver_getH_red
	subs r4, #LTBL_MAGFILTER_DEC
	b    ltblWaitL_ver_loop
ltblWaitL_ver_finished
	ldr  r3, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	cmp  r3, r5																	; if(nowTicks > verMin)
	bgt  ltblWaitL_pas_begin
	b    ltblWaitL_ver_finished
	
		
	; r0 = timeout
	; r1 = verTimeout
	; r2 = LTBL_COMP_Mask
	; r3 = ?
	; r4 = targetPas
	; r5 = pas
	; r6 = LTBL_REF_TIM
	; r7 = LTBL_COMP

ltblWaitL_pas_begin
	movs r5, #0x00
	ldr  r4, =ltblFilterVal
	ldr  r4, [r4]

ltblWaitL_pas_loop	
	ldr  r3, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	cmp  r3, r0																	; if(nowTicks > timeout)
	bgt  ltblWaitL_finished											; 
	cmp  r5, r4																	;	if(ver > val)										
	bgt  ltblWaitL_finished
	ldr  r3, [r7]
	ands r3, r2																	; r3 = COMP->CSR & MASK
	cmp  r3, #0x00
	bne  ltblWaitL_pas_getH
	
ltblWaitL_pas_getL
	cmp  r5, #LTBL_ZEROFILTER_DEC
	bgt  ltblWaitL_pas_getL_red
	movs r5, #0x00
	b    ltblWaitL_pas_loop
ltblWaitL_pas_getL_red
	subs r5, #LTBL_ZEROFILTER_DEC
	b    ltblWaitL_pas_loop

ltblWaitL_pas_getH
	adds r5, #LTBL_ZEROFILTER_INC
	b    ltblWaitL_pas_loop

ltblWaitL_finished
	ldr  r0, [r6,#LTBL_ASM_OFFSET_REF_TIM_CNT]	; r3 = LTBL_REF_TIM->CNT;
	pop  {r4-r7}
	bx   lr
}
/**
  * @brief  使电机以指定的 频率、响度 鸣叫指定的 毫秒数
  * @param  freq: 指定的频率。当频率不在预设的范围内时，电机不会鸣叫
  * @param  duration: 指定的持续时间，单位 毫秒
  * @param  volume: 指定的响度。实际响度不会超过预设的最大响度。当响度为 0 时，电机不会鸣叫
  *                 响度范围：0（最小）- 100（最大）
  * @retval None
  */
void LTBL_Tone(uint32_t freq, uint32_t duration, uint32_t volume)
{
	uint32_t fkhz = 0;
	uint32_t freqCycleUS = 0;
	uint32_t usPerTick = 0;
	uint32_t tickPerCycle = 0;
	uint32_t posTick = 0;
	uint32_t negTick = 0;
	uint32_t totalCycles = 0;
	volatile uint32_t nowTick = 0;
	volatile uint32_t lastTick = 0;
	volatile uint32_t diffTick = 0;
	SystemCoreClockUpdate();
	fkhz = SystemCoreClock / 1e3;
	LTBL_DISABLEIT;
	if(freq > LTBL_TONE_FREQ_MAX && freq < LTBL_TONE_FREQ_MIN)
	{
		freq = 1000;
		volume = 0;
	}
	if(volume > LTBL_TONE_VOLUME_MAX)
	{
		volume = LTBL_TONE_VOLUME_MAX;
	}
	freqCycleUS = 1e6 / freq;
	totalCycles = duration * 1e3 / freqCycleUS / 2;
	usPerTick = 1e3 / (fkhz / (LTBL_REF_TIM_PRESC + 1));
	tickPerCycle = freqCycleUS / usPerTick;
	posTick = tickPerCycle * volume / 2 / 100;
	negTick = tickPerCycle - posTick;
	LTBL_RESET_HMOS_U;
	LTBL_RESET_HMOS_V;
	LTBL_RESET_HMOS_W;
	LTBL_RESET_LMOS_U;
	LTBL_RESET_LMOS_V;
	LTBL_RESET_LMOS_W;
	ltblPinToPP();
	if(volume <= 100)
	{
		ltblLastThrottle = (volume * LTBL_PWM_RESOLUTION / 100);
	}
	else
	{
		ltblLastThrottle = 0;
	}
	
	#define ltblWaitPos     nowTick = LTBL_REF_TIM->CNT; \
	                        if(nowTick >= lastTick) { diffTick = nowTick - lastTick; } \
													else { diffTick = nowTick + 0x10000 - lastTick; } \
													if(diffTick >= posTick) { lastTick = nowTick; break; }
	#define ltblWaitNeg     nowTick = LTBL_REF_TIM->CNT; \
	                        if(nowTick >= lastTick) { diffTick = nowTick - lastTick; } \
													else { diffTick = nowTick + 0x10000 - lastTick; } \
													if(diffTick >= negTick) { lastTick = nowTick; break; }
	lastTick = LTBL_REF_TIM->CNT;
	LTBL_SET_LMOS_V;
	while(totalCycles --)
	{
		if(posTick)
		{
			LTBL_SET_HMOS_U;
			LTBL_RESET_HMOS_W;
		}
		while(1) { ltblWaitPos; }
		LTBL_RESET_HMOS_U;
		LTBL_RESET_HMOS_W;
		while(1) { ltblWaitNeg; }
		if(posTick)
		{
			LTBL_SET_HMOS_W;
			LTBL_RESET_HMOS_U;
		}
		while(1) { ltblWaitPos; }
		LTBL_RESET_HMOS_U;
		LTBL_RESET_HMOS_W;
		while(1) { ltblWaitNeg; }
	}
	LTBL_RESET_HMOS_U;
	LTBL_RESET_HMOS_W;
	LTBL_RESET_LMOS_V;
	LTBL_UpdateThrottle(0);
	LTBL_ENABLEIT;
}
/**
  * @brief  使电机以正常 6 步方波运行。执行此函数前，请保证以正常执行了函数 LTBL_Init()
  *         执行此函数前，请保证以正常执行了函数 LTBL_Init()
  * @param  None
  * @retval None
  */
void LTBL_Run()
{
	uint32_t i = 0;
	uint32_t verTimeout = 0;
	uint32_t verMin = 0;
	uint32_t commTimeout = LTBL_START_TICK_MAX;
	for(; i < sizeof(stepTicks) / sizeof(uint32_t); i++)
	{
		stepTicks[i] = LTBL_START_TICK_MAX;
	}
	#define CalcFilterVal	\
						avgStepTicks = (stepTicks[0] + stepTicks[1] + stepTicks[2] + stepTicks[3] + stepTicks[4] + stepTicks[5]) / 6;\
						/* filter val will be 500 when comm cycle is 2.5ms */ \
						ltblFilterVal = (avgStepTicks << 1) + 1;\
						ltblFilterVal = ltblFilterVal > LTBL_ZEROFILTER_MAX ? LTBL_ZEROFILTER_MAX : ltblFilterVal;\
						if(lastTicks >= commTimeout) { stabilityStep = 0; } else\
						{\
							if(stabilityStep < LTBL_STAB_STEP_MAX)\
							{\
								stabilityStep ++;\
							}\
						}\
						if(stabilityStep >= LTBL_STAB_STEP && avgStepTicks >= LTBL_LOW_SPEED_TICK)\
						{ verMin = 0; }\
						else { verMin = lastTicks >> 2; }\
						verTimeout = avgStepTicks * LTBL_MAGFILTER_ENABLE << 1;\
						verTimeout = verTimeout > LTBL_MAG_TICK_MAX ? LTBL_MAG_TICK_MAX : verTimeout;
	
	ltblPinToAF();
	while(1)
	{
		ltblStep0Normal();
		CalcFilterVal;
		lastTicks = stepTicks[0] = ltblWaitL(commTimeout, verTimeout, verMin);
		
		ltblStep1Normal();
		CalcFilterVal;
		lastTicks = stepTicks[1] = ltblWaitH(commTimeout, verTimeout, verMin);
		
		ltblStep2Normal();
		CalcFilterVal;
		lastTicks = stepTicks[2] = ltblWaitL(commTimeout, verTimeout, verMin);
		
		ltblStep3Normal();
		CalcFilterVal;
		lastTicks = stepTicks[3] = ltblWaitH(commTimeout, verTimeout, verMin);
		
		ltblStep4Normal();
		CalcFilterVal;
		lastTicks = stepTicks[4] = ltblWaitL(commTimeout, verTimeout, verMin);
		
		ltblStep5Normal();
		CalcFilterVal;
		lastTicks = stepTicks[5] = ltblWaitH(commTimeout, verTimeout, verMin);
	}
}

