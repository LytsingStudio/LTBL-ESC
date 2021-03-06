
#include "ltbl.h"

/**
  * 换相后立即激发的事件
  */
void (*LTBL_CommEvent)() = 0;
/**
  * LTBL 停止标志
  */
uint8_t LTBL_StopFlag = NO;
/**
  * 当前运行模式
  */
static LTBL_Modes_TypeDef ltblCurrentMode = LTBL_Mode_Normal;

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
  * Brake Mode 下的 CCER 寄存器配置
  * 函数 ltblGetBrakeModeCCER 可以生成该项的值
  */
static uint16_t ltblBrakeModeStepCCER[STEP_NULL + 1];
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
  * 运行策略集
  */
volatile LTBL_OperationStrategy_TypeDef ltblOperationStrategy_step0[LTBL_Mode_NULL];
volatile LTBL_OperationStrategy_TypeDef ltblOperationStrategy_step1[LTBL_Mode_NULL];
volatile LTBL_OperationStrategy_TypeDef ltblOperationStrategy_step2[LTBL_Mode_NULL];
volatile LTBL_OperationStrategy_TypeDef ltblOperationStrategy_step3[LTBL_Mode_NULL];
volatile LTBL_OperationStrategy_TypeDef ltblOperationStrategy_step4[LTBL_Mode_NULL];
volatile LTBL_OperationStrategy_TypeDef ltblOperationStrategy_step5[LTBL_Mode_NULL];
volatile LTBL_ThrottleStrategy_TypeDef ltblUpdateThrottles[LTBL_Mode_NULL];
/**
  * 运行策略函数声明
  */
void ltblNormalModeStep0(void);
void ltblNormalModeStep1(void);
void ltblNormalModeStep2(void);
void ltblNormalModeStep3(void);
void ltblNormalModeStep4(void);
void ltblNormalModeStep5(void);
void ltblStartup1ModeStep0(void);
void ltblStartup1ModeStep1(void);
void ltblStartup1ModeStep2(void);
void ltblStartup1ModeStep3(void);
void ltblStartup1ModeStep4(void);
void ltblStartup1ModeStep5(void);
void ltblBrakeModeStep0(void);
void ltblBrakeModeStep1(void);
void ltblBrakeModeStep2(void);
void ltblBrakeModeStep3(void);
void ltblBrakeModeStep4(void);
void ltblBrakeModeStep5(void);
void ltblReverseModeStep0(void);
void ltblReverseModeStep1(void);
void ltblReverseModeStep2(void);
void ltblReverseModeStep3(void);
void ltblReverseModeStep4(void);
void ltblReverseModeStep5(void);
void ltblFreeModeStep0(void);
void ltblFreeModeStep1(void);
void ltblFreeModeStep2(void);
void ltblFreeModeStep3(void);
void ltblFreeModeStep4(void);
void ltblFreeModeStep5(void);
void ltblUpdateThrottleNormal(uint16_t thr);
void ltblUpdateThrottleBrake(uint16_t thr);
void ltblUpdateThrottleReverse(uint16_t thr);
void ltblUpdateThrottleFree(uint16_t thr);
void ltblUpdateThrottleStartup1(uint16_t thr);

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
  * @brief  生成 Brake Mode 模式下的 CCER 寄存器的值
  * @param  None
  * @retval None
  */
static void ltblGetBrakeModeCCER()
{
	#define ltblGetStepFloatCCERMast_Stop \
	(TIM_OutputState_Disable|TIM_OutputNState_Disable|TIM_OCPolarity_Low|TIM_OCNPolarity_Low)
	
	#define ltblGetStepFloatCCERMast_Brake \
	(TIM_OutputState_Disable|TIM_OutputNState_Enable|TIM_OCPolarity_Low|TIM_OCNPolarity_Low)
	
	#define ltblGetStepFloatCCERMast_Damped \
	(TIM_OutputState_Enable|TIM_OutputNState_Enable|TIM_OCPolarity_Low|TIM_OCNPolarity_Low)
	
		ltblBrakeModeStepCCER[STEP_0] = (ltblGetStepFloatCCERMast_Brake << ((LTBL_PWM_CH_U - 1) << 2)) | 
																		(ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																		(ltblGetStepFloatCCERMast_Stop		<< ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblBrakeModeStepCCER[STEP_1] = (ltblGetStepFloatCCERMast_Brake << ((LTBL_PWM_CH_U - 1) << 2)) | 
																		(ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_V - 1) << 2)) |
																		(ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblBrakeModeStepCCER[STEP_2] = (ltblGetStepFloatCCERMast_Stop   << ((LTBL_PWM_CH_U - 1) << 2)) | 
																		(ltblGetStepFloatCCERMast_Brake << ((LTBL_PWM_CH_V - 1) << 2)) |
																		(ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblBrakeModeStepCCER[STEP_3] = (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																		(ltblGetStepFloatCCERMast_Brake << ((LTBL_PWM_CH_V - 1) << 2)) |
																		(ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblBrakeModeStepCCER[STEP_4] =  (ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_U - 1) << 2)) | 
																		 (ltblGetStepFloatCCERMast_Stop 	<< ((LTBL_PWM_CH_V - 1) << 2)) |
																		 (ltblGetStepFloatCCERMast_Brake << ((LTBL_PWM_CH_W - 1) << 2));
		
		ltblBrakeModeStepCCER[STEP_5] = (ltblGetStepFloatCCERMast_Stop   << ((LTBL_PWM_CH_U - 1) << 2)) | 
																		(ltblGetStepFloatCCERMast_Damped << ((LTBL_PWM_CH_V - 1) << 2)) |
																		(ltblGetStepFloatCCERMast_Brake << ((LTBL_PWM_CH_W - 1) << 2));
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
  * @brief  安装运行策略
  * @param  None
  * @retval None
  */
static void ltblInstallOperationStrategy()
{
	/* normal mode */
	ltblOperationStrategy_step0[LTBL_Mode_Normal] = ltblNormalModeStep0;
	ltblOperationStrategy_step1[LTBL_Mode_Normal] = ltblNormalModeStep1;
	ltblOperationStrategy_step2[LTBL_Mode_Normal] = ltblNormalModeStep2;
	ltblOperationStrategy_step3[LTBL_Mode_Normal] = ltblNormalModeStep3;
	ltblOperationStrategy_step4[LTBL_Mode_Normal] = ltblNormalModeStep4;
	ltblOperationStrategy_step5[LTBL_Mode_Normal] = ltblNormalModeStep5;
	ltblOperationStrategy_step0[LTBL_Mode_Brake] = ltblBrakeModeStep0;
	ltblOperationStrategy_step1[LTBL_Mode_Brake] = ltblBrakeModeStep1;
	ltblOperationStrategy_step2[LTBL_Mode_Brake] = ltblBrakeModeStep2;
	ltblOperationStrategy_step3[LTBL_Mode_Brake] = ltblBrakeModeStep3;
	ltblOperationStrategy_step4[LTBL_Mode_Brake] = ltblBrakeModeStep4;
	ltblOperationStrategy_step5[LTBL_Mode_Brake] = ltblBrakeModeStep5;
	ltblOperationStrategy_step0[LTBL_Mode_Reverse] = ltblReverseModeStep0;
	ltblOperationStrategy_step1[LTBL_Mode_Reverse] = ltblReverseModeStep1;
	ltblOperationStrategy_step2[LTBL_Mode_Reverse] = ltblReverseModeStep2;
	ltblOperationStrategy_step3[LTBL_Mode_Reverse] = ltblReverseModeStep3;
	ltblOperationStrategy_step4[LTBL_Mode_Reverse] = ltblReverseModeStep4;
	ltblOperationStrategy_step5[LTBL_Mode_Reverse] = ltblReverseModeStep5;
	ltblOperationStrategy_step0[LTBL_Mode_Free] = ltblFreeModeStep0;
	ltblOperationStrategy_step1[LTBL_Mode_Free] = ltblFreeModeStep1;
	ltblOperationStrategy_step2[LTBL_Mode_Free] = ltblFreeModeStep2;
	ltblOperationStrategy_step3[LTBL_Mode_Free] = ltblFreeModeStep3;
	ltblOperationStrategy_step4[LTBL_Mode_Free] = ltblFreeModeStep4;
	ltblOperationStrategy_step5[LTBL_Mode_Free] = ltblFreeModeStep5;
	ltblOperationStrategy_step0[LTBL_Mode_Startup1] = ltblStartup1ModeStep0;
	ltblOperationStrategy_step1[LTBL_Mode_Startup1] = ltblStartup1ModeStep1;
	ltblOperationStrategy_step2[LTBL_Mode_Startup1] = ltblStartup1ModeStep2;
	ltblOperationStrategy_step3[LTBL_Mode_Startup1] = ltblStartup1ModeStep3;
	ltblOperationStrategy_step4[LTBL_Mode_Startup1] = ltblStartup1ModeStep4;
	ltblOperationStrategy_step5[LTBL_Mode_Startup1] = ltblStartup1ModeStep5;
	
	ltblUpdateThrottles[LTBL_Mode_Normal] = ltblUpdateThrottleNormal;
	ltblUpdateThrottles[LTBL_Mode_Brake] = ltblUpdateThrottleBrake;
	ltblUpdateThrottles[LTBL_Mode_Reverse] = ltblUpdateThrottleReverse;
	ltblUpdateThrottles[LTBL_Mode_Free] = ltblUpdateThrottleFree;
	ltblUpdateThrottles[LTBL_Mode_Startup1] = ltblUpdateThrottleStartup1;
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
	ltblGetBrakeModeCCER();
	ltblGetStepFloatCCER(LTBL_Default_PWM_Mode);
	ltblInstallOperationStrategy();
}
/**
  * @brief  以 Normal Mode 更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void ltblUpdateThrottleNormal(uint16_t thr)
{
	uint8_t stepTyp = ltblStep >> 1;
	#if(LTBL_ParameterCheck_Enable == YES)
	if(thr > LTBL_PWM_RESOLUTION)
	{
		thr = LTBL_PWM_RESOLUTION;
		ltblLastThrottle = thr;
	}
	#else
	ltblLastThrottle = thr;
	#endif
	if(stepTyp == 0) { LTBL_PWMCCR_U = thr; }
	else if(stepTyp == 1) { LTBL_PWMCCR_V = thr; }
	else if(stepTyp == 2){ LTBL_PWMCCR_W = thr; }
}
/**
  * @brief  以 Brake Mode 更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void ltblUpdateThrottleBrake(uint16_t thr)
{
	#if(LTBL_ParameterCheck_Enable == YES)
	if(thr > LTBL_PWM_RESOLUTION)
	{
		thr = LTBL_PWM_RESOLUTION;
		ltblLastThrottle = thr;
	}
	#else
	ltblLastThrottle = thr;
	#endif
}
/**
  * @brief  以 Free Mode 更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void ltblUpdateThrottleFree(uint16_t thr)
{
	#if(LTBL_ParameterCheck_Enable == YES)
	if(thr > LTBL_PWM_RESOLUTION)
	{
		thr = LTBL_PWM_RESOLUTION;
		ltblLastThrottle = thr;
	}
	#else
	ltblLastThrottle = thr;
	#endif
}
/**
  * @brief  以 Reverse Mode 更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void ltblUpdateThrottleReverse(uint16_t thr)
{
	uint8_t stepTyp = ltblStep >> 1;
	#if(LTBL_ParameterCheck_Enable == YES)
	if(thr > LTBL_PWM_RESOLUTION)
	{
		thr = LTBL_PWM_RESOLUTION;
		ltblLastThrottle = thr;
	}
	#else
	ltblLastThrottle = thr;
	#endif
	if(stepTyp == 0) { LTBL_PWMCCR_U = thr; }
	else if(stepTyp == 1) { LTBL_PWMCCR_V = thr; }
	else if(stepTyp == 2){ LTBL_PWMCCR_W = thr; }
}
/**
  * @brief  以 Startup1 Mode 更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void ltblUpdateThrottleStartup1(uint16_t thr)
{
	thr = thr * 3;
	if(thr > LTBL_PWM_RESOLUTION)
	{
		thr = LTBL_PWM_RESOLUTION;
		ltblLastThrottle = thr;
	}
	ltblLastThrottle = thr;
}
/**
  * @brief  更新油门值
  * @param  thr: 指定的油门值
  * @retval None
  */
void LTBL_UpdateThrottle(uint16_t thr)
{
	ltblUpdateThrottles[ltblCurrentMode](thr);
}
/**
  * @brief  获取当前已安装的换相事件处理器
  * @param  None
	* @retval LTBL_CommEventHandler_TypeDef : 当前的事件处理器
  */
LTBL_CommEventHandler_TypeDef LTBL_GetCommEventHandler()
{
	return LTBL_CommEvent;
}
/**
  * @brief  为换相后事件安装指定的事件处理器
  * @param  commEvent: 指定的事件处理器
  * @retval None
  */
void LTBL_AttachCommEvent(LTBL_CommEventHandler_TypeDef commEvent)
{
	#if(LTBL_ParameterCheck_Enable == YES)
	if(commEvent)
	{
		LTBL_CommEvent = commEvent;
	}
	#else
	LTBL_CommEvent = commEvent;
	#endif
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
  * @brief  函数 ltblStepXNormal 将以 Normal Mode 换相到对应步 X
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
/**
  * @brief  函数 ltblStepXFree 将以 Free Mode 换相到对应步 X
  * @param  None
  * @retval None
  */
void ltblStep0Free()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_NULL];
	LTBL_PWMCCR_U = 0;
	LTBL_PWMCCR_V = 0;
	ltblSetComp(LTBL_COMP_CH_W);
	ltblStep = STEP_0;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep1Free()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_NULL];
	LTBL_PWMCCR_U = 0;
	LTBL_PWMCCR_W = 0;
	ltblSetComp(LTBL_COMP_CH_V);
	ltblStep = STEP_1;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep2Free()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_NULL];
	LTBL_PWMCCR_V = 0;
	LTBL_PWMCCR_W = 0;
	ltblSetComp(LTBL_COMP_CH_U);
	ltblStep = STEP_2;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep3Free()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_NULL];
	LTBL_PWMCCR_V = 0;
	LTBL_PWMCCR_U = 0;
	ltblSetComp(LTBL_COMP_CH_W);
	ltblStep = STEP_3;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep4Free()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_NULL];
	LTBL_PWMCCR_W = 0;
	LTBL_PWMCCR_U = 0;
	ltblSetComp(LTBL_COMP_CH_V);
	ltblStep = STEP_4;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep5Free()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblStepFloatCCER[STEP_NULL];
	LTBL_PWMCCR_W = 0;
	LTBL_PWMCCR_V = 0;
	ltblSetComp(LTBL_COMP_CH_U);
	ltblStep = STEP_5;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
/**
  * @brief  函数 ltblStepXBrake 将以 Brake Mode 换相到对应步 X
  * @param  None
  * @retval None
  */
void ltblStep0Brake()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblBrakeModeStepCCER[STEP_0];
	LTBL_PWMCCR_U = ltblLastThrottle;
	LTBL_PWMCCR_V = 0;
	ltblSetComp(LTBL_COMP_CH_W);
	ltblStep = STEP_0;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep1Brake()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblBrakeModeStepCCER[STEP_1];
	LTBL_PWMCCR_U = ltblLastThrottle;
	LTBL_PWMCCR_W = 0;
	ltblSetComp(LTBL_COMP_CH_V);
	ltblStep = STEP_1;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep2Brake()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblBrakeModeStepCCER[STEP_2];
	LTBL_PWMCCR_V = ltblLastThrottle;
	LTBL_PWMCCR_W = 0;
	ltblSetComp(LTBL_COMP_CH_U);
	ltblStep = STEP_2;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep3Brake()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblBrakeModeStepCCER[STEP_3];
	LTBL_PWMCCR_V = ltblLastThrottle;
	LTBL_PWMCCR_U = 0;
	ltblSetComp(LTBL_COMP_CH_W);
	ltblStep = STEP_3;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep4Brake()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblBrakeModeStepCCER[STEP_4];
	LTBL_PWMCCR_W = ltblLastThrottle;
	LTBL_PWMCCR_U = 0;
	ltblSetComp(LTBL_COMP_CH_V);
	ltblStep = STEP_4;
	LTBL_ENABLEIT;
	if(LTBL_CommEvent) { LTBL_CommEvent(); }
}
void ltblStep5Brake()
{
	LTBL_DISABLEIT;
	LTBL_TIM->CCER = ltblBrakeModeStepCCER[STEP_5];
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
  * @brief  切换 PWM 模式
  * @param  mode: 指定的 PWM 模式
  * @retval None
  */
void LTBL_SetPWMMode(LTBL_PWM_Modes mode)
{
	#if(LTBL_ParameterCheck_Enable == YES)
	if(mode >= LTBL_PWM_MODE_DAMPED)
	{
		ltblCurrentMode = LTBL_PWM_MODE_DAMPED; 
		ltblGetStepFloatCCER(mode);
	}
	#else
	ltblGetStepFloatCCER(mode);
	#endif
}
/**
  * @brief  立即切换 LTBL 运行策略（无论当前正处于任何模式下）
  * @param  mode: 指定的模式，当传入的模式无效时，不会执行任何操作
  * @retval None
  */
void LTBL_SetMode(LTBL_Modes_TypeDef mode)
{
	#if(LTBL_ParameterCheck_Enable == YES)
	if(mode < LTBL_Mode_NULL)
	{
		ltblCurrentMode = mode; 
	}
	#else
	ltblCurrentMode = mode;
	#endif
}
/**
  * @brief  本次电周期结束后停止工作，释放所有 MOS、复位状态并退出 LTBL_Run() 函数
	* @param  None
  * @retval None
  */
void LTBL_Stop()
{
	LTBL_StopFlag = 1;
}

/* -------------------- Normal Mode -------------------- */
static uint32_t verTimeout = 0;
static uint32_t verMin = 0;
static uint32_t commTimeout = LTBL_START_TICK_MAX;
static uint32_t ltblPhaseMin = 0;
static uint8_t  ltblPhaseMinIndex = 0;
static uint16_t ltblPhaseValue[6] = { 0 };
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
					else { verMin = (lastTicks >> 2) * LTBL_DEMAGWAIT_ENABLE; }\
					if(avgStepTicks <= LTBL_HIGH_SPEED_TICK)\
					{\
						verTimeout = 0;\
					}\
					else \
					{\
						verTimeout = avgStepTicks * LTBL_MAGFILTER_ENABLE << 1;\
						verTimeout = verTimeout > LTBL_MAG_TICK_MAX ? LTBL_MAG_TICK_MAX : verTimeout;\
					}
uint32_t ltblGetStepPhaseValue(uint32_t channel)
{
	LTBL_ADC->SMPR = ADC_SampleTime_1_5Cycles;
  LTBL_ADC->CHSELR = channel;
	ADC_StartOfConversion(LTBL_ADC);
	while(!ADC_GetFlagStatus(LTBL_ADC, ADC_FLAG_EOC));
	return LTBL_ADC->DR;
}
void ltblGetPhaseValue()
{
	#define TestPulseDelayUS	1
	#define TestDemagDelayUS	20
	
	LTBL_RESET_ALL;
	/* step 0 [U:H] [V:L] [W:-] */
	LTBL_SET_LMOS_V;
	LTBL_SET_HMOS_U;
	delayMicroseconds(TestPulseDelayUS);
	ltblPhaseValue[0] = ltblGetStepPhaseValue(LTBL_ADC_CH_W);
	LTBL_RESET_LMOS_V;
	LTBL_RESET_HMOS_U;
	delayMicroseconds(TestDemagDelayUS);
	
	/* step 1 [U:H] [W:L] [V:-] */
	LTBL_SET_LMOS_W;
	LTBL_SET_HMOS_U;
	delayMicroseconds(TestPulseDelayUS);
	ltblPhaseValue[1] = ltblGetStepPhaseValue(LTBL_ADC_CH_V);
	LTBL_RESET_LMOS_W;
	LTBL_RESET_HMOS_U;
	delayMicroseconds(TestDemagDelayUS);
	
	/* step 2 [V:H] [W:L] [U:-] */
	LTBL_SET_LMOS_W;
	LTBL_SET_HMOS_V;
	delayMicroseconds(TestPulseDelayUS);
	ltblPhaseValue[2] = ltblGetStepPhaseValue(LTBL_ADC_CH_U);
	LTBL_RESET_LMOS_W;
	LTBL_RESET_HMOS_V;
	delayMicroseconds(TestDemagDelayUS);
	
	/* step 3 [V:H] [U:L] [W:-] */
	LTBL_SET_LMOS_U;
	LTBL_SET_HMOS_V;
	delayMicroseconds(TestPulseDelayUS);
	ltblPhaseValue[3] = ltblGetStepPhaseValue(LTBL_ADC_CH_W);
	LTBL_RESET_LMOS_U;
	LTBL_RESET_HMOS_V;
	delayMicroseconds(TestDemagDelayUS);
	
	/* step 4 [W:H] [U:L] [V:-] */
	LTBL_SET_LMOS_U;
	LTBL_SET_HMOS_W;
	delayMicroseconds(TestPulseDelayUS);
	ltblPhaseValue[4] = ltblGetStepPhaseValue(LTBL_ADC_CH_V);
	LTBL_RESET_LMOS_U;
	LTBL_RESET_HMOS_W;
	delayMicroseconds(TestDemagDelayUS);
	
	/* step 5 [W:H] [V:L] [U:-] */
	LTBL_SET_LMOS_V;
	LTBL_SET_HMOS_W;
	delayMicroseconds(TestPulseDelayUS);
	ltblPhaseValue[5] = ltblGetStepPhaseValue(LTBL_ADC_CH_U);
	LTBL_RESET_LMOS_V;
	LTBL_RESET_HMOS_W;
	
	ltblPhaseMinIndex = 0;
	ltblPhaseMin = ltblPhaseValue[0];
	if(ltblPhaseValue[1] < ltblPhaseMin) { ltblPhaseMin = ltblPhaseValue[1]; ltblPhaseMinIndex = 1; }
	if(ltblPhaseValue[2] < ltblPhaseMin) { ltblPhaseMin = ltblPhaseValue[2]; ltblPhaseMinIndex = 2; }
	if(ltblPhaseValue[3] < ltblPhaseMin) { ltblPhaseMin = ltblPhaseValue[3]; ltblPhaseMinIndex = 3; }
	if(ltblPhaseValue[4] < ltblPhaseMin) { ltblPhaseMin = ltblPhaseValue[4]; ltblPhaseMinIndex = 4; }
	if(ltblPhaseValue[5] < ltblPhaseMin) { ltblPhaseMin = ltblPhaseValue[5]; ltblPhaseMinIndex = 5; }
}
void ltblNormalModeStep0()
{
	ltblStep0Normal();
	CalcFilterVal;
	lastTicks = stepTicks[0] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblNormalModeStep1()
{
	ltblStep1Normal();
	CalcFilterVal;
	lastTicks = stepTicks[1] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblNormalModeStep2()
{
	ltblStep2Normal();
	CalcFilterVal;
	lastTicks = stepTicks[2] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblNormalModeStep3()
{
	ltblStep3Normal();
	CalcFilterVal;
	lastTicks = stepTicks[3] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblNormalModeStep4()
{
	ltblStep4Normal();
	CalcFilterVal;
	lastTicks = stepTicks[4] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblNormalModeStep5()
{
	ltblStep5Normal();
	CalcFilterVal;
	lastTicks = stepTicks[5] = ltblWaitH(commTimeout, verTimeout, verMin);
}
#define ltblStartup1PulseUS		1000
#define ltblStartup1DemagUS		1500
void ltblStartup1ModeStep0()
{
	LTBL_REF_TIM->EGR = 1;
	while(1)
	{
		ltblPinToAF();
		ltblStep0Normal();
		delayMicroseconds(ltblStartup1PulseUS);
		LTBL_RESET_ALL;
		ltblPinToPP();
		delayMicroseconds(ltblStartup1DemagUS);
		if(ltblLastThrottle) ltblGetPhaseValue();
		if(0 == 2-(((ltblPhaseMinIndex + 1) % 6) / 2) ||
		0 == 2-(((ltblPhaseMinIndex + 0) % 6) / 2) ||
		0 == 2-(((ltblPhaseMinIndex + 5) % 6) / 2) || !ltblLastThrottle)
		{
			break;
		}
	}
	lastTicks = stepTicks[0] = LTBL_REF_TIM->CNT;
	CalcFilterVal;
	ltblPinToAF();
}
void ltblStartup1ModeStep1()
{
	LTBL_REF_TIM->EGR = 1;
	while(1)
	{
		ltblPinToAF();
		ltblStep1Normal();
		delayMicroseconds(ltblStartup1PulseUS);
		LTBL_RESET_ALL;
		ltblPinToPP();
		delayMicroseconds(ltblStartup1DemagUS);
		if(ltblLastThrottle) ltblGetPhaseValue();
		if(1 == 2-(((ltblPhaseMinIndex + 1) % 6) / 2) ||
		1 == 2-(((ltblPhaseMinIndex + 0) % 6) / 2) ||
		1 == 2-(((ltblPhaseMinIndex + 5) % 6) / 2) || !ltblLastThrottle)
		{
			break;
		}
	}
	lastTicks = stepTicks[1] = LTBL_REF_TIM->CNT;
	CalcFilterVal;
	ltblPinToAF();
}
void ltblStartup1ModeStep2()
{
	LTBL_REF_TIM->EGR = 1;
	while(1)
	{
		ltblPinToAF();
		ltblStep2Normal();
		delayMicroseconds(ltblStartup1PulseUS);
		LTBL_RESET_ALL;
		ltblPinToPP();
		delayMicroseconds(ltblStartup1DemagUS);
		if(ltblLastThrottle) ltblGetPhaseValue();
		if(2 == 2-(((ltblPhaseMinIndex + 1) % 6) / 2) ||
		2 == 2-(((ltblPhaseMinIndex + 0) % 6) / 2) ||
		2 == 2-(((ltblPhaseMinIndex + 5) % 6) / 2) || !ltblLastThrottle)
		{
			break;
		}
	}
	lastTicks = stepTicks[2] = LTBL_REF_TIM->CNT;
	CalcFilterVal;
	ltblPinToAF();
}
void ltblStartup1ModeStep3()
{
	LTBL_REF_TIM->EGR = 1;
	while(1)
	{
		ltblPinToAF();
		ltblStep3Normal();
		delayMicroseconds(ltblStartup1PulseUS);
		LTBL_RESET_ALL;
		ltblPinToPP();
		delayMicroseconds(ltblStartup1DemagUS);
		if(ltblLastThrottle) ltblGetPhaseValue();
		if(0 == 2-(((ltblPhaseMinIndex + 1) % 6) / 2) ||
		0 == 2-(((ltblPhaseMinIndex + 0) % 6) / 2) ||
		0 == 2-(((ltblPhaseMinIndex + 5) % 6) / 2) || !ltblLastThrottle)
		{
			break;
		}
	}
	lastTicks = stepTicks[3] = LTBL_REF_TIM->CNT;
	CalcFilterVal;
	ltblPinToAF();
}
void ltblStartup1ModeStep4()
{
	LTBL_REF_TIM->EGR = 1;
	while(1)
	{
		ltblPinToAF();
		ltblStep4Normal();
		delayMicroseconds(ltblStartup1PulseUS);
		LTBL_RESET_ALL;
		ltblPinToPP();
		delayMicroseconds(ltblStartup1DemagUS);
		if(ltblLastThrottle) ltblGetPhaseValue();
		if(1 == 2-(((ltblPhaseMinIndex + 1) % 6) / 2) ||
		1 == 2-(((ltblPhaseMinIndex + 0) % 6) / 2) ||
		1 == 2-(((ltblPhaseMinIndex + 5) % 6) / 2) || !ltblLastThrottle)
		{
			break;
		}
	}
	lastTicks = stepTicks[4] = LTBL_REF_TIM->CNT;
	CalcFilterVal;
	ltblPinToAF();
}
void ltblStartup1ModeStep5()
{
	LTBL_REF_TIM->EGR = 1;
	while(1)
	{
		ltblPinToAF();
		ltblStep5Normal();
		delayMicroseconds(ltblStartup1PulseUS);
		LTBL_RESET_ALL;
		ltblPinToPP();
		delayMicroseconds(ltblStartup1DemagUS);
		if(ltblLastThrottle) ltblGetPhaseValue();
		if(2 == 2-(((ltblPhaseMinIndex + 1) % 6) / 2) ||
		2 == 2-(((ltblPhaseMinIndex + 0) % 6) / 2) ||
		2 == 2-(((ltblPhaseMinIndex + 5) % 6) / 2) || !ltblLastThrottle)
		{
			break;
		}
	}
	lastTicks = stepTicks[5] = LTBL_REF_TIM->CNT;
	CalcFilterVal;
	ltblPinToAF();
}

void ltblBrakeModeStep0()
{
	ltblStep0Brake();
	CalcFilterVal;
	lastTicks = stepTicks[0] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblBrakeModeStep1()
{
	ltblStep1Brake();
	CalcFilterVal;
	lastTicks = stepTicks[1] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblBrakeModeStep2()
{
	ltblStep2Brake();
	CalcFilterVal;
	lastTicks = stepTicks[2] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblBrakeModeStep3()
{
	ltblStep3Brake();
	CalcFilterVal;
	lastTicks = stepTicks[3] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblBrakeModeStep4()
{
	ltblStep4Brake();
	CalcFilterVal;
	lastTicks = stepTicks[4] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblBrakeModeStep5()
{
	ltblStep5Brake();
	CalcFilterVal;
	lastTicks = stepTicks[5] = ltblWaitH(commTimeout, verTimeout, verMin);
}

void ltblReverseModeStep0()
{
	ltblStep5Normal();
	CalcFilterVal;
	lastTicks = stepTicks[0] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblReverseModeStep1()
{
	ltblStep4Normal();
	CalcFilterVal;
	lastTicks = stepTicks[1] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblReverseModeStep2()
{
	ltblStep3Normal();
	CalcFilterVal;
	lastTicks = stepTicks[2] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblReverseModeStep3()
{
	ltblStep2Normal();
	CalcFilterVal;
	lastTicks = stepTicks[3] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblReverseModeStep4()
{
	ltblStep1Normal();
	CalcFilterVal;
	lastTicks = stepTicks[4] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblReverseModeStep5()
{
	ltblStep0Normal();
	CalcFilterVal;
	lastTicks = stepTicks[5] = ltblWaitH(commTimeout, verTimeout, verMin);
}


void ltblFreeModeStep0()
{
	ltblStep0Free();
	CalcFilterVal;
	lastTicks = stepTicks[0] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblFreeModeStep1()
{
	ltblStep1Free();
	CalcFilterVal;
	lastTicks = stepTicks[1] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblFreeModeStep2()
{
	ltblStep2Free();
	CalcFilterVal;
	lastTicks = stepTicks[2] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblFreeModeStep3()
{
	ltblStep3Free();
	CalcFilterVal;
	lastTicks = stepTicks[3] = ltblWaitH(commTimeout, verTimeout, verMin);
}
void ltblFreeModeStep4()
{
	ltblStep4Free();
	CalcFilterVal;
	lastTicks = stepTicks[4] = ltblWaitL(commTimeout, verTimeout, verMin);
}
void ltblFreeModeStep5()
{
	ltblStep5Free();
	CalcFilterVal;
	lastTicks = stepTicks[5] = ltblWaitH(commTimeout, verTimeout, verMin);
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
	for(; i < sizeof(stepTicks) / sizeof(uint32_t); i++)
	{
		stepTicks[i] = LTBL_START_TICK_MAX;
	}
	LTBL_StopFlag = 0;
	ltblPinToAF();
	for(;;)
	{
		ltblOperationStrategy_step0[ltblCurrentMode]();
		if(LTBL_StopFlag) { break; }
		ltblOperationStrategy_step1[ltblCurrentMode]();
		if(LTBL_StopFlag) { break; }
		ltblOperationStrategy_step2[ltblCurrentMode]();
		if(LTBL_StopFlag) { break; }
		ltblOperationStrategy_step3[ltblCurrentMode]();
		if(LTBL_StopFlag) { break; }
		ltblOperationStrategy_step4[ltblCurrentMode]();
		if(LTBL_StopFlag) { break; }
		ltblOperationStrategy_step5[ltblCurrentMode]();
		if(LTBL_StopFlag) { break; }
	}
	LTBL_UpdateThrottle(0);
	LTBL_RESET_HMOS_U;
	LTBL_RESET_HMOS_V;
	LTBL_RESET_HMOS_W;
	LTBL_RESET_LMOS_U;
	LTBL_RESET_LMOS_V;
	LTBL_RESET_LMOS_W;
	ltblPinToPP();
}

