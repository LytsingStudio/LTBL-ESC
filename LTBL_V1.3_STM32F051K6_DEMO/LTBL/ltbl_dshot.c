
#include "stm32f0xx.h"
#include "ltbl_dshot.h"

/**
  * 表示 DSHOT 是否处于就绪态
  */
static volatile uint8_t ltblDshotInit = 0;
/**
  * 指定的 DMA 通道
  */
static DMA_Channel_TypeDef *ltblDMAx = 0;
/**
  * 存放 DSHOT 帧的附加信息
  */
static uint8_t ltblDshotExtendInfo[LTBL_SIGNAL_DSHOT_BufferLength];
/**
  * DSHOT 解码完成或信号超时 事件处理器
  */
static void (*LTBL_DSHOT_Captured)(int32_t throttle, uint8_t *ptrInfo) = 0;
/**
  * DSHOT 脉冲时长阈值
  */
uint32_t ltblDshotThreshold = 0;
/**
  * DSHOT 脉冲最小时长
  */
uint32_t ltblDshotPulseMin = 0;
/**
  * DSHOT 脉冲最大时长
  */
uint32_t ltblDshotPulseMax = 0;
/**
  * DSHOT 脉冲时长缓冲区
  */
uint32_t ltblDshotCapturedBuffer[LTBL_SIGNAL_DSHOT_BufferLength];

/**
  * @brief  获取指定的 DMA 通道
	* @param  None
  * @retval None
  */
static DMA_Channel_TypeDef* ltblDshotGetDMAChannel()
{
	if(LTBL_SIGNAL_DSHOT_TIM_ID == 1)
	{
		if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 1)
		{
			return DMA1_Channel2;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 2)
		{
			return DMA1_Channel3;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 3)
		{
			return DMA1_Channel5;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 4)
		{
			return DMA1_Channel4;
		}
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_ID == 2)
	{
		if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 1)
		{
			return DMA1_Channel5;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 2)
		{
			return DMA1_Channel3;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 3)
		{
			return DMA1_Channel1;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 4)
		{
			return DMA1_Channel4;
		}
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_ID == 2)
	{
		if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 1)
		{
			return DMA1_Channel4;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 2)
		{
			return 0;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 3)
		{
			return DMA1_Channel2;
		}
		else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 4)
		{
			return DMA1_Channel3;
		}
	}
	return 0;
}
/**
  * @brief  初始化运行所需的变量，配置 IO、定时器输入捕获等，以便使用 DSHOT 协议解码器
  * @param  None
  * @retval None
  */
void LTBL_DSHOT_Init()
{
	uint32_t fmhz = 0;
	uint32_t pinRes = LTBL_SIGNAL_DSHOT_PIN_RES;
	uint32_t pinCap = LTBL_SIGNAL_DSHOT_PIN_CAP;
	
	uint8_t pinSourceRes = (uint8_t)-1;
	uint8_t pinSourceCap = (uint8_t)-1;
	
	GPIO_InitTypeDef ioConfig;
	TIM_TimeBaseInitTypeDef timConfig;
	TIM_ICInitTypeDef icConfig;
	TIM_OCInitTypeDef ocConfig;
	DMA_InitTypeDef dmaConfig;
	
	ltblDshotInit = YES;
	
	SystemCoreClockUpdate();
	fmhz = SystemCoreClock / 1e6;
	ltblDshotThreshold = fmhz * LTBL_SIGNAL_DSHOT_ThresholdNS / 1000;
	ltblDshotPulseMax = fmhz * LTBL_SIGNAL_DSHOT_PULSE_MAX / 1000;
	ltblDshotPulseMin = fmhz * LTBL_SIGNAL_DSHOT_PULSE_MIN / 1000;
	ltblDMAx = ltblDshotGetDMAChannel();
	
	RCC_AHBPeriphClockCmd(LTBL_SIGNAL_DSHOT_GPIO_CAP_RCCPER, ENABLE);
	RCC_AHBPeriphClockCmd(LTBL_SIGNAL_DSHOT_GPIO_RES_RCCPER, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	if(LTBL_SIGNAL_DSHOT_TIM == TIM1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	if(LTBL_SIGNAL_DSHOT_TIM == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	if(LTBL_SIGNAL_DSHOT_TIM == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_DeInit(LTBL_SIGNAL_DSHOT_TIM);
	
	ioConfig.GPIO_Mode = GPIO_Mode_AF;
	ioConfig.GPIO_OType = GPIO_OType_PP;
	ioConfig.GPIO_Pin = LTBL_SIGNAL_DSHOT_PIN_RES;
	ioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;
	ioConfig.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LTBL_SIGNAL_DSHOT_GPIO_RES, &ioConfig);
	ioConfig.GPIO_Pin = LTBL_SIGNAL_DSHOT_PIN_CAP;
	GPIO_Init(LTBL_SIGNAL_DSHOT_GPIO_CAP, &ioConfig);
	while(pinCap) { pinSourceCap ++; pinCap >>= 1; }
	while(pinRes) { pinSourceRes ++; pinRes >>= 1; }
	GPIO_PinAFConfig(LTBL_SIGNAL_DSHOT_GPIO_CAP, pinSourceCap, LTBL_SIGNAL_DSHOT_AF_CH);
	GPIO_PinAFConfig(LTBL_SIGNAL_DSHOT_GPIO_RES, pinSourceRes, LTBL_SIGNAL_DSHOT_AF_CH);
	
	TIM_TimeBaseStructInit(&timConfig);
	timConfig.TIM_ClockDivision = TIM_CKD_DIV1;
	timConfig.TIM_CounterMode = TIM_CounterMode_Up;
	timConfig.TIM_Period = 0xffff;
	timConfig.TIM_Prescaler = 0;
	TIM_TimeBaseInit(LTBL_SIGNAL_DSHOT_TIM, &timConfig);
	
	TIM_ICStructInit(&icConfig);
	icConfig.TIM_Channel = LTBL_SIGNAL_DSHOT_TIM_RES_Channel;
	icConfig.TIM_ICFilter = 0;
	icConfig.TIM_ICPolarity = TIM_ICPolarity_Rising;
	icConfig.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(LTBL_SIGNAL_DSHOT_TIM, &icConfig);
	TIM_SelectInputTrigger(LTBL_SIGNAL_DSHOT_TIM, LTBL_SIGNAL_DSHOT_TIM_RES_TIXFPX);
	TIM_SelectSlaveMode(LTBL_SIGNAL_DSHOT_TIM, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(LTBL_SIGNAL_DSHOT_TIM, TIM_MasterSlaveMode_Enable);
	
	icConfig.TIM_Channel = LTBL_SIGNAL_DSHOT_TIM_CAP_Channel;
	icConfig.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInit(LTBL_SIGNAL_DSHOT_TIM, &icConfig);
	
	TIM_OCStructInit(&ocConfig);
	ocConfig.TIM_OCMode = TIM_OCMode_PWM1;
	ocConfig.TIM_OCPolarity = TIM_OCPolarity_Low;
	ocConfig.TIM_Pulse = fmhz * LTBL_SIGNAL_DSHOT_FrameTimeoutNS / 1000;
	if(LTBL_SIGNAL_DSHOT_TIM_CUT_CH == 1)
	{
		TIM_OC1Init(LTBL_SIGNAL_DSHOT_TIM, &ocConfig);
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_CUT_CH == 2)
	{
		TIM_OC2Init(LTBL_SIGNAL_DSHOT_TIM, &ocConfig);
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_CUT_CH == 3)
	{
		TIM_OC3Init(LTBL_SIGNAL_DSHOT_TIM, &ocConfig);
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_CUT_CH == 4)
	{
		TIM_OC4Init(LTBL_SIGNAL_DSHOT_TIM, &ocConfig);
	}
	
	dmaConfig.DMA_BufferSize = LTBL_SIGNAL_DSHOT_BufferLength;
	dmaConfig.DMA_DIR = DMA_DIR_PeripheralSRC;
	dmaConfig.DMA_M2M = DMA_M2M_Disable;
	dmaConfig.DMA_MemoryBaseAddr = (uint32_t)ltblDshotCapturedBuffer;
	dmaConfig.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dmaConfig.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dmaConfig.DMA_Mode = DMA_Mode_Normal;
	dmaConfig.DMA_PeripheralBaseAddr = (uint32_t)&LTBL_SIGNAL_DSHOT_TIMCCR;
	dmaConfig.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dmaConfig.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dmaConfig.DMA_Priority = DMA_Priority_High;
	if(ltblDMAx)
	{
		DMA_Init(ltblDMAx, &dmaConfig);
		DMA_Cmd(ltblDMAx, ENABLE);
	}
	
	if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 1)
	{
		TIM_DMACmd(LTBL_SIGNAL_DSHOT_TIM, TIM_DMA_CC1, ENABLE);
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 2)
	{
		TIM_DMACmd(LTBL_SIGNAL_DSHOT_TIM, TIM_DMA_CC2, ENABLE);
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 3)
	{
		TIM_DMACmd(LTBL_SIGNAL_DSHOT_TIM, TIM_DMA_CC3, ENABLE);
	}
	else if(LTBL_SIGNAL_DSHOT_TIM_CAP_CH == 4)
	{
		TIM_DMACmd(LTBL_SIGNAL_DSHOT_TIM, TIM_DMA_CC4, ENABLE);
	}
	
	TIM_ITConfig(LTBL_SIGNAL_DSHOT_TIM, LTBL_SIGNAL_DSHOT_TIM_CUT_IT, ENABLE);
	TIM_ClearITPendingBit(LTBL_SIGNAL_DSHOT_TIM, LTBL_SIGNAL_DSHOT_TIM_CUT_IT);
	NVIC_EnableIRQ(LTBL_SIGNAL_DSHOT_IRQn);
	
	TIM_Cmd(LTBL_SIGNAL_DSHOT_TIM, ENABLE);
}
/**
  * @brief  释放 DSHOT 协议解析所占用的所有外设资源
  * @param  None
  * @retval None
  */
void LTBL_DSHOT_Dispose()
{
	ltblDshotInit = NO;
	DMA_Cmd(ltblDMAx, DISABLE);
	TIM_Cmd(LTBL_SIGNAL_DSHOT_TIM, DISABLE);
	NVIC_DisableIRQ(LTBL_SIGNAL_DSHOT_IRQn);
}
/**
  * @brief  获取当前已安装的事件处理器
	* @param  None
	* @retval LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef : 当前已安装的事件处理器
  */
LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef LTBL_DSHOT_GetCaptureEventHandler()
{
	return LTBL_DSHOT_Captured;
}
/**
  * @brief  为 DSHOT 协议解码完成事件安装指定的事件处理器
	* @param  cap : 指定的事件处理器
  * @retval None
  */
void LTBL_DSHOT_AttachCaptureEvent(LTBL_SIGNAL_DSHOT_CapturedEventHandler_TypeDef cap)
{
	LTBL_DSHOT_Captured = cap;
}

/* DSHOT 数据结构解析器 */
uint8_t LTBL_SIGNAL_DSHOT_StructType1(void);
__asm uint8_t LTBL_SIGNAL_DSHOT_StructType1_ASM(void);

int16_t ltblThrottle = 0;
#if( LTBL_SIGNAL_USE_DSHOT == YES )
void LTBL_DSHOT_Handler()
{
	uint32_t bitCount = LTBL_SIGNAL_DSHOT_BufferLength - ltblDMAx->CNDTR;
	TIM_ClearITPendingBit(LTBL_SIGNAL_DSHOT_TIM, LTBL_SIGNAL_DSHOT_TIM_CUT_IT);
	if(!ltblDshotInit) { return; }
	DMA_Cmd(ltblDMAx, DISABLE);
	ltblDMAx->CNDTR = LTBL_SIGNAL_DSHOT_BufferLength;
	if(bitCount == LTBL_SIGNAL_DSHOT_FrameBitCount && LTBL_SIGNAL_DSHOT_TYPE_FUNC())
	{
		if(LTBL_DSHOT_Captured) LTBL_DSHOT_Captured(ltblThrottle, (uint8_t *)ltblDshotExtendInfo);
	}
	else
	{  
		#if( LTBL_SIGNAL_DSHOT_SetZeroWhenOutOfRange == YES )
		ltblThrottle = 0;
		if(LTBL_DSHOT_Captured) LTBL_DSHOT_Captured(ltblThrottle, (uint8_t *)ltblDshotExtendInfo);
		#endif
	}
	DMA_Cmd(ltblDMAx, ENABLE);
}
#endif

/**
  * @brief  经典 DSHOT 结构解析（11bit 油门信号 + 1bit 回传请求 + 4bit CRC校验码）
	* @param  None
  * @retval 指示是否符合该解析器的规范，0：解析失败、1：解析成功
  */
__asm uint8_t LTBL_SIGNAL_DSHOT_StructType1_ASM()
{
	extern ltblDshotCapturedBuffer
	extern ltblDshotThreshold
	extern ltblDshotPulseMin
	extern ltblDshotPulseMax
	extern ltblThrottle
	
	push {r4-r7}
	
	ldr r0, =ltblDshotCapturedBuffer
	ldr r1, =ltblDshotPulseMin
	ldr r1, [r1]
	
	ldmia r0!,{r4-r7}
	
	cmp r4, r1
	blt ltblSignalDshotStructType1_asm_end
	
	ldr r1, =ltblDshotPulseMax
	ldr r1, [r1]
	
	cmp r4, r1
	bgt ltblSignalDshotStructType1_asm_end
	
	ldr r1, =ltblDshotThreshold
	ldr r1, [r1]
	movs r2, #0x00
	
ltblSignalDshotStructType1_asm_readByte1
	cmp r4, r1
	blt ltblSignalDshotStructType1_asm_readByte2
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte2
	lsls r2, #0x01
	cmp r5, r1
	blt ltblSignalDshotStructType1_asm_readByte3
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte3
	lsls r2, #0x01
	cmp r6, r1
	blt ltblSignalDshotStructType1_asm_readByte4
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte4
	lsls r2, #0x01
	cmp r7, r1
	blt ltblSignalDshotStructType1_asm_readBytePrepare1
	adds r2, #0x01
	
ltblSignalDshotStructType1_asm_readBytePrepare1
	ldmia r0!,{r4-r7}

ltblSignalDshotStructType1_asm_readByte5
	lsls r2, #0x01
	cmp r4, r1
	blt ltblSignalDshotStructType1_asm_readByte6
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte6
	lsls r2, #0x01
	cmp r5, r1
	blt ltblSignalDshotStructType1_asm_readByte7
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte7
	lsls r2, #0x01
	cmp r6, r1
	blt ltblSignalDshotStructType1_asm_readByte8
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte8
	lsls r2, #0x01
	cmp r7, r1
	blt ltblSignalDshotStructType1_asm_readBytePrepare2
	adds r2, #0x01

ltblSignalDshotStructType1_asm_readBytePrepare2
	ldmia r0!,{r4-r7}
	
ltblSignalDshotStructType1_asm_readByte9
	lsls r2, #0x01
	cmp r4, r1
	blt ltblSignalDshotStructType1_asm_readByte10
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte10
	lsls r2, #0x01
	cmp r5, r1
	blt ltblSignalDshotStructType1_asm_readByte11
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte11
	lsls r2, #0x01
	cmp r6, r1
	blt ltblSignalDshotStructType1_asm_readByte12
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte12
	lsls r2, #0x01
	cmp r7, r1
	blt ltblSignalDshotStructType1_asm_readBytePrepare3
	adds r2, #0x01

ltblSignalDshotStructType1_asm_readBytePrepare3
	ldmia r0!,{r4-r7}

ltblSignalDshotStructType1_asm_readByte13
	lsls r2, #0x01
	cmp r4, r1
	blt ltblSignalDshotStructType1_asm_readByte14
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte14
	lsls r2, #0x01
	cmp r5, r1
	blt ltblSignalDshotStructType1_asm_readByte15
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte15
	lsls r2, #0x01
	cmp r6, r1
	blt ltblSignalDshotStructType1_asm_readByte16
	adds r2, #0x01
ltblSignalDshotStructType1_asm_readByte16
	lsls r2, #0x01
	cmp r7, r1
	blt ltblSignalDshotStructType1_asm_readByteVerify
	adds r2, #0x01
	
	ldr r0, =ltblDshotCapturedBuffer
	movs r7, #0
	str r7, [r0]

ltblSignalDshotStructType1_asm_readByteVerify
	movs r7, #0x0f
	lsrs r4, r2, #12
	lsrs r5, r2, #8
	ands r5, r7
	eors r4, r5
	lsrs r5, r2, #4
	ands r5, r7
	eors r4, r5
	mov  r5, r2
	ands r5, r7
	cmp r4, r5
	bne ltblSignalDshotStructType1_asm_end
	ldr r4, =ltblThrottle
	lsrs r2, #5+(11-LTBL_SIGNAL_DSHOT_RESOLUTION_P)
	strh r2, [r4]
	movs r0, #0x01
	pop {r4-r7}
	bx lr
	
ltblSignalDshotStructType1_asm_end
	movs r0, #0x00
	pop {r4-r7}
	bx lr
}
/**
  * @brief  经典 DSHOT 结构解析（11bit 油门信号 + 1bit 回传请求 + 4bit CRC校验码）
	* @param  None
  * @retval 指示是否符合该解析器的规范，0：解析失败、1：解析成功
  */
uint8_t LTBL_SIGNAL_DSHOT_StructType1()
{
	uint16_t block1 = ((ltblDshotCapturedBuffer[0] > ltblDshotThreshold) << 3) |
									 ((ltblDshotCapturedBuffer[1] > ltblDshotThreshold) << 2) |
									 ((ltblDshotCapturedBuffer[2] > ltblDshotThreshold) << 1) |
									 ((ltblDshotCapturedBuffer[3] > ltblDshotThreshold));
	uint16_t block2 = ((ltblDshotCapturedBuffer[4] > ltblDshotThreshold) << 3) |
									 ((ltblDshotCapturedBuffer[5] > ltblDshotThreshold) << 2) |
									 ((ltblDshotCapturedBuffer[6] > ltblDshotThreshold) << 1) |
									 ((ltblDshotCapturedBuffer[7] > ltblDshotThreshold));
	uint16_t block3 = ((ltblDshotCapturedBuffer[8] > ltblDshotThreshold) << 3) |
									 ((ltblDshotCapturedBuffer[9] > ltblDshotThreshold) << 2) |
									 ((ltblDshotCapturedBuffer[10] > ltblDshotThreshold) << 1) |
									 ((ltblDshotCapturedBuffer[11] > ltblDshotThreshold));
	uint16_t crcVer = ((ltblDshotCapturedBuffer[12] > ltblDshotThreshold) << 3) |
									 ((ltblDshotCapturedBuffer[13] > ltblDshotThreshold) << 2) |
									 ((ltblDshotCapturedBuffer[14] > ltblDshotThreshold) << 1) |
									 ((ltblDshotCapturedBuffer[15] > ltblDshotThreshold));
	
	if(ltblDshotCapturedBuffer[0] >= ltblDshotPulseMin && ltblDshotCapturedBuffer[0] <= ltblDshotPulseMax && (((block1 ^ block2 ^ block3) & 0x0f) == crcVer))
	{
		uint32_t rawThrottle = ((block1 << 7) | (block2 << 3) | (block3 >> 1)) & 0x7ff;
		ltblThrottle = LTBL_SIGNAL_DSHOT_RESOLUTION_P * rawThrottle / 0x7ff;
		ltblDshotExtendInfo[0] = (uint8_t)(block3 & 1);
		ltblDshotCapturedBuffer[0] = 0;
		return 1;
	}
	else
	{
		ltblDshotCapturedBuffer[0] = 0;
		return 0;
	}
}

