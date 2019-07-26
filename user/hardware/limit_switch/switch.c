#include "switch.h"

Switch_t Switch[10];

void Switch_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Switch_SetExitFunc(uint8_t switch_idx, void (*exit_func)(void))
{
	Switch[switch_idx].exit_func = exit_func;
}

bool Switch_GetLevel(uint8_t switch_idx)
{
	return Switch[switch_idx].level;
}

uint8_t debug1,debug2;
void Switch_Scan(void)
{
	Switch[SWITCH_NAME_FRONT].level = PEin(5);
	Switch[SWITCH_NAME_REAR].level = PEin(6);
}

void EXTI9_5_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		EXTI_ClearFlag(EXTI_Line5);
		if (Switch[SWITCH_NAME_FRONT].exit_func != NULL)
			(*Switch[SWITCH_NAME_FRONT].exit_func)();
	}

	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		EXTI_ClearFlag(EXTI_Line6);
		if (Switch[SWITCH_NAME_REAR].exit_func != NULL)
			(*Switch[SWITCH_NAME_REAR].exit_func)();
	}
}
