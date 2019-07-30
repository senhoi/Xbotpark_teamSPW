#include "pwm.h"

PWM_Servo_t Sevro_Trigger;

void PWM_TIM4_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 90 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
}

void PWM_ServoInit(PWM_Servo_t *servo, TIM_TypeDef *tim, uint8_t oc, uint16_t upper_freq, uint16_t lower_freq, float max_val, float min_val, float set_val)
{
    servo->tim = tim;
    servo->oc = oc;
    servo->upper_freq = upper_freq;
    servo->lower_freq = lower_freq;
    servo->max_val = max_val;
    servo->min_val = min_val;
    servo->set_val = set_val;

    PWM_ServoSetVal(servo, servo->set_val);
}

void PWM_ServoSetVal(PWM_Servo_t *servo, float set_val)
{
    servo->set_val = set_val;

    switch (servo->oc)
    {
    case 1:
        TIM_SetCompare1(servo->tim, (uint32_t)(servo->set_val / (servo->max_val - servo->min_val) * (servo->upper_freq - servo->lower_freq)) + servo->lower_freq);
        break;
    case 2:
        TIM_SetCompare2(servo->tim, (uint32_t)(servo->set_val / (servo->max_val - servo->min_val) * (servo->upper_freq - servo->lower_freq)) + servo->lower_freq);
        break;
    case 3:
        TIM_SetCompare3(servo->tim, (uint32_t)(servo->set_val / (servo->max_val - servo->min_val) * (servo->upper_freq - servo->lower_freq)) + servo->lower_freq);
        break;
    case 4:
        TIM_SetCompare4(servo->tim, (uint32_t)(servo->set_val / (servo->max_val - servo->min_val) * (servo->upper_freq - servo->lower_freq)) + servo->lower_freq);
        break;

    default:
        break;
    }
}
