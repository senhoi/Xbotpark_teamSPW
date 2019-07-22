#include "led.h"

void LED_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    LED_Grenn_Off();
    LED_Red_Off();

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    LED_Flow_Off(GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8);
}

void LED_Grenn_Off(void)
{
    GPIO_SetBits(GPIOF, GPIO_Pin_14);
}

void LED_Grenn_On(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_14);
}

void LED_Green_Toggle(void)
{
    GPIO_ToggleBits(GPIOF, GPIO_Pin_14);
}

void LED_Red_Off(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_11);
}
void LED_Red_On(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}

void LED_Red_Toggle(void)
{
    GPIO_ToggleBits(GPIOE, GPIO_Pin_11);
}

void LED_Flow_On(uint16_t num)
{
    GPIO_ResetBits(GPIOG, num);
}

void LED_Flow_Off(uint16_t num)
{
    GPIO_SetBits(GPIOG, num);
}

void LED_Flow_Toggle(uint16_t num)
{
    GPIO_ToggleBits(GPIOG, num);
}
