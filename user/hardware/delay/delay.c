#include "delay.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

void delayInit(uint8_t SysClock)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us = SysClock / 8;				//不论是否使用OS,fac_us都需要使用
	fac_ms = (u16)fac_us * 1000;		//非OS下,代表每个ms需要的systick时钟数   
}

void delay_us(uint16_t nus)
{
    uint32_t temp;	    	 
	SysTick -> LOAD = nus *fac_us;  		 
	SysTick -> VAL = 0x00;
	SysTick -> CTRL |= SysTick_CTRL_ENABLE_Msk ;
	do
	{
		temp=SysTick->CTRL;
	}while((temp & 0x01) && !(temp & (1 << 16)));		//等待时间到达   
	SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick -> VAL = 0X00;							//清空计数器 
}

void delay_ms(uint16_t nms)
{
    uint8_t repeat = nms / 540;
	uint16_t remain = nms % 540;
	while(repeat)
	{
		delay_xms(540);
		repeat--;
	}
	if(remain)
		delay_xms(remain);
}

void delay_xms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick -> LOAD = (uint32_t)nms * fac_ms;
	SysTick -> VAL = 0x00;
	SysTick -> CTRL |= SysTick_CTRL_ENABLE_Msk ;
	do
	{
		temp = SysTick -> CTRL;
	}while((temp & 0x01) && !(temp & (1 << 16))); 
	SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick -> VAL = 0X00;
} 
