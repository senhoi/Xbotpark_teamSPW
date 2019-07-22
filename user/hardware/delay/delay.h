#ifndef DELAY_H
#define DELAY_H
#include "main.h"
#include "stm32f4xx.h"

void delayInit(uint8_t SysClock);
void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);
void delay_xms(uint16_t nms);

#endif
