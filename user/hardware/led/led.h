#ifndef LED_H
#define LED_H
#include "main.h"

void LED_Config(void);

void LED_Grenn_On(void);
void LED_Grenn_Off(void);
void LED_Green_Toggle(void);

void LED_Red_On(void);
void LED_Red_Off(void);
void LED_Red_Toggle(void);

void LED_Flow_On(uint16_t num);
void LED_Flow_Off(uint16_t num);
void LED_Flow_Toggle(uint16_t num);

#endif
