#ifndef POWER_CTRL_H
#define POWER_CTRL_H

#include "main.h"

#define POWER1_CTRL_SWITCH 0
#define POWER2_CTRL_SWITCH 1
#define POWER3_CTRL_SWITCH 2
#define POWER4_CTRL_SWITCH 3

void PowerControl_Config(void);

void PowerControl_On(uint8_t num);
void PowerControl_Off(uint8_t num);
void PowerControl_Toggle(uint8_t num);
void DevicePower_Config(void);

#endif
