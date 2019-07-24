#ifndef CAN_H
#define CAN_H

#include "main.h"

uint8_t CAN1_Config(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
uint8_t CAN2_Config(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
#endif