#ifndef ACUATOR_H
#define ACUATOR_H

#include "main.h"

#define pos2len 115  //  mm/360 degree
#define init_distance 280.0 // mm
void Acuator_reset();
void Acuator_init(uint8_t switch_off);
uint8_t Init_done();

#endif
