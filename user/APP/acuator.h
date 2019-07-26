#ifndef ACUATOR_H
#define ACUATOR_H

#include "main.h"

#define pos2len 19  //  mm/360 degree

void Acuator_reset();
void Acuator_init(bool start);
bool Init_done();

#endif
