#ifndef ACUATOR_H
#define ACUATOR_H

#include "main.h"

typedef struct Actuator_t
{
	uint8_t id;

	uint8_t homing_start;
	uint8_t homing_success;

	float offset; //Unit: degree

	float radius;
	float perimeter;

} Actuator_t;

extern Actuator_t Actr_Lift;
extern Actuator_t Actr_Move;
extern Actuator_t Actr_Roll;
extern Actuator_t Actr_Deepth_Main;
extern Actuator_t Actr_Deepth_Trim;

void Actr_Init(Actuator_t *actr, uint8_t id, float radius);

void Actr_Homing(Actuator_t *actr, uint8_t signal, float homing_shift);
void Actr_HomingReset(Actuator_t *actr);
uint8_t Actr_GetHomingState(Actuator_t *actr);

void Actr_SetPos(Actuator_t *actr, float pos);	//Unit: mm
void Actr_SetMaxVel(Actuator_t *actr, float vel); //Unit: mm/s
float Actr_GetPos(Actuator_t *actr);			  //Unit: mm
float Actr_GetVel(Actuator_t *actr);			  //Unit: mm/s

#endif
