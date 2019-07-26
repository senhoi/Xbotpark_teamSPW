#include "acuator.h"

Actuator_t Actr_Lift;
Actuator_t Actr_Move;
Actuator_t Actr_Roll;
Actuator_t Actr_Deepth_Main;
Actuator_t Actr_Deepth_Trim;

void Actr_Init(Actuator_t *actr, uint8_t id, float radius)
{
	actr->id = id;
	actr->radius = radius;
	actr->perimeter = 2 * 3.1416f * radius;
}

void Actr_Homing(Actuator_t *actr, uint8_t signal, float homing_shift)
{
	if (!Actr_GetHomingState(actr))
	{
		if (!actr->homing_start)
		{
			if (!signal)
			{
				if (RM_Motor[actr->id].config.mode != RM_MotorMode_Velocity)
					RM_MotorInit(actr->id, RM_MotorType_M3508, RM_MotorMode_Velocity);
				RM_MotorSetVel(actr->id, -1000);
			}
			else
			{
				if (RM_Motor[actr->id].config.mode != RM_MotorMode_Position)
				{
					RM_MotorInit(actr->id, RM_MotorType_M3508, RM_MotorMode_Position);
					Actr_SetMaxVel(actr, 100.0f);
				}
				actr->homing_start = 1;
				actr->offset = RM_MotorGetPos(actr->id);
				Actr_SetPos(actr, homing_shift);
			}
		}
		else
		{
			float dist = Actr_GetPos(actr) - homing_shift;
			if (fabs(dist) < 10)
			{
				actr->homing_success = 1;
			}
		}
	}
}

void Actr_HomingReset(Actuator_t *actr)
{
	actr->homing_start = 0;
	actr->homing_success = 0;
	actr->offset = 0;
}

uint8_t Actr_GetHomingState(Actuator_t *actr)
{
	return actr->homing_success;
}

void Actr_SetPos(Actuator_t *actr, float pos)
{
	RM_MotorSetPos(actr->id, actr->offset + 360 * pos / actr->perimeter);
}

void Actr_SetMaxVel(Actuator_t *actr, float vel)
{
	RM_Motor[actr->id].config.pid_pos.max_out = vel / actr->perimeter * 60.0f * RM_Motor[actr->id].config.gear_ratio;
	RM_Motor[actr->id].config.pid_pos.min_out = -vel / actr->perimeter * 60.0f * RM_Motor[actr->id].config.gear_ratio;
}

float Actr_GetPos(Actuator_t *actr)
{
	return RM_MotorGetPos(actr->id) / 360 * actr->perimeter;
}

float Actr_GetVel(Actuator_t *actr)
{
	return RM_MotorGetVel(actr->id) * actr->perimeter / 60.0f / RM_Motor[actr->id].config.gear_ratio;
}
