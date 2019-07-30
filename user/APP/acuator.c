#include "acuator.h"

Actuator_t Actr_Lift;
Actuator_t Actr_Move;
Actuator_t Actr_Roll;
Actuator_t Actr_Deepth_Main;
Actuator_t Actr_Deepth_Trim;

void Actr_Init(Actuator_t *actr, uint8_t id, float radius, uint8_t mode, float max_vel)
{
	actr->id = id;
	actr->radius = radius;
	actr->perimeter = 2 * 3.1416f * radius;
	actr->max_vel = max_vel;
	Actr_SetMode(actr, mode);
	Actr_SetMaxVel(actr, max_vel);
}

void Actr_Homing(Actuator_t *actr, uint8_t signal, float homing_shift, float homing_rpm)
{
	if (!Actr_GetHomingState(actr))
	{
		if (!actr->homing_start)
		{
			if (!signal)
			{
				if (RM_Motor[actr->id].config.mode != RM_MotorMode_Velocity)
					RM_MotorInit(actr->id, RM_MotorType_M3508, RM_MotorMode_Velocity);
				RM_MotorSetVel(actr->id, homing_rpm);
			}
			else
			{
				if (RM_Motor[actr->id].config.mode != RM_MotorMode_Position)
				{
					RM_MotorInit(actr->id, RM_MotorType_M3508, RM_MotorMode_Position);
					Actr_SetMaxVel(actr, actr->max_vel);
				}
				actr->homing_start = 1;
				if (actr->id < 16)
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

void Actr_SetMode(Actuator_t *actr, uint8_t mode)
{
	switch (mode)
	{
	case ACTR_MODE_CURRENT:
		RM_Motor[actr->id].config.mode = RM_MotorMode_Current;
		break;

	case ACTR_MODE_VELOCITY:
		RM_Motor[actr->id].config.mode = RM_MotorMode_Velocity;
		break;

	case ACTR_MODE_POSITION:
		RM_Motor[actr->id].config.mode = RM_MotorMode_Position;
		break;

	default:
		break;
	}
}

void Actr_SetCur(Actuator_t *actr, float cur)
{
	RM_MotorSetCur(actr->id, cur);
}

void Actr_SetPos(Actuator_t *actr, float pos)
{
	actr->set_pos = pos;
	RM_MotorSetPos(actr->id, actr->offset + 360 * pos / actr->perimeter);
}

void Actr_SetPos_Relative(Actuator_t *actr, float pos)
{
	actr->set_pos = pos + Actr_GetPos(actr);
	Actr_SetPos(actr, pos + Actr_GetPos(actr));
}

uint8_t Actr_CheckPos(Actuator_t *actr, float error)
{
	if (fabs(actr->set_pos - Actr_GetPos(actr)) <= error)
		return 1;
	else
		return 0;
}

void Actr_SetMaxVel(Actuator_t *actr, float vel)
{
	RM_Motor[actr->id].config.pid_pos.max_out = vel / actr->perimeter * 60.0f * RM_Motor[actr->id].config.gear_ratio;
	RM_Motor[actr->id].config.pid_pos.min_out = -vel / actr->perimeter * 60.0f * RM_Motor[actr->id].config.gear_ratio;
}

float Actr_GetPos(Actuator_t *actr)
{
	return (RM_MotorGetPos(actr->id) - actr->offset) / 360 * actr->perimeter;
}

float Actr_GetVel(Actuator_t *actr)
{
	return RM_MotorGetVel(actr->id) * actr->perimeter / 60.0f / RM_Motor[actr->id].config.gear_ratio;
}

void Actr_Release(Actuator_t *actr)
{
	Actr_SetMode(actr, ACTR_MODE_CURRENT);
	Actr_SetCur(actr, 0);
}

void Actr_Return(Actuator_t *actr)
{
	Actr_SetMode(actr, ACTR_MODE_POSITION);
	Actr_SetPos(actr, 0);
}

static int32_t remote_motor_data[2];

void Actr_Remote_Timer(Actuator_t *actr)
{
	if (actr->remote_timer > 0)
		actr->remote_timer--;
}

void Actr_Remote_SetPos_Relative(Actuator_t *actr, float pos)
{
	actr->set_pos += pos;
	remote_motor_data[0] = (actr->offset + 360 * actr->set_pos / actr->perimeter);
	remote_motor_data[1] = -(actr->offset + 360 * actr->set_pos / actr->perimeter);

	UART_SendArr_32b(&Uart2PC, remote_motor_data, 2);
}

void Actr_Remote_SetPos(Actuator_t *actr, float pos)
{
	actr->remote_timer = 7.0f * fabs(actr->offset + 360 * pos / actr->perimeter - remote_motor_data[0]);
	actr->set_pos = pos;
	remote_motor_data[0] = (actr->offset + 360 * pos / actr->perimeter);
	remote_motor_data[1] = -(actr->offset + 360 * pos / actr->perimeter);

	UART_SendArr_32b(&Uart2PC, remote_motor_data, 2);
}

float Actr_Remote_GetPos(Actuator_t *actr)
{
	return remote_motor_data[0] / 360.0f * actr->perimeter;
}

void Actr_Remote_Homing(Actuator_t *actr)
{
	actr->offset = Actr_Remote_GetPos(actr) / actr->perimeter * 360;
}

uint8_t Actr_Remote_CheckPos(Actuator_t *actr, float error_time)
{
	if (actr->remote_timer < error_time)
		return 1;
	else
		return 0;
}

extern UART_Frame_t Uart2PC_Release;
void Actr_Remote_Release(void)
{
	UART_SendArr_32b(&Uart2PC_Release, NULL, 0);
}
/**********************************/

void Sys_Init(Sys_t *sys, Actuator_t *actr_x, Actuator_t *actr_y, Actuator_t *actr_z, Actuator_t *actr_ro)
{
	sys->begin = SYS_NOT_BEGIN;

	sys->actr_x = actr_x;
	sys->actr_y = actr_y;
	sys->actr_z = actr_z;
	sys->actr_ro = actr_ro;

	sys->cmd_idx = 0;
	sys->cmd_counter = 0;
	sys->cmd_done = SYS_CMD_NOT_DONE;
}

void Sys_ExecBegin(Sys_t *sys)
{
	sys->begin = SYS_BEGIN;
}

void Sys_ExecStop(Sys_t *sys)
{
	sys->begin = SYS_NOT_BEGIN;
}

void Cmd_Add(Sys_t *sys, Cmd_Spd_t spd, Cmd_Frm_t frm, float x, float y, float z, Cmd_Surf_t surf, Cmd_Prog_t prog)
{
	sys->cmd[sys->cmd_counter].cmd_spd = spd;
	sys->cmd[sys->cmd_counter].cmd_frm = frm;
	sys->cmd[sys->cmd_counter].pos_x = x;
	sys->cmd[sys->cmd_counter].pos_y = y;
	sys->cmd[sys->cmd_counter].pos_z = z;
	sys->cmd[sys->cmd_counter].cmd_surf = surf;
	sys->cmd[sys->cmd_counter].cmd_prog = prog;
	sys->cmd_counter++;
}

void Cmd_Check(Sys_t *sys)
{
	if (Actr_CheckPos(sys->actr_x, 5) &&
		Actr_Remote_CheckPos(sys->actr_y, 5) &&
		Actr_CheckPos(sys->actr_z, 5) &&
		Actr_CheckPos(sys->actr_ro, 5))
		sys->cmd_done = SYS_CMD_DONE;
}

void Cmd_Interpret(Sys_t *sys)
{
	if (sys->begin == SYS_BEGIN)
	{
		Cmd_Check(sys);
		if (sys->cmd_done == SYS_CMD_DONE)
		{
			Cmd_Interpret_Prog(sys);
			Cmd_Interpret_Spd(sys);
			Cmd_Interpret_Frm(sys);
			Cmd_Interpret_Surf(sys);
			sys->cmd_done = SYS_CMD_NOT_DONE;
			sys->cmd_idx++;
			if (sys->cmd_idx == sys->cmd_counter)
				Sys_ExecStop(sys);
		}
	}
}

void Cmd_Interpret_Prog(Sys_t *sys)
{
	switch (sys->cmd[sys->cmd_idx].cmd_prog)
	{
	case CMD_PROG_STOP:
		Actr_Return(sys->actr_x);
		Actr_Return(sys->actr_y);
		Actr_Return(sys->actr_z);
		TRIGGER_OFF();
		break;

	case CMD_PROG_PAUSE:
		Actr_SetPos(sys->actr_x, Actr_GetPos(sys->actr_x));
		Actr_Remote_SetPos(sys->actr_y, Actr_Remote_GetPos(sys->actr_y));
		Actr_SetPos(sys->actr_z, Actr_GetPos(sys->actr_z));
		break;

	case CMD_PROG_MOV_ONLY:
		TRIGGER_OFF();
		break;

	case CMD_PROG_WORKING:
		break;

	default:
		break;
	}
}

void Cmd_Interpret_Spd(Sys_t *sys)
{
	switch (sys->cmd[sys->cmd_idx].cmd_spd)
	{
	case CMD_SPD_FAST:
		Actr_SetMaxVel(sys->actr_x, sys->actr_x->max_vel * 2);
		//Actr_SetMaxVel(sys->actr_y, sys->actr_y->max_vel * 2);
		Actr_SetMaxVel(sys->actr_z, sys->actr_z->max_vel * 2);
		break;

	case CMD_SPD_NORM:
		Actr_SetMaxVel(sys->actr_x, sys->actr_x->max_vel);
		//Actr_SetMaxVel(sys->actr_y, sys->actr_y->max_vel);
		Actr_SetMaxVel(sys->actr_z, sys->actr_z->max_vel);
		break;

	default:
		break;
	}
}

void Cmd_Interpret_Frm(Sys_t *sys)
{
	switch (sys->cmd[sys->cmd_idx].cmd_frm)
	{
	case CMD_FRM_ABSOLUTE:
		Actr_SetPos(sys->actr_x, sys->cmd[sys->cmd_idx].pos_x);
		Actr_Remote_SetPos(sys->actr_y, sys->cmd[sys->cmd_idx].pos_y);
		Actr_SetPos(sys->actr_z, sys->cmd[sys->cmd_idx].pos_z);
		break;

	case CMD_FRM_RELATIVE:
		Actr_SetPos_Relative(sys->actr_x, sys->cmd[sys->cmd_idx].pos_x);
		Actr_Remote_SetPos_Relative(sys->actr_y, sys->cmd[sys->cmd_idx].pos_y);
		Actr_SetPos_Relative(sys->actr_z, sys->cmd[sys->cmd_idx].pos_z);
		break;

	default:
		break;
	}
}

void Cmd_Interpret_Surf(Sys_t *sys)
{
	switch (sys->cmd[sys->cmd_idx].cmd_surf)
	{
	case CMD_SURF_FRONT:
	case CMD_SURF_LEFT:
		Actr_SetPos(sys->actr_ro, 0);
		break;

	case CMD_SURF_RIGHT:
		Actr_SetPos(sys->actr_ro, 180);
		break;

	case CMD_SURF_TOP:
		Actr_SetPos(sys->actr_ro, 90);
		break;

	case CMD_SURF_BOTTOM:
		Actr_SetPos(sys->actr_ro, -90);
		break;

	default:
		break;
	}

	if (sys->cmd[sys->cmd_idx].cmd_prog == CMD_PROG_STOP || sys->cmd[sys->cmd_idx].cmd_prog == CMD_PROG_MOV_ONLY)
	{
		TRIGGER_OFF();
		return;
	}

	switch (sys->cmd[sys->cmd_idx].cmd_surf)
	{
	case CMD_SURF_FRONT:
		Actr_SetPos(sys->actr_ro, 0);
		TRIGGER_A();
		break;

	case CMD_SURF_LEFT:
	case CMD_SURF_RIGHT:
	case CMD_SURF_TOP:
	case CMD_SURF_BOTTOM:
		TRIGGER_B();
		break;

	default:
		break;
	}
}
