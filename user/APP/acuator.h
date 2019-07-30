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

	float max_vel;

	float set_pos;

	uint16_t remote_timer;

} Actuator_t;

#define ACTR_MODE_CURRENT 0
#define ACTR_MODE_VELOCITY 1
#define ACTR_MODE_POSITION 2

extern Actuator_t Actr_Lift;
extern Actuator_t Actr_Move;
extern Actuator_t Actr_Roll;
extern Actuator_t Actr_Deepth_Main;
extern Actuator_t Actr_Deepth_Trim;

void Actr_Init(Actuator_t *actr, uint8_t id, float radius, uint8_t mode, float max_vel);

void Actr_Homing(Actuator_t *actr, uint8_t signal, float homing_shift, float homing_rpm);
void Actr_HomingReset(Actuator_t *actr);
uint8_t Actr_GetHomingState(Actuator_t *actr);

void Actr_SetMode(Actuator_t *actr, uint8_t mode);
void Actr_SetPos(Actuator_t *actr, float pos);			//Unit: mm
void Actr_SetPos_Relative(Actuator_t *actr, float pos); //Unit: mm
void Actr_SetMaxVel(Actuator_t *actr, float vel);		//Unit: mm/s
float Actr_GetPos(Actuator_t *actr);					//Unit: mm
float Actr_GetVel(Actuator_t *actr);					//Unit: mm/s

void Actr_Remote_SetPos(Actuator_t *actr, float pos);
void Actr_Remote_SetPos_Relative(Actuator_t *actr, float pos);
float Actr_Remote_GetPos(Actuator_t *actr);
void Actr_Remote_Homing(Actuator_t *actr);
void Actr_Remote_Timer(Actuator_t *actr);

uint8_t Actr_CheckPos(Actuator_t *actr, float error);

void Actr_Release(Actuator_t *actr);
void Actr_Remote_Release(void);

typedef enum Cmd_Spd_t
{
	CMD_SPD_FAST,
	CMD_SPD_NORM
} Cmd_Spd_t;

typedef enum Cmd_Frm_t
{
	CMD_FRM_ABSOLUTE,
	CMD_FRM_RELATIVE
} Cmd_Frm_t;

typedef enum Cmd_Prog_t
{
	CMD_PROG_STOP,
	CMD_PROG_PAUSE,
	CMD_PROG_MOV_ONLY,
	CMD_PROG_WORKING
} Cmd_Prog_t;

typedef enum Cmd_Surf_t
{
	CMD_SURF_FRONT,
	CMD_SURF_TOP,
	CMD_SURF_BOTTOM,
	CMD_SURF_LEFT,
	CMD_SURF_RIGHT
} Cmd_Surf_t;

typedef struct Command_t
{
	float pos_x;
	float pos_y;
	float pos_z;

	Cmd_Spd_t cmd_spd;
	Cmd_Frm_t cmd_frm;
	Cmd_Prog_t cmd_prog;
	Cmd_Surf_t cmd_surf;
} Command_t;

typedef struct Sys_t
{
	uint8_t begin;

	Actuator_t *actr_x;
	Actuator_t *actr_y;
	Actuator_t *actr_z;
	Actuator_t *actr_ro;

	Command_t cmd[100];
	uint8_t cmd_idx;
	uint8_t cmd_counter;

	uint8_t cmd_done;
} Sys_t;

#define SYS_CMD_DONE 1
#define SYS_CMD_NOT_DONE 0

#define SYS_BEGIN 1
#define SYS_NOT_BEGIN 0

#define TRIGGER_OFF() PWM_ServoSetVal(&Sevro_Trigger, 150.0f)
#define TRIGGER_A() PWM_ServoSetVal(&Sevro_Trigger, 240.0f)
#define TRIGGER_B() PWM_ServoSetVal(&Sevro_Trigger, 60.0f)

void Sys_Init(Sys_t *sys, Actuator_t *actr_x, Actuator_t *actr_y, Actuator_t *actr_z, Actuator_t *actr_ro);

void Sys_ExecBegin(Sys_t *sys);

void Sys_ExecStop(Sys_t *sys);

void Cmd_Add(Sys_t *sys, Cmd_Spd_t spd, Cmd_Frm_t frm, float x, float y, float z, Cmd_Surf_t surf, Cmd_Prog_t prog);

void Cmd_Check(Sys_t *sys);

void Cmd_Interpret(Sys_t *sys);

void Cmd_Interpret_Prog(Sys_t *sys);

void Cmd_Interpret_Spd(Sys_t *sys);

void Cmd_Interpret_Frm(Sys_t *sys);

void Cmd_Interpret_Surf(Sys_t *sys);

#endif
