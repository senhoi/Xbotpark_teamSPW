#include "acuator.h"

static bool prev_switch = false;
static bool init_start = false;
static bool init_success = false;
static float init_pos = 0;
static float set_pos = 0;
static float distance = 0;

void  Acuator_init(bool switch_off){
	
	if(!init_start){
		if(!switch_off){
			RM_MotorSetVel(1, 0);	
		}
		else{
			if(!prev_switch){
				init_start = true;
				RM_MotorInit(1, RM_MotorType_M3508, RM_MotorMode_Position);
				init_pos = RM_Motor[1].info.pos;
				set_pos = init_pos + distance;
				RM_MotorSetPos(1, set_pos);
			}
		}		
	}
	else{
		int dist = RM_Motor[1].info.pos-set_pos;
		if(abs(dist) < 10){
			init_success = true;
		}
	}
		
	prev_switch = switch_off;
	
}

void Acuator_reset(){
	static float init_pos = 0;
	static float set_pos = 0;
	init_start = false;
	init_success = false;
	prev_switch = false;
}

bool Init_done(){
	return init_success;
}