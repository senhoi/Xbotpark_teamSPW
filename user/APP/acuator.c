#include "acuator.h"
static uint8_t prev_switch = 0;
static uint8_t init_start = 0;
static uint8_t init_success = 0;
static float init_pos = 0;
static float set_pos = 0;

void  Acuator_init(uint8_t switch_off){
	
	if(!init_start){
		if(!switch_off){
			RM_MotorSetVel(1, -1000);	
		}
		else{
			if(!prev_switch){
				init_start = 1;
				RM_MotorInit(1, RM_MotorType_M3508, RM_MotorMode_Position);
				init_pos = RM_Motor[1].info.pos/RM_Motor[1].config.gear_ratio;
				RM_Motor[1].config.pid_pos.max_out = 1000.0f;
				RM_Motor[1].config.pid_pos.min_out = -1000.0f;
				set_pos = init_pos + 360*init_distance/pos2len;
				RM_MotorSetPos(1, set_pos);
			}
		}		
	}
	else{
		float dist = RM_Motor[1].info.pos-set_pos;
		if(fabs(dist) < 10){
			init_success = 1;
		}
	}
		
	prev_switch = switch_off;
	
}

void Acuator_reset(){
	init_pos = 0;
	set_pos = 0;
	init_start = 0;
	init_success = 0;
	prev_switch = 0;
}

uint8_t Init_done(){
	return init_success;
}
