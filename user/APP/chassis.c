#include "chassis.h"

float Speed_Wheel1 = 0.0f;
float Speed_Wheel2 = 0.0f;
float Speed_Wheel3 = 0.0f;
float Speed_Wheel4 = 0.0f;

float Speed_x = 0.0f;
float Speed_y = 0.0f;
float Speed_a = 0.0f;

float Ratio_v = 20.0f, Ratio_a = 5.0f;
void ChassisControl(void)
{
	const RC_ctrl_t* DT7;
	
	DT7 = get_remote_control_point();
		
	Speed_x = DT7->rc.ch[0];
	Speed_y = DT7->rc.ch[1];
	Speed_a = DT7->rc.ch[2];
	CalcWheelSpeed();
}

void CalcWheelSpeed(void)
{
	Speed_Wheel1 = +Ratio_v * Speed_x - Ratio_v * Speed_y - Ratio_a * Speed_a;
	Speed_Wheel2 = +Ratio_v * Speed_x + Ratio_v * Speed_y - Ratio_a * Speed_a;
	Speed_Wheel3 = -Ratio_v * Speed_x + Ratio_v * Speed_y - Ratio_a * Speed_a;
	Speed_Wheel4 = -Ratio_v * Speed_x - Ratio_v * Speed_y - Ratio_a * Speed_a;
}
