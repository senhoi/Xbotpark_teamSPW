#include "tasks.h"

const RC_ctrl_t *DT7;

UART_Frame_t Uart2PC;
UART_Frame_t Uart2PC_Release;
UART_Frame_t PC2Uart;
int value = 0;
int balcony_approach = 0;
int balcony_arrive = 0;
int balcony_enter = 0;
int balcony_leaving = 0;
int received_data[128]; //[0] represents for X-axis [1] represents for distance
void _TASKS_ManualCtrl(void);
void _TASKS_AutoCtrl_Init(void);
void _TASKS_AutoCtrl(void);

void TASKS_Init()
{
	/* Hardware Configuation */
	delayInit(180);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	LED_Config();

	Remote_Config();

	PowerControl_Config();

	//DevicePower_Config();

	CAN1_Config(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	//CAN2_Config(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

	Switch_Init();

	TIM3_Init(1000, 90);

	TIM6_Init(1000, 90);

	PWM_TIM4_Config();

	USART6_Init();

	/* Actuator Configuation */
	RM_MotorInit(0, RM_MotorType_M3508, RM_MotorMode_Position); //Chassis horizontal axis
	RM_MotorInit(1, RM_MotorType_M3508, RM_MotorMode_Position); //Actuator horizontal axis
	RM_MotorInit(2, RM_MotorType_M3508, RM_MotorMode_Position); //Actuator roll axis
	RM_MotorInit(3, RM_MotorType_M3508, RM_MotorMode_Position); //Actuator horizontal axis

	RM_MotorInit(23, RM_MotorType_M3508, RM_MotorMode_Position); //Actuator vertical axis

	Actr_Init(&Actr_Move, 0, 56.0f / 2.0f / 2, ACTR_MODE_POSITION, 150);
	Actr_Init(&Actr_Lift, 23, 23, ACTR_MODE_POSITION, 100);
	Actr_Init(&Actr_Deepth_Main, 1, 22.5, ACTR_MODE_POSITION, 100);
	Actr_Init(&Actr_Roll, 2, 27.3, ACTR_MODE_POSITION, 360);
	Actr_Init(&Actr_Deepth_Trim, 3, 25.46 / 2, ACTR_MODE_POSITION, 150);

	PWM_ServoInit(&Sevro_Trigger, TIM4, 1, 2500, 500, 300.0f, 0.0f, 150.0f);

	UART_SetTxFrame(&Uart2PC, 0x55AA, 0xAA55, 0x01, UART_FUNC_DATA_N32);
	UART_SetTxFrame(&Uart2PC_Release, 0x55AA, 0xAA55, 0x02, UART_FUNC_DATA_N32);
	UART_SetRxFrame(&PC2Uart, 0x55AA, 0xAA55);

	Uart2PC.ready = true;
}

void TASKS_Timer_H_1000hz()
{
	Switch_Scan();

	RM_MotorCtrlCalc();

	RM_MotorSendCmd_auto();
}

void TASKS_Timer_H_100hz()
{
	memcpy(received_data,PC2Uart.dat,PC2Uart.len);
}

void TASKS_Timer_H_50hz()
{
	DT7 = get_remote_control_point();

	_TASKS_ManualCtrl();

	UART_AutoSend();
}

void TASKS_Timer_H_10hz()
{
}

void TASKS_Timer_H_1hz()
{
}

void TASKS_Timer_L_1000hz()
{
	Actr_Remote_Timer(&Actr_Lift);
}

float remote_motor_fdb[2] = {0};
void TASKS_Timer_L_100hz()
{
	if (UART_GetReadyFlag(&PC2Uart) == 0)
		return;

	memcpy(remote_motor_fdb, PC2Uart.dat, PC2Uart.len);
}

void TASKS_Timer_L_50hz()
{
	/* float motor_data[3];
	if (RM_Motor[2].config.set_vel != 0)
	{
		motor_data[0] = RM_Motor[2].info.pos;
		motor_data[1] = RM_Motor[2].info.vel;
		motor_data[2] = RM_Motor[2].info.cur;
		UART_SendArr_flt(&Uart2PC, motor_data, 3);
		//UART_Printf(&Uart2PC, "%10.3f\t%10.3f\t\r\n", RM_Motor[2].config.set_pos, RM_Motor[2].info.pos);
	}*/
}

void TASKS_Timer_L_10hz()
{
}

void TASKS_Timer_L_1hz()
{
}

void TASKS_While()
{
	while (1)
	{
	}
}

float main_pos = 0;
float trim_pos = 0;
float move_pos = 0;
float roll_deg = 0;
float lift_pos = 0;

Sys_t Sys_FlatWall;

void _TASKS_ManualCtrl(void)
{
	static char s_prev[2];

	switch (DT7->rc.s[0])
	{
	case RC_SW_UP:				   //遥控模式
		if (s_prev[0] != RC_SW_UP) //Edge
		{
			Actr_SetMode(&Actr_Deepth_Main, ACTR_MODE_POSITION);
			Actr_SetMode(&Actr_Deepth_Trim, ACTR_MODE_POSITION);
			Actr_SetMode(&Actr_Move, ACTR_MODE_POSITION);
			Actr_SetMode(&Actr_Roll, ACTR_MODE_POSITION);

			main_pos = Actr_GetPos(&Actr_Deepth_Main);
			trim_pos = Actr_GetPos(&Actr_Deepth_Trim);
			roll_deg = Actr_GetPos(&Actr_Roll);
			move_pos = Actr_GetPos(&Actr_Move);
			
			Actr_Lift.offset = 0;
			lift_pos = Actr_Remote_GetPos(&Actr_Lift);
		}

		lift_pos += 1.0f / 50.0f * DT7->rc.ch[1] / 660.0f * Actr_Lift.max_vel;
		move_pos += 1.0f / 50.0f * DT7->rc.ch[0] / 660.0f * Actr_Move.max_vel;
		main_pos += 1.0f / 50.0f * DT7->rc.ch[2] / 660.0f * Actr_Deepth_Main.max_vel;
		main_pos = limit(540, 0, main_pos);
		trim_pos = -0.0006f * main_pos * main_pos - 0.6017f * main_pos + 68.5432f;
		trim_pos = limit(0, -440, trim_pos);
		roll_deg += 1.0f / 50.0f * DT7->rc.ch[3] / 660.0f * Actr_Roll.max_vel;

		Actr_Remote_SetPos(&Actr_Lift, lift_pos);
		Actr_SetPos(&Actr_Deepth_Trim, trim_pos);
		Actr_SetPos(&Actr_Deepth_Main, main_pos);
		Actr_SetPos(&Actr_Move, move_pos);
		Actr_SetPos(&Actr_Roll, roll_deg);

		switch (DT7->rc.s[1])
		{
		case RC_SW_UP:
			if (s_prev[1] != RC_SW_UP) //Edge
				PWM_ServoSetVal(&Sevro_Trigger, 60.0f);
			break;
		case RC_SW_MID:
			if (s_prev[1] != RC_SW_MID) //Edge
				PWM_ServoSetVal(&Sevro_Trigger, 150.0f);
			break;
		case RC_SW_DOWN:
			if (s_prev[1] != RC_SW_DOWN) //Edge
				PWM_ServoSetVal(&Sevro_Trigger, 240.0f);
			break;

		default:
			break;
		}
		break;

	case RC_SW_MID:					 //卸力模式
		if (s_prev[0] != RC_SW_DOWN) //Edge
		{
			Actr_Release(&Actr_Deepth_Trim);
			Actr_Release(&Actr_Deepth_Main);
			Actr_Release(&Actr_Move);
			Actr_Release(&Actr_Roll);
			PWM_ServoSetVal(&Sevro_Trigger, 150.0f);
			Actr_Remote_Release();
		}
		break;

	case RC_SW_DOWN:				//归位模式
		if (s_prev[0] != RC_SW_MID) //Edge
			;
		switch (DT7->rc.s[1])
		{
		case RC_SW_UP:
			if (s_prev[1] != RC_SW_UP) //Edge
			{
				Actr_HomingReset(&Actr_Deepth_Main);
				Actr_HomingReset(&Actr_Deepth_Trim);
				Actr_HomingReset(&Actr_Move);
				Actr_HomingReset(&Actr_Roll);
			}
			Actr_Homing(&Actr_Deepth_Main, !Switch_GetLevel(SWITCH_NAME_FRONT), 100, -1000.0f);
			Actr_Homing(&Actr_Deepth_Trim, 1, 0, 0.0f);
			Actr_Homing(&Actr_Move, 1, 0, 0.0f);
			Actr_Homing(&Actr_Roll, 1, 0, 0.0f);
			Actr_Remote_Homing(&Actr_Lift);
			
			if( balcony_arrive ){
				if((received_data[2]-cam2acu-50)%10 >4){
					value = ((received_data[2]-cam2acu-50)/10 * 10 +5)*10;
				}
				else{
					value = (received_data[2]-cam2acu-50)/10 * 100;			
				}		
			}
			else{
				if((received_data[1]-cam2acu-50)%10 >4){
					value = ((received_data[1]-cam2acu-50)/10 * 10 +5)*10;
				}
				else{
					value = (received_data[1]-cam2acu-50)/10 * 100;			
				}	
			
			}


					
			if(Actr_GetHomingState(&Actr_Deepth_Main)){
				if(value>max_length){
					Actr_SetPos(&Actr_Deepth_Main,max_length);
				}
				else if(value<=0){
				
				}
				else{
					Actr_SetPos(&Actr_Deepth_Main,value);				
				}
			}
			
			
			if(received_data[1]-received_data[4] >15 && !balcony_approach){
				balcony_approach=1;		
			}
			else if(received_data[1]-received_data[2] >15 && balcony_approach){
				balcony_arrive =1;
				balcony_approach=0;
			}
			else if(balcony_arrive && abs(received_data[1]-received_data[2]) <5){
				balcony_arrive =0;
				balcony_enter = 1;
			}
			else if(balcony_enter && received_data[1]-received_data[2] <-15 ){
				balcony_leaving = 1;
				balcony_enter = 0;				
			}
			else if( balcony_leaving && abs(received_data[1]-received_data[2]) <5){
				balcony_leaving = 0;
			}
			
			

			break;

		case RC_SW_MID:
			if (s_prev[1] != RC_SW_MID) //Edge
				;
			break;

		case RC_SW_DOWN:
			if (s_prev[1] != RC_SW_DOWN) //Edge
				_TASKS_AutoCtrl_Init();
			_TASKS_AutoCtrl();
			break;

		default:
			break;
		}

		break;

	default:
		break;
	}

	s_prev[0] = DT7->rc.s[0];
	s_prev[1] = DT7->rc.s[1];
}

void _TASKS_AutoCtrl_Init(void)
{
	Sys_Init(&Sys_FlatWall, &Actr_Move, &Actr_Lift, &Actr_Deepth_Main, &Actr_Roll);

	/*Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 0, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY);	  //伸出喷头，不喷涂
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 0, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	//开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 500, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //下降一层，不喷涂

	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 700, 500, 500, CMD_SURF_FRONT, CMD_PROG_WORKING); //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 700, 500, 100, CMD_SURF_LEFT, CMD_PROG_WORKING);  //开始喷涂左侧墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 300, 500, 100, CMD_SURF_FRONT, CMD_PROG_WORKING); //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 300, 500, 500, CMD_SURF_RIGHT, CMD_PROG_WORKING); //开始喷涂右侧墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 500, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);   //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 1000, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //下降一层，不喷涂

	Cmd_Add(&Sys_FlatWall, CMD_SPD_FAST, CMD_FRM_ABSOLUTE, 300, 1000, 500, CMD_SURF_BOTTOM, CMD_PROG_MOV_ONLY); //移动到凸台底部
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 700, 1000, 500, CMD_SURF_BOTTOM, CMD_PROG_WORKING);  //喷涂凸台底部

	Cmd_Add(&Sys_FlatWall, CMD_SPD_FAST, CMD_FRM_ABSOLUTE, 0, 1000, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY);   //返回
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 1000, 500, CMD_SURF_FRONT, CMD_PROG_WORKING); //开始喷涂前方墙面

	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 1000, 500, CMD_SURF_FRONT, CMD_PROG_STOP); //停止*/

	/*Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 0, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY);	  //伸出喷头，不喷涂
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 0, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	//开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 500, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //下降一层，不喷涂
	
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 500, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	  //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 1000, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //下降一层，不喷涂
	
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 1000, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	  //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 1000, 500, CMD_SURF_FRONT, CMD_PROG_STOP); //停止*/

	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 0, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY);	  //伸出喷头，不喷涂
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 2000, 0, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	//开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 2000, 400, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //下降一层，不喷涂

	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 400, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);  //开始喷涂前方墙面
	//Cmd_Add(&Sys_FlatWall, CMD_SPD_FAST, CMD_FRM_ABSOLUTE, 300, 400, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //快速移动，不喷涂
	//Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 400, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	//开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 800, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY);   //下降一层，不喷涂

	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 2000, 800, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);	//开始喷涂前方墙面
	//Cmd_Add(&Sys_FlatWall, CMD_SPD_FAST, CMD_FRM_ABSOLUTE, 700, 800, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY);   //快速移动，不喷涂
	//Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 1000, 800, 500, CMD_SURF_FRONT, CMD_PROG_WORKING);   //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 2000, 1200, 500, CMD_SURF_FRONT, CMD_PROG_MOV_ONLY); //下降一层，不喷涂

	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 1200, 500, CMD_SURF_FRONT, CMD_PROG_WORKING); //开始喷涂前方墙面
	Cmd_Add(&Sys_FlatWall, CMD_SPD_NORM, CMD_FRM_ABSOLUTE, 0, 1200, 500, CMD_SURF_FRONT, CMD_PROG_STOP);	//停止*/

	Sys_ExecBegin(&Sys_FlatWall);
}

void _TASKS_AutoCtrl(void)
{
	Cmd_Interpret(&Sys_FlatWall);

	main_pos = Actr_GetPos(Sys_FlatWall.actr_z);

	trim_pos = -0.0006f * main_pos * main_pos - 0.6017f * main_pos + 68.5432f;
	trim_pos = limit(0, -440, trim_pos);

	Actr_SetPos(&Actr_Deepth_Trim, trim_pos);
}
