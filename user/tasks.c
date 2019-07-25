#include "tasks.h"

const RC_ctrl_t *DT7;

PWM_Servo_t Sevro_Trigger;

UART_Frame_t Uart2PC;
UART_Frame_t PC2Uart;

void _TASKS_ManualCtrl(void);

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

	TIM3_Init(1000, 90);

	TIM6_Init(1000, 90);

	PWM_TIM4_Config();

	USART6_Init();

	/* Actuator Configuation */
	RM_MotorInit(0, RM_MotorType_M3508, RM_MotorMode_Velocity); //Chassis horizontal axis
	RM_MotorInit(1, RM_MotorType_M3508, RM_MotorMode_Velocity); //Actuator horizontal axis
	RM_MotorInit(2, RM_MotorType_M3508, RM_MotorMode_Velocity); //Actuator roll axis
	RM_MotorInit(3, RM_MotorType_M3508, RM_MotorMode_Velocity); //Actuator horizontal axis
	RM_MotorInit(4, RM_MotorType_M3508, RM_MotorMode_Velocity); //Actuator roll axis

	PWM_ServoInit(&Sevro_Trigger, TIM4, 1, 2500, 500, 300.0f, 0.0f, 150.0f);

	UART_SetTxFrame(&Uart2PC, 0x55AA, 0xAA55, 0x01, UART_FUNC_DATA_N32);
	UART_SetRxFrame(&PC2Uart, 0x55AA, 0xAA55);

	Uart2PC.ready = true;
}


void TASKS_Timer_H_1000hz()
{
	RM_MotorCtrlCalc();

	RM_MotorSendCmd_auto();
}

void TASKS_Timer_H_100hz()
{
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
}

void TASKS_Timer_L_100hz()
{
	UART_GetReadyFlag(&PC2Uart);
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

void _TASKS_ManualCtrl()
{
	static char s_prev[2];
	int32_t remote_motor_data[2];

	switch (DT7->rc.s[0])
	{
	case RC_SW_UP:
 
		remote_motor_data[0] = 10 * DT7->rc.ch[0];
		remote_motor_data[1] = -10 * DT7->rc.ch[0];

		RM_MotorSetVel(0, 10 * DT7->rc.ch[1]);
		RM_MotorSetVel(1, 10 * DT7->rc.ch[2]);
		RM_MotorSetVel(2, 10 * DT7->rc.ch[3]);
		
		UART_SendArr_32b(&Uart2PC, remote_motor_data, 2);

		switch (DT7->rc.s[1])
		{
		case RC_SW_UP:
			if (s_prev[1] != RC_SW_UP) //Edge
				PWM_ServoSetVal(&Sevro_Trigger, 0.0f);
			break;
		case RC_SW_MID:
			if (s_prev[1] != RC_SW_MID) //Edge
				PWM_ServoSetVal(&Sevro_Trigger, 150.0f);
			break;
		case RC_SW_DOWN:
			if (s_prev[1] != RC_SW_DOWN) //Edge
				PWM_ServoSetVal(&Sevro_Trigger, 300.0f);
			break;

		default:
			break;
		}
		if (s_prev[0] != RC_SW_UP) //Edge
			;
		break;

	case RC_SW_MID:
		if (s_prev[0] != RC_SW_MID) //Edge
			;
		break;

	case RC_SW_DOWN:
		if (s_prev[0] != RC_SW_DOWN) //Edge
			;
		break;

	default:
		break;
	}

}
