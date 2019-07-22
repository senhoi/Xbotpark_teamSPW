#include "rm_esc.h"

RM_Motor_t RM_Motor[8];

void RM_MotorInit(uint8_t id, RM_MotorType_t type, RM_MotorMode_t mode)
{
    RM_MotorEnable(id);
    RM_Motor[id].config.type = type;
    RM_Motor[id].config.mode = mode;
    switch (type)
    {
    case RM_MotorType_M3508:
        RM_Motor[id].config.gear_ratio = 3591.0f / 187.0f;
        PID_Init(&RM_Motor[id].config.pid_pos, PID_Mode_Regular, 8.0f, 0.0f, 0.0f, 0.0f, 5000.0f, -5000.0f, 170.0f);
        PID_Init(&RM_Motor[id].config.pid_vel, PID_Mode_Increment, 10.0f, 0.05f, 0.0f, 0.0f, 16384.0f, -16384.0f, 10000.0f);
        PID_SetIntSp(&RM_Motor[id].config.pid_vel, true, 10.0f);
        break;
    case RM_MotorType_M2006:
        RM_Motor[id].config.gear_ratio = 36.0f;
        PID_Init(&RM_Motor[id].config.pid_pos, PID_Mode_Regular, 8.0f, 0.0f, 0.0f, 0.0f, 5000, -5000, 170.0f);
        PID_Init(&RM_Motor[id].config.pid_vel, PID_Mode_Increment, 10.0f, 0.05f, 0.0f, 0.0f, 16384.0f, -16384.0f, 10000.0f);
				PID_SetIntSp(&RM_Motor[id].config.pid_vel, true, 10.0f);
        break;
    case RM_MotorType_Unknown:
        RM_Motor[id].config.gear_ratio = 1.0f;
        PID_Init(&RM_Motor[id].config.pid_pos, PID_Mode_Regular, 8.0f, 0.0f, 0.0f, 0.0f, 5000, -5000, 170.0f);
        PID_Init(&RM_Motor[id].config.pid_vel, PID_Mode_Increment, 10.0f, 0.05f, 0.0f, 0.0f, 16384.0f, -16384.0f, 10000.0f);
				PID_SetIntSp(&RM_Motor[id].config.pid_vel, true, 10.0f);
        break;
    default:
        break;
    }
    RM_Motor[id].config.set_cur = 0.0f;
    RM_Motor[id].config.set_pos = 0.0f;
    RM_Motor[id].config.set_vel = 0.0f;
}

void RM_MotorGetInfo(CanRxMsg *RxMsg)
{
    uint8_t id = RxMsg->StdId - 0x201;
    int16_t last_pulse, delta_pulse;

    last_pulse = RM_Motor[id].info.pls;
    RM_Motor[id].info.pls = (uint16_t)(RxMsg->Data[0] << 8 | RxMsg->Data[1]);
    RM_Motor[id].info.vel = (int16_t)(RxMsg->Data[2] << 8 | RxMsg->Data[3]);
    RM_Motor[id].info.cur = (int16_t)(RxMsg->Data[4] << 8 | RxMsg->Data[5]);
    RM_Motor[id].info.tmp = (uint8_t)(RxMsg->Data[6]);
    delta_pulse = RM_Motor[id].info.pls - last_pulse;
    if (delta_pulse > 3600)
        RM_Motor[id].info.turn--;
    else if (delta_pulse < -3600)
        RM_Motor[id].info.turn++;
    RM_Motor[id].info.pos = 360.0f * RM_Motor[id].info.turn + RM_Motor[id].info.pls / 8191.0f * 360.0f;
}

uint8_t RM_MotorSendCmd(uint16_t std_id, uint16_t cur1, uint16_t cur2, uint16_t cur3, uint16_t cur4)
{
    uint8_t mailbox = 0;

    CanTxMsg TxMsg;
    TxMsg.StdId = std_id;
    TxMsg.DLC = 0x08;
    TxMsg.IDE = CAN_Id_Standard;
    TxMsg.RTR = CAN_RTR_Data;

    TxMsg.Data[0] = cur1 >> 8;
    TxMsg.Data[1] = cur1;
    TxMsg.Data[2] = cur2 >> 8;
    TxMsg.Data[3] = cur2;
    TxMsg.Data[4] = cur3 >> 8;
    TxMsg.Data[5] = cur3;
    TxMsg.Data[6] = cur4 >> 8;
    TxMsg.Data[7] = cur4;

    mailbox = CAN_Transmit(CAN1, &TxMsg);
    if (mailbox == CAN_TxStatus_NoMailBox)
        return CAN_TxStatus_Failed;
    return CAN_TxStatus_Ok;
}

void RM_MotorSendCmd_auto(void)
{
    uint8_t send_0to3 = 0, send_4to7 = 0;
    for (int id = 0; id < 8; id++)
    {
        if (RM_Motor[id].config.enable == 1)
        {
            if (id < 4)
                send_0to3 = 1;
            else
                send_4to7 = 1;
        }
    }

    if (send_0to3 == 1)
        RM_MotorSendCmd(0x200, RM_Motor[0].config.set_cur, RM_Motor[1].config.set_cur, RM_Motor[2].config.set_cur, RM_Motor[3].config.set_cur);
    if (send_4to7 == 1)
        RM_MotorSendCmd(0x1FF, RM_Motor[4].config.set_cur, RM_Motor[5].config.set_cur, RM_Motor[6].config.set_cur, RM_Motor[7].config.set_cur);
}

void RM_MotorEnable(uint8_t id)
{
    RM_Motor[id].config.enable = 1;
}

void RM_MotorDisable(uint8_t id)
{
    RM_Motor[id].config.enable = 0;
}

void RM_MotorCtrlCalc(void)
{
    for (int id = 0; id < 8; id++)
    {
        if (RM_Motor[id].config.enable == 1)
        {
            if (RM_Motor[id].config.mode == RM_MotorMode_Current)
            {
                RM_Motor[id].config.set_cur = RM_Motor[id].config.set_cur;
            }

            if (RM_Motor[id].config.mode == RM_MotorMode_Velocity)
            {
                RM_Motor[id].config.set_cur = (int16_t)PID_Calc(&RM_Motor[id].config.pid_vel, RM_Motor[id].info.vel);
            }

            if (RM_Motor[id].config.mode == RM_MotorMode_Position)
            {
                RM_MotorSetVel(2, (int16_t)PID_Calc(&RM_Motor[id].config.pid_pos, RM_Motor[id].info.pos));
                RM_Motor[id].config.set_cur = PID_Calc(&RM_Motor[id].config.pid_vel, RM_Motor[id].info.vel);
            }
        }
    }
}

void RM_MotorSetMode(uint8_t id, RM_MotorMode_t mode)
{
    RM_Motor[id].config.mode = mode;
}

void RM_MotorSetCur(uint8_t id, float cur)
{
    RM_Motor[id].config.set_cur = (uint16_t)(cur / 20.f * 16384);
}

void RM_MotorSetVel(uint8_t id, int16_t vel)
{
    RM_Motor[id].config.set_vel = vel;
    PID_SetRef(&RM_Motor[id].config.pid_vel, RM_Motor[id].config.set_vel);
}

void RM_MotorSetPos(uint8_t id, float pos)
{
    RM_Motor[id].config.set_pos = pos * RM_Motor[id].config.gear_ratio;
    PID_SetRef(&RM_Motor[id].config.pid_pos, RM_Motor[id].config.set_pos);
}

void RM_MotorSetPos_(uint8_t id, float pos)
{
    RM_Motor[id].config.set_pos = pos;
    PID_SetRef(&RM_Motor[id].config.pid_pos, RM_Motor[id].config.set_pos);
}
