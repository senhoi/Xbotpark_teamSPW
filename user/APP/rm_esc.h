#ifndef RM_ESC_H
#define RM_ESC_H

#include "../user_lib/pid.h"
#include "main.h"

typedef enum RM_MotorType_t
{
    RM_MotorType_Unknown,
    RM_MotorType_M3508,
    RM_MotorType_M2006
} RM_MotorType_t;

typedef enum RM_MotorMode_t
{
    RM_MotorMode_Current,
    RM_MotorMode_Velocity,
    RM_MotorMode_Position
} RM_MotorMode_t;

typedef struct RM_MotorInfo_t
{
    uint16_t pls; //Unit: Pulse Range: 0~8192 to 0~360 degree
    float pos;    //Unit: degree (calaulated by the number of pulse)
    int16_t turn; //Unit: turns
    int16_t vel;  //Unit: RPM
    int16_t cur;  //Unit: 20/16384 A
    uint8_t tmp;  //Unit: C
} RM_MotorInfo_t;

typedef struct RM_MotorConfig_t
{
    RM_MotorType_t type;
    RM_MotorMode_t mode;
    uint8_t enable;
    float gear_ratio;
    float set_pos;   //Unit: degree
    int16_t set_vel; //Unit: RPM
    int16_t set_cur; //Unit: 20/16384 A
    PID_t pid_pos;
    PID_t pid_vel;
} RM_MotorConfig_t;

typedef struct RM_Motor_t
{
    RM_MotorInfo_t info;
    RM_MotorConfig_t config;
} RM_Motor_t;

extern RM_Motor_t RM_Motor[8];

void RM_MotorInit(uint8_t id, RM_MotorType_t type, RM_MotorMode_t mode);
void RM_MotorSetMode(uint8_t id, RM_MotorMode_t mode);
void RM_MotorSetCur(uint8_t id, float cur);
void RM_MotorSetVel(uint8_t id, int16_t vel);
void RM_MotorSetPos(uint8_t id, float pos);

void RM_MotorGetInfo(CanRxMsg *RxMsg); //This function should be embedded in a high freq timer task.

void RM_MotorCtrlCalc(void);     //This function should be embedded in a relatively low freq timer task.
void RM_MotorSendCmd_auto(void); //This function should be embedded in a relatively low freq timer task.

// The esc id is actually from 1 to 8, the esc id has been modified to 0 to 7 in this program.

uint8_t RM_MotorSendCmd(uint16_t std_id, uint16_t cur1, uint16_t cur2, uint16_t cur3, uint16_t cur4);

void RM_MotorEnable(uint8_t id);
void RM_MotorDisable(uint8_t id);

#endif
