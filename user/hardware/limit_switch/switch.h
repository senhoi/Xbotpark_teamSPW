#ifndef SWITCH_H
#define SWITCH_H

#include "main.h"

#define SWITCH_NAME_FRONT 0
#define SWITCH_NAME_REAR 1
#define SWITCH_NAME_UNKNOWN2 2
#define SWITCH_NAME_UNKNOWN3 3
#define SWITCH_NAME_UNKNOWN4 4
#define SWITCH_NAME_UNKNOWN5 5
#define SWITCH_NAME_UNKNOWN6 6
#define SWITCH_NAME_UNKNOWN7 7
#define SWITCH_NAME_UNKNOWN8 8
#define SWITCH_NAME_UNKNOWN9 9

#define SWITCH_LEVEL_LOW 0
#define SWITCH_LEVEL_HIGH 1

typedef struct Switch_t
{
	void (*exit_func)(void);
	uint8_t level;
} Switch_t;

extern Switch_t Switch[10];

void Switch_Init(void);
void Switch_Scan(void);
void Switch_SetExitFunc(uint8_t switch_idx, void (*exit_func)(void));
uint8_t Switch_GetLevel(uint8_t switch_idx);

#endif
