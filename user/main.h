#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"

//#include "arm_math.h"
#include "acuator.h"
#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "chassis.h"
#include "delay.h"
#include "flash.h"
#include "laser.h"
#include "led.h"
#include "rm_esc.h"
#include "pid.h"
#include "pid_math.h"
#include "power_ctrl.h"
#include "pwm.h"
#include "rc.h"
#include "remote.h"
#include "rng.h"
#include "switch.h"
#include "sys.h"
#include "timer.h"
#include "uart.h"
#include "math.h"
#include "stdlib.h"
#include "user_lib.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;

#define SysCoreClock 180

#define RC_NVIC 2

#define CAN1_NVIC 2
#define CAN2_NVIC 2
#define TIM3_NVIC 2
#define TIM6_NVIC 2
#define SPI5_RX_NVIC 2
#define MPU_INT_NVIC 2

#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#endif /* __MAIN_H */
