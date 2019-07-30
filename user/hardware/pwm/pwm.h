#ifndef PWM_H
#define PWM_H

#include "main.h"

typedef struct PWM_Servo_t
{
	TIM_TypeDef *tim;
	uint8_t oc;

	uint16_t upper_freq;
	uint16_t lower_freq;

	float max_val;
	float min_val;
	float set_val;

} PWM_Servo_t;

extern PWM_Servo_t Sevro_Trigger;

void PWM_TIM4_Config(void);

void PWM_ServoInit(PWM_Servo_t *servo, TIM_TypeDef *tim, uint8_t oc, uint16_t upper_freq, uint16_t lower_freq, float max_val, float min_val, float set_val);

void PWM_ServoSetVal(PWM_Servo_t *servo, float set_val);

#endif
