#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"

u8 PWM_Out_Init(uint16_t hz);
void SetPwm(int16_t pwm[],s16 min,s16 max);

#endif

