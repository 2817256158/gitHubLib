#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

/*
*M6 左下  M7 右下
*M2 左上  M1 右上
*/
/*电机Pwm范围0-1000*/
void Motor_Init(void);
void Motor_Speed_Set(int speed_pwm1,int speed_pwm,int speed_pwm3,int speed_pwm4);
#endif