#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h" 

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float last_err;
	float accumulate_err;
}PID_Creat;

int pitch_balance(float Angle,short gyro);
int roll_balance(float Angle,short gyro);
int yaw_balance(float Angle,short gyro);
int PID_Version2(int ex_value,short re_value,int num);
float PID_Control(PID_Creat*object, float expect_val, float real_val);
#endif