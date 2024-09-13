#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/*无人机机翼分布*/
/*
*电机2        电机1
*     \      /
*
*     /      \
*电机3         电机4
*       机头
*/

void Motor_Init()
{
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
}

void Motor_Speed_Set(int speed_pwm1,int speed_pwm2,int speed_pwm3,int speed_pwm4)
{
    if(speed_pwm1>7200)speed_pwm1=7200;
    if(speed_pwm1<0)speed_pwm1=0;
    if(speed_pwm2>7200)speed_pwm2=7200;
    if(speed_pwm2<0)speed_pwm2=0;
    if(speed_pwm3>7200)speed_pwm3=7200;
    if(speed_pwm3<0)speed_pwm3=0;
    if(speed_pwm4>7200)speed_pwm4=7200;
    if(speed_pwm4<0)speed_pwm4=0;

    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speed_pwm1);


    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,speed_pwm2);


    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,speed_pwm3);


    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,speed_pwm4);
}
