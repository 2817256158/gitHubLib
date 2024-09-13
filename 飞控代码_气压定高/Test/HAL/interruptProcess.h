#ifndef INTERRUPTPROCESS_H
#define INTERRUPTPROCESS_H

#include "config.h"

/*外部全局变量*/
extern volatile uint8_t MPU6050_Ready;

extern float Pitch, Roll, Yaw;//欧拉角
extern short Gyrox, Gyroy, Gyroz;//角速度
extern float Aacx, Aacy, Aacz;//角加速度

extern volatile int Pitch_pwm;
extern volatile int Roll_pwm;
extern volatile int Yaw_pwm;
extern volatile int High_Pwm;

extern volatile int Motor_Speed_Pwm1;
extern volatile int Motor_Speed_Pwm2;
extern volatile int Motor_Speed_Pwm3;
extern volatile int Motor_Speed_Pwm4;
extern volatile int Motor_Base_Pwm;

extern volatile uint8_t MPU6050_Ready;
extern volatile uint8_t Motor_Ready;
extern volatile uint8_t Spl06_Ready;

extern IWDG_HandleTypeDef hiwdg;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t USART1_Rx_Byte;
extern uint8_t USART2_Rx_Byte;

extern uint16_t Task_Sum;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);//mpu6050数据ready 中断处理函数  优先级2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);//定时器 中断处理函数  tim1:优先级0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//串口 中断处理函数  uart1:优先级1  uart2:优先级3
#endif
