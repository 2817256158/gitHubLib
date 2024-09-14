#ifndef TASK_H
#define TASK_H

#include "config.h"

typedef struct 
{
    uint8_t run_flag;//1 运行 0 挂起
    uint16_t task_id;//任务ID
    uint16_t task_run_time;//运行时间
    void (*fun) (void);//函数指针
    uint8_t lock;//强制挂起
}Task_Creat;

/*外部更改变量声明区*/
extern float Pitch, Roll, Yaw;
extern float spl06_pressure;
extern float spl06_speed;
extern IWDG_HandleTypeDef hiwdg;

extern volatile uint8_t Motor_Run;
extern volatile int Motor_Speed_Pwm1;//电机四轴实际PWM输出变量
extern volatile int Motor_Speed_Pwm2;
extern volatile int Motor_Speed_Pwm3;
extern volatile int Motor_Speed_Pwm4;   


extern PID_Creat posture_pid_pitch;
extern PID_Creat gyro_pid_pitch;
extern PID_Creat posture_pid_roll;
extern PID_Creat gyro_pid_roll;
extern PID_Creat posture_pid_yaw;
extern PID_Creat gyro_pid_yaw;

extern PID_Creat spl06_pid_1;
extern PID_Creat spl06_pid_2;


void fresh_motor_output(void);//更新电机输出
void message_send(void);//串口任务发送任务
void feed_dog(void);//喂狗任务
void fresh_spl06_data(void);//更新spl06气压数据
void fresh_pid_output(void);//更新PID输出
void clu_pid_val(void);//计算PID输出值

void TaskHandle(void);//任务调度

extern uint16_t Task_Sum;
extern Task_Creat Task_List[10];
#endif
