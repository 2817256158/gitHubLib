#include "task.h"

uint16_t Task_Sum=5;
/*运行状态 ：任务ID ： 运行时间 ：函数指针 : 强制挂起*/
Task_Creat Task_List[10]={    
    {0,1,20,message_send,1},//串口任务发送任务
    {0,2,4,fresh_motor_output,0},//更新电机输出
	{0,3,25,fresh_spl06_data,0}, //更新spl06气压数据
    {0,4,30,fresh_pid_output,0},//更新PID输出
    {0,5,10,clu_pid_val,0},//计算PID输出值
};

void fresh_motor_output()
{
    if(Motor_Run == 1)Motor_Speed_Set(Motor_Speed_Pwm1, Motor_Speed_Pwm2, Motor_Speed_Pwm3, Motor_Speed_Pwm4); 
    else Motor_Speed_Set(0, 0, 0, 0);   
}
void message_send()
{
    printf("samples:%d,%d,%d,%d,%d\r\n",Motor_Speed_Pwm1,Motor_Speed_Pwm2,Motor_Speed_Pwm3,Motor_Speed_Pwm4,Motor_Run);
}
void fresh_spl06_data()
{
    static float pressure_buff[20]={0};
    static float last_spl06_pressure=0;
    static float last_spl06_speed=0;
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);//关闭mpu6050中断，避免中断带来影响
    
    spl06_pressure = (spl06_calibration_altitude_read()*0.3+last_spl06_pressure*0.7);
    spl06_pressure = sliding_filter(spl06_pressure, pressure_buff,10);//滑动滤波
    last_spl06_pressure = spl06_pressure;

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void fresh_pid_output(void)
{
    static int Oil_Lock=1;
    static int High_Lock=0;
    static int place_Lock=0;
    static int zitai_Lock=1;
    static int base_value[4]={150,130,160,150};
    Motor_Speed_Pwm1 = Motor_Base_Pwm*Oil_Lock + base_value[0] + Roll_pwm*zitai_Lock - Pitch_pwm*zitai_Lock - Yaw_pwm*zitai_Lock + High_Pwm*High_Lock;
    Motor_Speed_Pwm2 = Motor_Base_Pwm*Oil_Lock + base_value[1] - Roll_pwm*zitai_Lock - Pitch_pwm*zitai_Lock + Yaw_pwm*zitai_Lock + High_Pwm*High_Lock;
    Motor_Speed_Pwm3 = Motor_Base_Pwm*Oil_Lock + base_value[2] - Roll_pwm*zitai_Lock + Pitch_pwm*zitai_Lock - Yaw_pwm*zitai_Lock + High_Pwm*High_Lock;
    Motor_Speed_Pwm4 = Motor_Base_Pwm*Oil_Lock + base_value[3] + Roll_pwm*zitai_Lock + Pitch_pwm*zitai_Lock + Yaw_pwm*zitai_Lock + High_Pwm*High_Lock;
    if(Motor_Speed_Pwm1>7200)Motor_Speed_Pwm1=7200;
    if(Motor_Speed_Pwm1<0)Motor_Speed_Pwm1=0;
    if(Motor_Speed_Pwm2>7200)Motor_Speed_Pwm2=7200;
    if(Motor_Speed_Pwm2<0)Motor_Speed_Pwm2=0;
    if(Motor_Speed_Pwm3>7200)Motor_Speed_Pwm3=7200;
    if(Motor_Speed_Pwm3<0)Motor_Speed_Pwm3=0;
    if(Motor_Speed_Pwm4>7200)Motor_Speed_Pwm4=7200;
    if(Motor_Speed_Pwm4<0)Motor_Speed_Pwm4=0;
}
void clu_pid_val(void)
{
    High_Pwm=PID_Control(&spl06_pid_1, PID_Control(&spl06_pid_2, 0, spl06_speed), spl06_pressure);
}
void TaskHandle(void)
{
    uint16_t i=0;
    for(i=0;i<Task_Sum;i++)
    {
        if(Task_List[i].run_flag == 1 && Task_List[i].lock == 0)//运行时间到了，并且未强制挂起
        {
            Task_List[i].fun();
            Task_List[i].run_flag = 0;//清除运行标记
        }
    }
}