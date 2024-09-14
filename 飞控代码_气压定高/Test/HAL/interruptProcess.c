#include "interruptProcess.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
  if (GPIO_Pin == GPIO_PIN_15)
  {
    /*mpu6050姿态计算必须放中断 保证数据的实时性*/
    mpu_dmp_get_data(&Roll, &Pitch, &Yaw);
    MPU_Get_Gyroscope(&Gyroy, &Gyrox, &Gyroz);
    MPU_Get_Accelerometer(&Aacx,&Aacy,&Aacz);
    
    MPU6050_Calibration(Aacx,Aacy,Aacz,Gyrox,Gyroy,Gyroz);//未校正则开始校正

    Pitch_pwm = PID_Control(&gyro_pid_pitch, PID_Control(&posture_pid_pitch, 0.0f, Pitch), (float)(Gyroy*-1.0));
    Roll_pwm = PID_Control(&gyro_pid_roll, PID_Control(&posture_pid_roll, 0.0f, Roll), (float)(Gyrox*-1.0));
    #if 0
    Yaw_pwm = PID_Version2(yaw_balance(Yaw,(short)(Gyroz)*-1.0),Gyroz,2);
    Pitch_pwm = PID_Version2(pitch_balance(Pitch,(short)(Gyroy)*-1.0),Gyroy,1);
    Roll_pwm = PID_Version2(roll_balance(Roll,(short)(Gyroz)*-1.0),Gyrox,0);
    #endif
  }                                                                                                                     
  UNUSED(GPIO_Pin);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t TIM1_Tick=0;
  uint16_t i=0;
  if (htim == &htim1)
  {
    TIM1_Tick++;
		if ( TIM1_Tick % 50 == 0)HAL_IWDG_Refresh(&hiwdg);
		if ( TIM1_Tick % 25 == 0)spl0601_get_raw_pressure(),spl0601_get_raw_temp();   
    for( i=0 ; i < Task_Sum ; i++){if ( TIM1_Tick % Task_List[i].task_run_time == 0)Task_List[i].run_flag = 1;}//判断是否到任务运行时间
    if (TIM1_Tick == 1000001)TIM1_Tick = 0;     
  }
  UNUSED(htim);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  { 
    uart1_process(USART1_Rx_Byte);//串口数据处理
    printf("rx:%c %d\r\n",USART1_Rx_Byte,Motor_Run);
    USART1_Rx_Byte = 0;
    HAL_UART_Receive_IT(&huart1, &USART1_Rx_Byte, 1);//开始接受下一字节数据
  }
  if(huart == &huart2)
  {
    
  }
  UNUSED(huart);
}