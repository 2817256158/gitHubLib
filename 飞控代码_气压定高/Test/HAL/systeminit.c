#include "systeminit.h"

void system_peripheral_init(void)
{   
    /*外设初始化*/
    Motor_Init();//电机初始化
    HAL_Delay(100);
    printf("motor ready\r\n");
    E49_Init();//无线传输初始化
    HAL_Delay(100);
    printf("e49 ready\r\n");
    spl0601_init();//spl06初始化
    HAL_Delay(100);
	spl0601_start_continuous(3);
    printf("spl06 ready\r\n");

    MPU6050_Init();//MPU6050初始化
    HAL_Delay(1000);
    printf("mpu6050 ready\r\n");

    HAL_TIM_Base_Start_IT(&htim1);//开启任务处理时钟
    HAL_UART_Receive_IT(&huart1, &USART1_Rx_Byte, 1);//开启串口1数据接收
    HAL_UART_Receive_IT(&huart2, &USART2_Rx_Byte, 1);//开启串口2数据接收
    MX_IWDG_Init();//开启看门狗

    printf("Peripheral Init OK\r\n");
    HAL_Delay(100);

    printf("System start running\r\n");
    HAL_Delay(500);
}
