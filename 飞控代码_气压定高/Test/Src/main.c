/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"//头文件库
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*全局姿态角*/
float Pitch, Roll, Yaw;//欧拉角
short Gyrox, Gyroy, Gyroz;//角速度
float Aacx, Aacy, Aacz;//角加速度
/*全局PID输出变量*/
volatile int Pitch_pwm = 0;
volatile int Roll_pwm = 0;
volatile int Yaw_pwm = 0;
volatile int High_Pwm=0;
/*全局四轴电机输出*/
volatile int Motor_Speed_Pwm1 = 0;//电机四轴实际PWM输出变量
volatile int Motor_Speed_Pwm2 = 0;
volatile int Motor_Speed_Pwm3 = 0;
volatile int Motor_Speed_Pwm4 = 0;
volatile int Motor_Base_Pwm = 0;//油门
/*全局外设就绪变量*/
volatile uint8_t MPU6050_Ready = 0;
volatile uint8_t Motor_Run = 0;
volatile uint8_t Spl06_Ready = 0;
/*全局外设数据处理开关变量*/
volatile uint8_t Spl06_Start = 0;
/*全局系统外设句柄*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;
/*全局串口数据变量*/
uint8_t USART1_Rx_Byte;//遥控接收数据
uint8_t USART2_Rx_Byte;//openmv数据
/*全局气压数据*/
float spl06_pressure;//大气压
float spl06_speed;
/*全局姿态环PID参数*/
PID_Creat posture_pid_pitch={-80.0f, -0.0f, -0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,10000.0f};//外环输出限幅无所谓 内环进行限制
PID_Creat gyro_pid_pitch={0.80f, 0.0f, 3.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3000.0f};//输出限幅 把姿态控制区间限制在+-3000内
PID_Creat posture_pid_roll={-80.0f, -0.0f, -0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f};
PID_Creat gyro_pid_roll={0.80f, 0.0f, 3.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3000.0f};
PID_Creat posture_pid_yaw={-80.0f, -0.0f, -0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f};
PID_Creat gyro_pid_yaw={0.80f, 0.0f, 3.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3000.0f};
/*全局高度环PID参数*/
PID_Creat spl06_pid_1={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4000.0f};
PID_Creat spl06_pid_2={100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4000.0f};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //---------------开始系统初始化-----------------//
  system_peripheral_init();
  //---------------Debug模式-----------------//
  #if Debug
  Motor_Run=0;
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		TaskHandle();//任务处理
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
