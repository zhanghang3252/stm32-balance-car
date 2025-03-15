/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MOTOR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

float pitch,roll,yaw;
short aacx,aacy,aacz;			
short gyrox,gyroy,gyroz;		
float temp;//温度
int16_t Count1=0,Count2=0,Diretion1=0,Diretion2=0;

float error0=0,error1=0,error2=0;//本次误差 上次误差 上上次误差
float Integral_error = 0.0;//误差积分

float Differential_error=0.0;//误差微分
float Differential_error1=0.0;//上一次误差微分
float Diff_velocity=0.2;//微分滤波强度

float kp=15.9,ki=0.4,kd=15.6;//pid控制项
float current_value1=0;//上一次实际值
float pwm_max = 450.0,pwm_min=20.0;//PWM限幅
float pwm=0;//输出pwm
uint8_t direction=0;//旋转方向
float target = 0.0;//目标值

void Balance_control(float current_value,float set_value)
{
	//如果获取数据出错继续返回上一次pwm值
	if(current_value>360.0 | current_value<360.0)
	{
			MOTOR_TURN(1,direction,fabs(pwm));		
			MOTOR_TURN(2,direction,fabs(pwm));
	}		
		
	//	error2 = error1;//上上次误差
	error1 = error0;//上次误差
	error0 = set_value - current_value;//当前误差
	//积分分离
	if(fabs(error0) < 10)	{
		Integral_error+=error0;//积分误差
	}
	else{
		Integral_error=0;
	}
	//积分限幅防止积分深度饱和提升系统响应
	if(fabs(Integral_error)>pwm_max)
	{
		if(Integral_error>0)	Integral_error = pwm_max;
		if(Integral_error<0)	Integral_error = -1*pwm_max;
	}
	Differential_error=(error0-error1);//误差微分
	pwm = kp*(error0)+ki*(Integral_error)+kd*Differential_error;//位置式pwm输出计算
	if(pwm < 0)		direction = 0;//正传
	if(pwm > 0)		direction = 1;//反转
	
	if(fabs(pwm) > pwm_max)//输出限幅
	{
		if (pwm < 0) 	pwm = -pwm_max;
		if (pwm > 0)	pwm = pwm_max;	
	}
	if(fabs(pwm) < pwm_min)//输出限幅
	{
		if (pwm < 0) 	pwm = -pwm_min;
		if (pwm > 0)	pwm = pwm_min;	
	}
	MOTOR_TURN(1,direction,fabs(pwm));		
	MOTOR_TURN(2,direction,fabs(pwm));
//	printf("%.2f %.2f %.2f\r\n",error_value,error_accumulation,last_error_value);
	printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",current_value,set_value,pwm,error0,Integral_error,kd*Differential_error);
	current_value1=current_value;
}

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	 if(MPU_Init() != 0){
		printf("\r\nMPU_Init error!!!\r\n");
		return 0;
	 }
	 else{
		printf("\r\nMPU_Init ok!!!\r\n");
	 }
	 if(mpu_dmp_init() != 0){
			printf("\r\nmpu_dmp_init error!!!\r\n");
			return 0;
	 }	
	 else{
		printf("\r\nmpu_dmp_init ok!!!\r\n");
	 }	 
		MOTOR_INIT();	 
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //开启编码器模式
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //开启编码器模式
	 	HAL_TIM_Base_Start_IT(&htim2);                  //开启定时器的中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
			while(mpu_dmp_get_data(&pitch, &roll, &yaw));
//			MOTOR_TURN(1,direction,40);		
//			MOTOR_TURN(2,direction,40);
//		MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		
//		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
//		temp=MPU_Get_Temperature();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t tim2_count=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		tim2_count++;
		if(tim2_count>=5)//5ms进一次
		{
			tim2_count = 0;
			Balance_control(pitch,target);	
		}
	}
}

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
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
