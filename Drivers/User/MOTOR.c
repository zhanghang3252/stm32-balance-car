#include "MOTOR.h"
#include "tim.h"
#define MOTOR_AIN1_PIN 	GPIO_PIN_12
#define MOTOR_AIN2_PIN 	GPIO_PIN_13
#define MOTOR_BIN1_PIN 	GPIO_PIN_14
#define MOTOR_BIN2_PIN 	GPIO_PIN_15

#define MOTOR_PWMA_PIN 	GPIO_PIN_8
#define MOTOR_PWMB_PIN 	GPIO_PIN_11

#define MOTOR_EN_PIN 		GPIO_PIN_12

#define USE_PWM 	1
void MOTOR_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = MOTOR_AIN1_PIN|MOTOR_AIN2_PIN|MOTOR_BIN1_PIN|MOTOR_BIN2_PIN;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MOTOR_EN_PIN;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	MOTOR_EN(1);
	
	#if(USE_PWM == 0)
	GPIO_InitStruct.Pin = MOTOR_PWMA_PIN|MOTOR_PWMB_PIN;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,MOTOR_PWMA_PIN,1);
	HAL_GPIO_WritePin(GPIOA,MOTOR_PWMB_PIN,1);
	#else
	//HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	#endif
	

}

void MOTOR_EN(uint8_t EN)
{
		HAL_GPIO_WritePin(GPIOA,MOTOR_EN_PIN,EN);
}

void MOTOR_SPEEN(unsigned char motor,uint16_t speed)
{
	if(motor == 1)	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);    //修改比较值，修改占空比
	if(motor == 2)	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, speed);    //修改比较值，修改占空比
}

void MOTOR_ADVANCE(unsigned char motor)
{
	if(motor == 1){
		HAL_GPIO_WritePin(GPIOB,MOTOR_AIN1_PIN,0);
		HAL_GPIO_WritePin(GPIOB,MOTOR_AIN2_PIN,1);	
	}

	if(motor == 2){
		HAL_GPIO_WritePin(GPIOB,MOTOR_BIN1_PIN,0);
		HAL_GPIO_WritePin(GPIOB,MOTOR_BIN2_PIN,1);
	}
}

void MOTOR_RECOIAL(unsigned char motor)
{
	if(motor == 1){
		HAL_GPIO_WritePin(GPIOB,MOTOR_AIN1_PIN,1);
		HAL_GPIO_WritePin(GPIOB,MOTOR_AIN2_PIN,0);	
	}

	if(motor == 2){
		HAL_GPIO_WritePin(GPIOB,MOTOR_BIN1_PIN,1);
		HAL_GPIO_WritePin(GPIOB,MOTOR_BIN2_PIN,0);
	}
}

/*
电机转控制
direction：转动方向 1正转0反转，其他值失能电机
speed：速度控制 0-500
*/
void MOTOR_TURN(unsigned char motor,unsigned char direction, unsigned int speed)
{
	MOTOR_SPEEN(motor,speed);
	if(direction == 1){
		MOTOR_ADVANCE(motor);
	}
	else if(direction == 0){
		MOTOR_RECOIAL(motor);
	}
	else{
		MOTOR_EN(0);//失能电机
	}	
}