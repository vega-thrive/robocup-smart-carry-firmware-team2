#include "main.h"
#include "tim.h"

volatile int velocity[5];
volatile int encoder[5];
volatile int motor_pwm[5];   /* Move_Control 最后一次输出的带符号 PWM (监控用) */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

void Move_Init(void)
{
	HAL_GPIO_WritePin(MOTO_POWER_GPIO_Port, MOTO_POWER_Pin, GPIO_PIN_SET);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void Move_Control(int motoa, int motob, int motoc, int motod)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		switch (i)
		{
			case 0:
				if (motoa >= 900) motoa = 900;
				else if (motoa < -950) motoa = -950;
				if (motoa >= 0) {
					HAL_GPIO_WritePin(MOTOA0_GPIO_Port, MOTOA0_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(MOTOA1_GPIO_Port, MOTOA1_Pin, GPIO_PIN_SET);
					TIM8->CCR1 = motoa;
				} else {
					HAL_GPIO_WritePin(MOTOA0_GPIO_Port, MOTOA0_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(MOTOA1_GPIO_Port, MOTOA1_Pin, GPIO_PIN_RESET);
					TIM8->CCR1 = -motoa;
				}
				motor_pwm[1] = motoa;
				break;
			case 1:
				if (motob >= 900) motob = 900;
				else if (motob < -950) motob = -950;
				if (motob >= 0) {
					HAL_GPIO_WritePin(MOTOB0_GPIO_Port, MOTOB0_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(MOTOB1_GPIO_Port, MOTOB1_Pin, GPIO_PIN_RESET);
					TIM8->CCR2 = motob;
				} else {
					HAL_GPIO_WritePin(MOTOB0_GPIO_Port, MOTOB0_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(MOTOB1_GPIO_Port, MOTOB1_Pin, GPIO_PIN_SET);
					TIM8->CCR2 = -motob;
				}
				motor_pwm[2] = motob;
				break;
			case 2:
				if (motoc >= 900) motoc = 900;
				else if (motoc < -950) motoc = -950;
				if (motoc >= 0) {
					HAL_GPIO_WritePin(MOTOC0_GPIO_Port, MOTOC0_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(MOTOC1_GPIO_Port, MOTOC1_Pin, GPIO_PIN_RESET);
					TIM8->CCR3 = motoc;
				} else {
					HAL_GPIO_WritePin(MOTOC0_GPIO_Port, MOTOC0_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(MOTOC1_GPIO_Port, MOTOC1_Pin, GPIO_PIN_SET);
					TIM8->CCR3 = -motoc;
				}
				motor_pwm[3] = motoc;
				break;
			case 3:
				if (motod >= 900) motod = 900;
				else if (motod < -950) motod = -950;
				if (motod >= 0) {
					HAL_GPIO_WritePin(MOTOD0_GPIO_Port, MOTOD0_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(MOTOD1_GPIO_Port, MOTOD1_Pin, GPIO_PIN_SET);
					TIM8->CCR4 = motod;
				} else {
					HAL_GPIO_WritePin(MOTOD0_GPIO_Port, MOTOD0_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(MOTOD1_GPIO_Port, MOTOD1_Pin, GPIO_PIN_RESET);
					TIM8->CCR4 = -motod;
				}
				motor_pwm[4] = motod;
				break;
		}
	}
}

void Move_Velocity(void)
{
	velocity[1] = (short)TIM2->CNT;
	TIM2->CNT = 0;
	velocity[2] = (short)TIM3->CNT;
	TIM3->CNT = 0;
	velocity[3] = -(short)TIM4->CNT;
	TIM4->CNT = 0;
	velocity[4] = -(short)TIM5->CNT;
	TIM5->CNT = 0;
}

void Move_Encoder(void)
{
	encoder[1] += velocity[1];
	encoder[2] += velocity[2];
	encoder[3] += velocity[3];
	encoder[4] += velocity[4];
}
