/*
 * motors.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */
#include "motors.h"

uint16_t speed_convert(float fval) // float to uint16_t convert
{
    if (fval > MAX_EMotor_Speed || fval < -MAX_EMotor_Speed) return(UINT16_MAX);
    if (fval >= 0 )
    {
        return((uint16_t)((fval) / (MAX_EMotor_Speed) * UINT16_MAX));
    }else
    {
        return((uint16_t)((-fval) / (MAX_EMotor_Speed) * UINT16_MAX));
    }
}

void EncoderMotor::update_params(float w, bool ena)
{
	if ( w > 0 && ena)
	{
		ENA = 1;
		DIR = 0;
		setted_vel = speed_convert(w);
		c_vel = speed_data_register2;
	}else if (ena)
	{
		ENA = 1;
		DIR = 1;
		setted_vel = speed_convert(w);
		c_vel = speed_data_register2;
	}else
	{
		ENA = 0;
		setted_vel = 0;
		Speed_Timer->Instance->EGR|= 1UL << 0; // reset speed_timer
		Speed_Timer->Instance->SR&= ~(1UL << 0); //reset interrupt flag
	}
}
void EncoderMotor::set_params()
{
	if (ENA)
	{
		if (DIR){
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_SET);
			Encoder_Timer->Instance->CR1|= (1UL << 4); 			//set DIR counting down in Encoder timer
		}else
		{
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
			Encoder_Timer->Instance->CR1&= ~(1UL << 4);			//set DIR counting up in Encoder timer
		}
		//---set PWM----
		Pulse = (setted_vel* Pwm_Timer->Instance->ARR/0xFFFF);
		__HAL_TIM_SET_COMPARE(Pwm_Timer, Pwm_Timer_Chanel1, Pulse);

		if (!(Pwm_Timer->Instance->CR1 & (1<<0)))
		{
			HAL_TIM_PWM_Start(Pwm_Timer, Pwm_Timer_Chanel1);
		}
		//--------------

		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_SET);
	}else
	{
		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(Pwm_Timer, Pwm_Timer_Chanel1);

		speed_data_register2=0;
		c_vel = 0;

	}
}
