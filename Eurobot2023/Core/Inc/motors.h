/*
 * motors.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stdint.h"
//#include "math.h"
#include "stm32f4xx_hal.h"

//!!!!!!!!-----------USER DEFINED PARAMS BEGIN------------!!!!!!!!!!!!!!!!!!!

#define MAX_EMotor_Speed 6 //define max speed of EncoderMotor

//!!!!!!!!-----------USER DEFINED PARAMS END------------!!!!!!!!!!!!!!!!!!!

uint16_t speed_convert(float fval); // float to uint16_t convert

class EncoderMotor
{
public:
	EncoderMotor(GPIO_TypeDef* dir_port, GPIO_TypeDef* ena_port,
				uint16_t dir_pin, uint16_t ena_pin,
				TIM_HandleTypeDef* encoder_timer,uint16_t encoder_timer_chanel1,
				TIM_HandleTypeDef* speed_timer, uint16_t speed_timer_chanel2,
				TIM_HandleTypeDef* pwm_timer,uint16_t pwm_timer_chanel1)
	{
		c_vel = 0;
		setted_vel = 0;
		DIR = 0;
		ENA = 0;
		Pulse = 0;
		speed_data_register2 = 0;

		DIR_Port = dir_port;
		DIR_Pin = dir_pin;
		ENA_Port = ena_port;
		ENA1_Pin = ena_pin;

		Encoder_Timer = encoder_timer;
		Encoder_Timer_Chanel1 = encoder_timer_chanel1;

		Speed_Timer = speed_timer;
		Speed_Timer_Chanel2 = speed_timer_chanel2;

		Pwm_Timer = pwm_timer;
		Pwm_Timer_Chanel1 = pwm_timer_chanel1;
		//------------------for use HAL config function

		//------------------
		//HAL_TIM_IC_Start_DMA(Speed_Timer, Speed_Timer_Chanel1, &speed_data_register1, 1);
		HAL_TIM_IC_Start_DMA(Speed_Timer, Speed_Timer_Chanel2,&speed_data_register2, 1);
	}

	uint16_t c_vel; 	// current angular velocity
	uint16_t setted_vel;
	bool DIR; 					//direction of rotation 0 -is CW, 1 -is CCW
	bool ENA; 					//Motor enable (1 - is enable, 0 - is disable)

	GPIO_TypeDef* DIR_Port;
	uint16_t DIR_Pin;

	GPIO_TypeDef* ENA_Port;
	uint16_t ENA1_Pin;

	TIM_HandleTypeDef* Encoder_Timer;
	uint16_t Encoder_Timer_Chanel1;

	TIM_HandleTypeDef* Speed_Timer;
	uint16_t Speed_Timer_Chanel1;
	uint16_t Speed_Timer_Chanel2;
	uint32_t speed_data_register1;
	uint32_t speed_data_register2;

	TIM_HandleTypeDef* Pwm_Timer;
	uint16_t Pwm_Timer_Chanel1;
	uint16_t Pulse;


//-------methods EncoderMotor-------------------------------
	void update_params(float w, bool ena);
	void set_params();

//----------------------------------------------------------
	};

#endif /* INC_MOTORS_H_ */
