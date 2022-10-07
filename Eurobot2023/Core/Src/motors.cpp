/*
 * motors.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */
#include "motors.h"

// TODO: Should be a part of EncoderMotor class
uint16_t speed_convert(float fval) { // float to uint16_t convert
    if (fval < -MAX_EMotor_Speed || MAX_EMotor_Speed < fval) {
        return(UINT16_MAX);
    }

    if (fval >= 0) {
        return (fval / MAX_EMotor_Speed) * UINT16_MAX;
    } else {
        return (-fval / MAX_EMotor_Speed) * UINT16_MAX;
    }
}

void EncoderMotor::_velocity_callback(const std_msgs::Float64&) {
	/*implementation*/
}

EncoderMotor::EncoderMotor (
    GPIO_TypeDef* dir_port,
    GPIO_TypeDef* ena_port,
    uint16_t dir_pin,
    uint16_t ena_pin,
    TIM_HandleTypeDef* encoder_timer,
    uint16_t encoder_timer_chanel1,
    TIM_HandleTypeDef* speed_timer,
    uint16_t speed_timer_chanel2,
    TIM_HandleTypeDef* pwm_timer,
    uint16_t pwm_timer_chanel1,
	const char* topic_name
) : _velocity_subcriber(topic_name, [&](const std_msgs::Float64& msg){_velocity_callback(msg);}) {
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

    HAL_TIM_IC_Start_DMA(Speed_Timer, Speed_Timer_Chanel2, &speed_data_register2, 1);
}

void EncoderMotor::update_params(float w, bool ena) { // TODO: change unclear variable names like w
	if (w > 0 && ena) {
		ENA = 1;
		DIR = 0;
		setted_vel = speed_convert(w);
		c_vel = speed_data_register2;
	} else if (ena) {
		ENA = 1;
		DIR = 1;
		setted_vel = speed_convert(w);
		c_vel = speed_data_register2;
	} else {
		ENA = 0;
		setted_vel = 0;
		Speed_Timer->Instance->EGR |= 1UL << 0; // reset speed_timer
		Speed_Timer->Instance->SR &= ~(1UL << 0); //reset interrupt flag
	}
}

void EncoderMotor::set_params() {
	if (ENA) {
		if (DIR) {
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_SET);
			Encoder_Timer->Instance->CR1|= (1UL << 4); 			//set DIR counting down in Encoder timer
		} else {
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
			Encoder_Timer->Instance->CR1&= ~(1UL << 4);			//set DIR counting up in Encoder timer
		}

		//---set PWM----
		Pulse = (setted_vel* Pwm_Timer->Instance->ARR/0xFFFF);
		__HAL_TIM_SET_COMPARE(Pwm_Timer, Pwm_Timer_Chanel1, Pulse);

		if (!(Pwm_Timer->Instance->CR1 & (1<<0))) {
			HAL_TIM_PWM_Start(Pwm_Timer, Pwm_Timer_Chanel1);
		}
		//--------------

		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(Pwm_Timer, Pwm_Timer_Chanel1);

		speed_data_register2=0;
		c_vel = 0;
	}
}

