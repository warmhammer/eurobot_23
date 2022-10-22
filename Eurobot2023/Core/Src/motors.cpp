/*
 * motors.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */
#include "motors.h"

EncoderMotor::EncoderMotor (
    GPIO_TypeDef* dir_port,
    GPIO_TypeDef* ena_port,
    uint16_t dir_pin,
    uint16_t ena_pin,
    TIM_HandleTypeDef* encoder_timer,
    uint16_t encoder_timer_chanel,
    TIM_HandleTypeDef* speed_timer,
    uint16_t speed_timer_chanel,
    TIM_HandleTypeDef* pwm_timer,
    uint16_t pwm_timer_chanel,
    void (*callback_func)(const std_msgs::Float32&)
) {                                                        // TODO: cstr implementation
    //Node = node;
    _encoder_tick_duration = 0;
    setted_vel = 0;
    DIR = 0;
    _enable = false;
    _pulse = 0;
    _speed_data_register = 0;

    DIR_Port = dir_port;
    DIR_Pin = dir_pin;
    ENA_Port = ena_port;
    ENA1_Pin = ena_pin;

    _encoder_timer = encoder_timer;
    _encoder_timer_channel = encoder_timer_chanel;

    _speed_timer = speed_timer;
    _speed_timer_channel = speed_timer_chanel;

    _pwm_timer = pwm_timer;
    _pwm_timer_channel = pwm_timer_chanel;

}

void EncoderMotor::init() {
    _encoder_init_value = UINT32_MAX / 2;                                               // TODO
    __HAL_TIM_SET_COUNTER(_encoder_timer, _encoder_init_value);                         // set init encoder counter value to prevent underflow (overflow)
    HAL_TIM_Base_Start(_encoder_timer);
    //HAL_TIM_IC_Start(_speed_timer, _speed_timer_channel);                               // start encoder timer
    HAL_TIM_IC_Start_DMA(_speed_timer, _speed_timer_channel, &_encoder_tick_duration, 1);
}

uint16_t EncoderMotor::_angular_velocity_to_pwm (float cmd_vel) {
    if (cmd_vel < -MAX_MOTOR_ANGULAR_VEL || MAX_MOTOR_ANGULAR_VEL < cmd_vel) {
        return(UINT16_MAX);
    }

    if (cmd_vel >= 0) {
        return (cmd_vel / MAX_MOTOR_ANGULAR_VEL) * UINT16_MAX;
    } else {
        return (-cmd_vel / MAX_MOTOR_ANGULAR_VEL) * UINT16_MAX;
    }
}

void EncoderMotor::update_angle() {
    angle.data = (__HAL_TIM_GET_COUNTER(_encoder_timer) - static_cast<float>(_encoder_init_value)) * _rads_per_encoder_tick;
}

void EncoderMotor::update_velocity() {
    if ( _encoder_tick_duration != 0 ){

        cur_velocity.data = _tick_duration_to_angular_velocity(_encoder_tick_duration, DIR);

    }
}

float EncoderMotor::_tick_duration_to_angular_velocity(const uint32_t tick_duration, bool direction) {
    if (tick_duration != 0){
        float angular_velocity = _rads_per_encoder_tick * static_cast<float>(VELOCITY_TIMER_FREQUENCY / SPEED_TIMER_PRESCALER) /  tick_duration ;

        if (direction == true) {
           return -angular_velocity;
        }

        return angular_velocity;
    }

    return 0;
}

void EncoderMotor::update_params(float angular_velocity, bool enable) {
	if (enable == true) {
	    if(angular_velocity > 0) {
            _enable = true;
            DIR = 0;
            setted_vel = _angular_velocity_to_pwm(angular_velocity);
        } else {
            _enable = true;
            DIR = 1;
            setted_vel = _angular_velocity_to_pwm(angular_velocity);
        }
	} else {
		_enable = false;
		setted_vel = 0;
		_speed_timer->Instance->EGR |= 1UL << 0;                     // reset speed_timer
		_speed_timer->Instance->SR &= ~(1UL << 0);                   //reset interrupt flag
	}
	//_uint_to_float64_encoder_converter();                           //CONVERTER
}

void EncoderMotor::set_params() {
	if (_enable) {
		if (DIR) {
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_SET);
			_encoder_timer->Instance->CR1|= (1UL << 4); 			    //set DIR counting down in Encoder timer
		} else {
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
			_encoder_timer->Instance->CR1&= ~(1UL << 4);			    //set DIR counting up in Encoder timer
		}

//---set PWM----
		_pulse = (setted_vel* _pwm_timer->Instance->ARR/0xFFFF);
		__HAL_TIM_SET_COMPARE(_pwm_timer, _pwm_timer_channel, _pulse);

		if (!(_pwm_timer->Instance->CR1 & (1<<0))) {
			HAL_TIM_PWM_Start(_pwm_timer, _pwm_timer_channel);
		}

		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_SET);        // -------------------------------MOTOR START

		//_uint_to_float64_speed_converter(&C_Vel.data, &DIR);        //add CONVERTER tim registers to float64 for speed and angle
//--------------

	} else {
		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_RESET);      // -----------------------------MOTOR STOP
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(_pwm_timer, _pwm_timer_channel);

		_speed_data_register = 0;
		_encoder_tick_duration = 0;
	}

}
