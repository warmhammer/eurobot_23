/*
 * motors.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 *      Editor: Maxim Popov
 */
#include "motors.h"

#include <cmath>

namespace motors {
	EncoderMotor::EncoderMotor (
		wrappers::pin_wrapper dir_pin,
		wrappers::pin_wrapper ena_pin,
		wrappers::timer_wrapper encoder_timer,
		wrappers::timer_wrapper velocity_timer,
		wrappers::timer_wrapper pwm_timer,
		bool inversed,
		ros::NodeHandle& node,
		const char* angle_topic_name,
		const char* velocity_topic_name,
		const char* pwd_topic_name
	) :
		_dir_pin(dir_pin),
		_ena_pin(ena_pin),
		_encoder_timer(encoder_timer),
		_velocity_timer(velocity_timer),
		_pwm_timer(pwm_timer),
		_inversed(inversed),
		_node(node),
		_velocity_publisher(velocity_topic_name, &_cur_velocity),
		_angle_publisher(angle_topic_name, &_cur_angle),
		_pwd_subscriber(pwd_topic_name, [&, this](const std_msgs::Float32& msg){this->_write(msg);}),
		_enable(true),
		_written_velocity(0),
		_direction(DIRECT),
		_encoder_tick_duration(0),
		_encoder_tick_count(_encoder_init_value)
	{
		_cur_angle.data = 0;
		_cur_velocity.data = 0;

		_write(_cur_velocity);
	}

	void EncoderMotor::init() {
		__HAL_TIM_SET_COUNTER(_encoder_timer.tim, _encoder_init_value);
		HAL_TIM_IC_Start_DMA(_encoder_timer.tim, _encoder_timer.channel, &_encoder_tick_count, 1);

		HAL_TIM_Base_Start_IT(_velocity_timer.tim);
		HAL_TIM_IC_Start_DMA(_velocity_timer.tim, _velocity_timer.channel, &_encoder_tick_duration, 1);
		_velocity_timer.tim->Instance->CR1 |= 1 << 2;

		_node.advertise(_velocity_publisher);
		_node.advertise(_angle_publisher);

		_node.subscribe(_pwd_subscriber);
	}

	uint16_t EncoderMotor::_angular_velocity_to_pwm (float cmd_vel) {
		if (std::abs(cmd_vel) > MAX_MOTOR_ANGULAR_VEL) {
			return(UINT16_MAX);
		}

		return (std::abs(cmd_vel) / MAX_MOTOR_ANGULAR_VEL) * UINT16_MAX;
	}

	void EncoderMotor::_read_angle() {
		_cur_angle.data = (static_cast<int64_t>(_encoder_tick_count) - _encoder_init_value) * RADS_PER_ENCODER_TICK; //TODO int64_t on the 32 bit system without Float Pointer Unit

		if (_inversed == true) {
			_cur_angle.data = -_cur_angle.data;
		}
	}

	void EncoderMotor::_read_velocity() {
		float vel = _tick_duration_to_angular_velocity(_encoder_tick_duration, _direction);

		if (std::isnan(vel) == false && std::abs(vel) < (1.2f * MAX_MOTOR_ANGULAR_VEL)) {
			_cur_velocity.data = vel;

			if (_inversed == true) {
				_cur_velocity.data = -_cur_velocity.data;
			}
		}
	}

	float EncoderMotor::_tick_duration_to_angular_velocity(uint32_t tick_duration, _Direction direction) {
		if (tick_duration != 0){
			float angular_velocity = (RADS_PER_ENCODER_TICK * VELOCITY_COUNTER_FREQUENCY) / tick_duration;

			if (direction == REVERSE) {
			   return -angular_velocity;
			}

			return angular_velocity;
		}

		return NAN;
	}

	void EncoderMotor::_set_velocity_params(float angular_velocity) {
		if (_enable == true) {
			if(angular_velocity > 1e-5) {
				_direction = (_inversed == false ? DIRECT : REVERSE);
			} else if (angular_velocity < -1e-5) {
				_direction = (_inversed == false ? REVERSE : DIRECT);
			}
			// else angular_velocity == 0: ignoring direction change

			_written_velocity = _angular_velocity_to_pwm(angular_velocity);
		} else {
			_written_velocity = 0;
		}
	}

	void EncoderMotor::_update_hardware() {
		if (_enable == true) {
			if (_direction == DIRECT) {

				HAL_GPIO_WritePin(_dir_pin.port, _dir_pin.pin, GPIO_PIN_RESET);
				_encoder_timer.tim->Instance->CR1 &= ~(1UL << 4);				//set DIR counting up in Encoder timer

			} else if (_direction == REVERSE){

				HAL_GPIO_WritePin(_dir_pin.port, _dir_pin.pin, GPIO_PIN_SET);
				_encoder_timer.tim->Instance->CR1 |= (1UL << 4);				//set DIR counting down in Encoder timer
			}

	//---set PWM----
			uint16_t pulse = _written_velocity * _pwm_timer.tim->Instance->ARR/0xFFFF;
			__HAL_TIM_SET_COMPARE(_pwm_timer.tim, _pwm_timer.channel, pulse);

			if (!(_pwm_timer.tim->Instance->CR1 & (1 << 0))) {
				HAL_TIM_PWM_Start(_pwm_timer.tim, _pwm_timer.channel);
			}

			HAL_GPIO_WritePin(_ena_pin.port, _ena_pin.pin, GPIO_PIN_SET);

		} else {
			_velocity_timer.tim->Instance->EGR |= 1UL << 0;                     // reset velocity_timer
			_velocity_timer.tim->Instance->SR &= ~(1UL << 0);                   // reset interrupt flag

			HAL_GPIO_WritePin(_ena_pin.port, _ena_pin.pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_ena_pin.port, _ena_pin.pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Stop(_pwm_timer.tim, _pwm_timer.channel);

			_encoder_tick_duration = 0;
		}
	}

	const std_msgs::Float32* EncoderMotor::get_cur_velocity() {
		return &_cur_velocity;
	}

	const std_msgs::Float32* EncoderMotor::get_cur_angle() {
		return &_cur_angle;
	}

	void EncoderMotor::publish(bool readFlag) {
		if (readFlag == true) {
			read();
		}

		_velocity_publisher.publish(&_cur_velocity);
		_angle_publisher.publish(&_cur_angle);
	}

	void EncoderMotor::_write(const std_msgs::Float32& msg) {
		_set_velocity_params(msg.data);
		_update_hardware();
	}

	void EncoderMotor::read() {
        _read_velocity();
        _read_angle();
	}

	void EncoderMotor::__set_velocity_to_null__(TIM_HandleTypeDef* htim) {
		if (htim == _velocity_timer.tim) {
			_encoder_tick_duration = UINT16_MAX;
		}
	}
}
