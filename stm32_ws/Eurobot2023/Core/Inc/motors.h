/*
 * motors.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 *      Editor: Maxim Popov
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdint.h>

#include "ros.h"
#include "std_msgs/Float32.h"

#include "stm32f4xx_hal.h"

#include "wrappers.h"

namespace motors {
	constexpr float MAX_MOTOR_ANGULAR_VEL = 425 * (2 * 3.14159) / 60;             // define max speed of EncoderMotor (rad/s) or 425 rpm
	constexpr float ENCODER_TICKS_PER_REVOLUTION = 18.8 * 6;   // pulse per revolution
	constexpr float RADS_PER_ENCODER_TICK = (2 * 3.14159) / (ENCODER_TICKS_PER_REVOLUTION * 2);   //radian, both tick edges

	constexpr uint32_t SPEED_TIMER_PRESCALER = 199 + 1;
	constexpr uint32_t VELOCITY_TIMER_FREQUENCY = 50000000;
	constexpr float VELOCITY_COUNTER_FREQUENCY = static_cast<float>(VELOCITY_TIMER_FREQUENCY) / SPEED_TIMER_PRESCALER;

	class EncoderMotor {
		public:
			EncoderMotor (
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
			);

		public:
			void init();

			void read();
			void publish(bool readFlag = true);

			const std_msgs::Float32* get_cur_velocity();
			const std_msgs::Float32* get_cur_angle();

			/*Pseudo public methods*/
			void __set_velocity_to_null__(TIM_HandleTypeDef* htim);

		private:
			enum _Direction {DIRECT, REVERSE};


			void _read_angle();
			void _read_velocity();

			void _write(const std_msgs::Float32& msg);
			void _set_velocity_params(float angular_vel);
			void _update_hardware();

			uint16_t _angular_velocity_to_pwm(float cmd_vel);
			float _tick_duration_to_angular_velocity(uint32_t tick_duration, _Direction direction);


			wrappers::pin_wrapper _dir_pin;
			wrappers::pin_wrapper _ena_pin;

			wrappers::timer_wrapper _encoder_timer;
			wrappers::timer_wrapper _velocity_timer;
			wrappers::timer_wrapper _pwm_timer;

			bool _inversed;

			ros::NodeHandle& _node;

			ros::Publisher _velocity_publisher;
			ros::Publisher _angle_publisher;

			ros::Subscriber<std_msgs::Float32> _pwd_subscriber;

			std_msgs::Float32 _cur_angle;
			std_msgs::Float32 _cur_velocity;

			bool _enable;
			uint16_t _written_velocity;

			_Direction _direction;
			uint32_t _encoder_tick_duration;
			uint32_t _encoder_tick_count;

			static constexpr uint32_t _encoder_init_value = UINT32_MAX / 2;
	};
}

#endif /* INC_MOTORS_H_ */
