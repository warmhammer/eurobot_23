/*
 * motors.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 *      Editor: Maxim Popov
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <cstdint>

#include "ros.h"
#include "std_msgs/Float32.h"

#include "stm32f4xx_hal.h"

//!!!!!!!!-----------USER DEFINED PARAMS BEGIN------------!!!!!!!!!!!!!!!!!!!

constexpr float MAX_MOTOR_ANGULAR_VEL = 44;             // define max speed of EncoderMotor (rad/s) or 425 rpm
constexpr float ENCODER_TICKS_PER_REVOLUTION = 112.4;   // pulse per revolution
constexpr uint32_t SPEED_TIMER_PRESCALER = 50;

//--------------------SYSTEM PARAMS BEGIN-------------------------------------
constexpr uint32_t VELOCITY_TIMER_FREQUENCY = 50000000;    // !!!!CAUTION!!!! ALL OF THE MOTOR TIMERS HAVE INCOMMON CLOCK BUS (APB1)
//constexpr uint32_t Timer_Encoder_init_value       =  200000;

//--------------------SYSTEM PARAMS END---------------------------------------

//!!!!!!!!-----------USER DEFINED PARAMS END------------!!!!!!!!!!!!!!!!!!!

class EncoderMotor {
    public:
	    EncoderMotor (
	        GPIO_TypeDef* dir_port,
	        GPIO_TypeDef* ena_port,
	        uint16_t dir_pin,
	        uint16_t ena_pin,
	        TIM_HandleTypeDef* encoder_timer,
	        uint16_t encoder_timer_channel, // TODO: ????
			TIM_HandleTypeDef* speed_timer,
			uint16_t speed_timer_channel,
			TIM_HandleTypeDef* pwm_timer,
			uint16_t pwm_timer_channel,
			bool inversed,
			ros::NodeHandle& node,
			const char* angle_topic_name,
			const char* velocity_topic_name,
			const char* pwd_topic_name
	    );

    public:
	    void init();

        void update_angle();
        void update_velocity();

        void update_params(float angular_vel, bool ena);
        void set_params();

        const std_msgs::Float32* get_cur_velocity();
        const std_msgs::Float32* get_cur_angle();

        void publish();

    private:
        // TODO: Any private variable or method should starts with _ like _velocity_callback(...) or _velocity_subcriber

        uint16_t _angular_velocity_to_pwm(float cmd_vel);
        float _tick_duration_to_angular_velocity(const uint32_t tick_duration, bool direction);

        void _callback(const std_msgs::Float32& msg);

        uint32_t _encoder_tick_duration;                                               // current angular velocity
        const uint32_t _encoder_init_value = UINT32_MAX / 2; // TODO: definition in cstr

        const float _rads_per_encoder_tick = (2 * 3.1415) / ENCODER_TICKS_PER_REVOLUTION;   //radian
        uint16_t setted_vel;

        // TODO: direction to enum
        bool DIR; 					                                            // direction of rotation 0 -is CW, 1 -is CCW
        bool _enable; 					                                            //Motor enable (1 - is enable, 0 - is disable)

        GPIO_TypeDef* DIR_Port;
        uint16_t DIR_Pin;

        GPIO_TypeDef* ENA_Port;                                                 // TODO: struct for pins
        uint16_t ENA1_Pin;

        TIM_HandleTypeDef* _encoder_timer;                                      // TODO: struct for timer
        uint16_t _encoder_timer_channel;

        TIM_HandleTypeDef* _pwm_timer;
        uint16_t _pwm_timer_channel;                                           //TODO: const
        uint16_t _pulse;

        TIM_HandleTypeDef* _speed_timer;                                       // TODO: velocity not speed
        uint16_t _speed_timer_channel;
        uint32_t _speed_data_register;                                       // current angular velocity 2

        bool _inversed;

        std_msgs::Float32 _cur_angle;
        std_msgs::Float32 _cur_velocity;

        ros::NodeHandle& _node;

        ros::Publisher _velocity_publisher;
		ros::Publisher _angle_publisher;

		ros::Subscriber<std_msgs::Float32> _pwd_subscriber;
};

#endif /* INC_MOTORS_H_ */
