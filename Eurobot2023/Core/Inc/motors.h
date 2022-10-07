/*
 * motors.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>



//!!!!!!!!-----------USER DEFINED PARAMS BEGIN------------!!!!!!!!!!!!!!!!!!!

#define MAX_EMotor_Speed 10 //define max speed of EncoderMotor

//!!!!!!!!-----------USER DEFINED PARAMS END------------!!!!!!!!!!!!!!!!!!!


void Wheel_Callback(const std_msgs::Float64&); //wheel CallBack

class EncoderMotor {
    public:
	    EncoderMotor (
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
			const char* cmd_vel_topic,
			const char* cur_vel_topic
	    );

	    //-------methods EncoderMotor-------------------------------
	    uint16_t speed_convert(float fval); // float to uint16_t convert
        void update_params(float angular_vel, bool ena);
        void set_params();
        void velocity_callback(const std_msgs::Float64&);

        ros::Subscriber<std_msgs::Float64> velocity_subcriber;
        ros::Publisher Cur_Vel;

    private:
        // TODO: Any private variable or method should starts with _ like _velocity_callback(...) or _velocity_subcriber

        std_msgs::UInt32 C_Vel; // current angular velocity
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
        uint32_t speed_data_register2; // current angular velocity 2

        TIM_HandleTypeDef* Pwm_Timer;
        uint16_t Pwm_Timer_Chanel1;
        uint16_t Pulse;

};



#endif /* INC_MOTORS_H_ */
