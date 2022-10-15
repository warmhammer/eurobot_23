/*
 * motors.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

// TODO: <...> for libraries and "..." for user's .h files

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>



//!!!!!!!!-----------USER DEFINED PARAMS BEGIN------------!!!!!!!!!!!!!!!!!!!

#define MAX_EMotor_Speed                44      //define max speed of EncoderMotor (rad/s) or 425 rpm
#define TRANS_RATIO                     112     //pulse per revel

//--------------------SYSTEM PARAMS BEGIN-------------------------------------
#define Speed_Timer_Fr                  50000000 // !!!!CAUTION!!!! ALL OF THE MOTOR TIMERS HAVE INCOMMON CLOCK BUS (APB1)
#define Timer_Encoder_init_value        200000

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
	        uint16_t encoder_timer_chanel1,
			TIM_HandleTypeDef* speed_timer,
			uint16_t speed_timer_chanel2,
			TIM_HandleTypeDef* pwm_timer,
			uint16_t pwm_timer_chanel1,
			void (*callback_func)(const std_msgs::Float64&)

	    );

    public:
	    //-------methods EncoderMotor-------------------------------
	    void Init();
        void update_params(float angular_vel, bool ena);
        void set_params();

    public:
        std_msgs::UInt16 angle; //!!
        std_msgs::UInt16 cur_velocity; //!!


    private:
        // TODO: Any private variable or method should starts with _ like _velocity_callback(...) or _velocity_subcriber

        uint16_t _speed_convert(float fval);                                    // float to uint16_t convert
        void _uint_to_float64_speed_converter(uint32_t* uint_value, bool* dir);
        void _uint_to_float64_encoder_converter();

        std_msgs::UInt32 C_Vel;                                                 // current angular velocity
        const float _delta_fi_min_shaft = (2*3.1415)/TRANS_RATIO;                 //radian
        uint16_t setted_vel;

        bool DIR; 					                                            //direction of rotation 0 -is CW, 1 -is CCW
        bool ENA; 					                                            //Motor enable (1 - is enable, 0 - is disable)

        GPIO_TypeDef* DIR_Port;
        uint16_t DIR_Pin;

        GPIO_TypeDef* ENA_Port;
        uint16_t ENA1_Pin;

        TIM_HandleTypeDef* Encoder_Timer;
        uint16_t Encoder_Timer_Chanel1;

        TIM_HandleTypeDef* Pwm_Timer;
        uint16_t Pwm_Timer_Chanel1;
        uint16_t Pulse;

        TIM_HandleTypeDef* Speed_Timer;
        uint16_t Speed_Timer_Chanel1;
        uint16_t Speed_Timer_Chanel2;
        uint32_t speed_data_register2; // current angular velocity 2

};



#endif /* INC_MOTORS_H_ */
