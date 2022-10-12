/*
 * motors.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: Valery_Danilov
 */
#include "motors.h"

uint16_t EncoderMotor::_speed_convert(float fval) { // float to uint16_t convert
    if (fval < -MAX_EMotor_Speed || MAX_EMotor_Speed < fval) {
        return(UINT16_MAX);
    }

    if (fval >= 0) {
        return (fval / MAX_EMotor_Speed) * UINT16_MAX;
    } else {
        return (-fval / MAX_EMotor_Speed) * UINT16_MAX;
    }
}

//void EncoderMotor::pwd_callback(const std_msgs::Float64& pwd) {
//	/*implementation*/
//	// TODO: having there PWD in Float64(!). Should cast to UINT16 and write PWD to motor
//    update_params(pwd.data, 1);
//    set_params();
//    _velocity_publisher.publish(&C_Vel);
////    _angle.data = ...;
////    _angle_publisher.publish(_angle);
//}

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
	ros::NodeHandle& node,
    void (*callback_func)(const std_msgs::Float64&),
	const char* pwd_topic,
	const char* angle_topic,
	const char* cur_vel_topic
)
	  // Node(node)                                           //_pwd_subcriber(pwd_topic, [&](const std_msgs::Float64& msg){_pwd_callback(msg);})
	//_angle_publisher(angle_topic, &_angle)
	//, _velocity_publisher(cur_vel_topic, &_cur_velocity)
    //,_pwd_subcriber(pwd_topic, callback_func)
{
    //Node = node;
    C_Vel.data = 0;
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

    HAL_TIM_IC_Start_DMA(Speed_Timer, Speed_Timer_Chanel2, &C_Vel.data, 1);

    //node.advertise(_angle_publisher);
    //node.advertise(_velocity_publisher);
    //node.subscribe(_pwd_subcriber);

    //node.negotiateTopics();
    //node.logdebug("node adv and subs");
}

void EncoderMotor::_uint_to_float64_speed_converter(uint32_t* uint_value, bool* dir){

        _cur_velocity.data = _delta_fi_min_shaft /( (*uint_value) * (double)(1/Speed_Timer_Fr) );

        if ( (*dir) ){
            _cur_velocity.data *= -1;
        }
}


void EncoderMotor::update_params(float angular_vel, bool ena) {
	if (angular_vel > 0 && ena) {
		ENA = 1;
		DIR = 0;
		setted_vel = _speed_convert(angular_vel);
	} else if (ena) {
		ENA = 1;
		DIR = 1;
		setted_vel = _speed_convert(angular_vel);
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
		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_SET);    // -------------------------------MOTOR START
		_uint_to_float64_speed_converter(&C_Vel.data, &DIR);    //add converter tim registers to float64 for speed and angle
        //--------------

	} else {
		HAL_GPIO_WritePin(ENA_Port, ENA1_Pin, GPIO_PIN_RESET); // -----------------------------MOTOR STOP
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(Pwm_Timer, Pwm_Timer_Chanel1);

		speed_data_register2=0;
		C_Vel.data = 0;
	}

}

