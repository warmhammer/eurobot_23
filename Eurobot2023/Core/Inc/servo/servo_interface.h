/*
 * servo_interface.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery_Danilov
 *      Edited: Maxim_Popov
 */

#ifndef INC_SERVO_INTERFACE_H_
#define INC_SERVO_INTERFACE_H_

#include <vector>
#include <initializer_list>

#include "stm32f4xx_hal.h"

#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "servo_description.h"

namespace servo_interface {
    class Servo_Interface {
        public:
            Servo_Interface (
				std::initializer_list<servo_description::Servo> servo_params_list,
				ros::NodeHandle& node,
				const char* servo_cmd_topic
            );

            void init(I2C_HandleTypeDef *hi2c);

        private:
            bool _write(const std_msgs::Float32MultiArray& msg);


            std::vector<servo_description::Servo> _servos;

            ros::NodeHandle& _node;
            ros::Subscriber<std_msgs::Float32MultiArray> _pos_topic_subscriber;
    };
}

#endif /* INC_SERVO_INTERFACE_H_ */
