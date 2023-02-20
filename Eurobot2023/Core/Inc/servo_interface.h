/*
 * servo_driver.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery_Danilov
 *      Edited: Maxim_Popov
 */

#ifndef INC_SERVO_INTERFACE_H_
#define INC_SERVO_INTERFACE_H_

#include <stdlib.h>
#include "stm32f4xx_hal.h"

#include "ros.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "servo_description.h"
#include "pca9685.h"

constexpr int SERVO_COUNT = 3; //number of connected servo's

    class Servo_Interface{
        public:
            Servo_Interface(
                    I2C_HandleTypeDef *hi2c,
                    ros::NodeHandle& node,
                    //const char* servo_state_topic_name,
                    const char* servo_cmd_topic,
                    const Servo* servos
            );
            void update();
            void Init(I2C_HandleTypeDef *hi2c);
        private:
            void _write(const std_msgs::Float32MultiArray& msg);

            Servo* _servos[SERVO_COUNT];

            //ros::Publisher _servo_state_publisher;

            ros::NodeHandle& _node;

            ros::Subscriber<std_msgs::Float32MultiArray> _servo_cmd_subscriber;
    };



#endif /* INC_SERVO_INTERFACE_H_ */
