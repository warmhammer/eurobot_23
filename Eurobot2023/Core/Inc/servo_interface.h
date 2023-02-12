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

#include "ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"

//#include "servo_trajectory_generator.h"
//#include "servo_description.h"

constexpr int SERVO_COUNT = 10; //number of connected servo's

    class Servo_Interface{
        public:
            Servo_Interface(
                    ros::NodeHandle& node,
                    const char* servo_state_topic_name,
                    const char* servo_cmd_topic
            );
        private:
            ros::Publisher _servo_state_publisher;

            ros::Subscriber<std_msgs::UInt16MultiArray> _servo_cmd_subscriber;
    };



#endif /* INC_SERVO_INTERFACE_H_ */
