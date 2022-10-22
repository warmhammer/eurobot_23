/*
 * callbacks.h
 *
 *  Created on: Oct 10, 2022
 *      Author: valery
 */

#ifndef INC_CALLBACKS_H_
#define INC_CALLBACKS_H_

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>


    void left_encoder_motor_callback(const std_msgs::Float32& msg);
    void right_encoder_motor_callback(const std_msgs::Float32& msg);
   // void test_callback(const std_msgs::Float64& msg){}



#endif /* INC_CALLBACKS_H_ */
