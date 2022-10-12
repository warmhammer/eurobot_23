/*
 * callbacks.h
 *
 *  Created on: Oct 10, 2022
 *      Author: valery
 */

#ifndef INC_CALLBACKS_H_
#define INC_CALLBACKS_H_

#include <std_msgs/Float64.h>

    void encoder_motor_left_callback(const std_msgs::Float64& msg);
    void encoder_motor_right_callback(const std_msgs::Float64& msg);



#endif /* INC_CALLBACKS_H_ */
