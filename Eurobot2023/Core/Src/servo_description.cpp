/*
 * servo_description.cpp
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery Danilov
 *      Edited: Maxim Popov
 */

#include "servo_description.h"


Servo::Servo(
        float angle_range,
        float default_position,
        float min_angle,
        float max_angle,
        float operating_vel,
        unsigned int default_frequency
        ):
        _angle_range(angle_range),
        _operating_vel(operating_vel),
        _default_frequency(default_frequency),
        _default_position(default_position),
        _current_position(default_position),
        _max_angle(max_angle),
        _min_angle(min_angle)
        {

        }
const float Servo::get_min_angle(){
    return _min_angle;
}
const float Servo::get_max_angle(){
    return _max_angle;
}
