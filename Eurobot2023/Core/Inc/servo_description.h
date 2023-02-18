/*
 * servo_description.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery Danilov
 *      Edited: Maxim Popov
 */

#ifndef INC_SERVO_DESCRIPTION_H_
#define INC_SERVO_DESCRIPTION_H_


/*struct servo
{
    public:
        void set_servo_params(float angle_range,
                              float default_position,
                              float operating_vel,
                              unsigned int default_frequency){
            _angle_range = angle_range;
            _default_position = default_position;
            _operating_vel = operating_vel;
            _default_frequency = default_frequency;
        }
    private:
        float _angle_range;
        float _default_position;
        float _operating_vel;
        unsigned int _default_frequency;
};*/

//#include "servo_trajectory_generator.h"

class Servo{
        public:
            Servo( float angle_range,
                   float default_position,
                   float min_angle,
                   float max_angle,
                   float operating_vel,
                   unsigned int default_frequency);
           float get_min_angle();
           float get_max_angle();
        private:
            float _angle_range;            // [ rad ]
            float _operating_vel;          // [ rad/s ]
            unsigned int _default_frequency;        // [ Hz ]
            float _default_position;       // [ rad ]
            float _min_angle;              // [ rad ]
            float _max_angle;              // [ rad ]
            float _current_position;
    };



#endif /* INC_SERVO_DESCRIPTION_H_ */
