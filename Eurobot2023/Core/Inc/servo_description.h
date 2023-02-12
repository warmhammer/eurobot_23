/*
 * servo_description.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery Danilov
 *      Edited: Maxim Popov
 */

#ifndef INC_SERVO_DESCRIPTION_H_
#define INC_SERVO_DESCRIPTION_H_

//#include "servo_trajectory_generator.h"
    template <class generator_type >
    class Servo{
        public:
            Servo( float angle_range,
                   float default_position,
                   float min_angle,
                   float max_angle,
                   float operating_vel,
                   unsigned int default_frequency,
                   generator_type* generator);
        private:
            generator_type* _traj_generator;

            float _angle_range;            // [ rad ]
            float _operating_vel;          // [ rad/s ]
            unsigned int _default_frequency;        // [ Hz ]
            float _default_position;       // [ rad ]
            float _min_angle;              // [ rad ]
            float _max_angle;              // [ rad ]
            float _current_position;
    };


#endif /* INC_SERVO_DESCRIPTION_H_ */
