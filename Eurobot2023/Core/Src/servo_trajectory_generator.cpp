/*
 * motors.cpp
 *
 *  Created on: Jan 27, 2023
 *      Author: Valery_Danilov
 *      Editor: Maxim Popov
 */
#include "servo_trajectory_generator.h"

#include <cmath>

    Minimum_Jerk_Generator::Minimum_Jerk_Generator(
            uint32_t frequency,
            uint16_t current_pos
            )

            {
                _setted_pos = current_pos;
                _frequency = frequency;
                _current_pos = current_pos;
                _error = 0;
                _sample_count = 0;
                _current_sample = 0;
                _state = 0;
                _travel_time = 0;
            }

    void Minimum_Jerk_Generator::init(){

    }

    uint16_t Minimum_Jerk_Generator::do_step(uint16_t position){
            if (_current_sample <= _sample_count && _sample_count != 0){
                _current_sample++;

                if (_setted_pos > _current_pos) {
                    position = _current_pos
                            + static_cast<uint16_t>((_setted_pos - _current_pos) *
                            (a3*pow(_current_sample/_sample_count,3)
                            + a4*pow(_current_sample/_sample_count,4)
                            + a5*pow(_current_sample/_sample_count,5)));



                }else{
                    position = _current_pos
                             - static_cast<uint16_t>((_current_pos -_setted_pos) *
                             (a3*pow(_current_sample/_sample_count,3)
                             + a4*pow(_current_sample/_sample_count,4)
                             + a5*pow(_current_sample/_sample_count,5)));

                }
            }else{
                _state = 0;
                _current_sample = 0;
                _sample_count = 0;
                _current_pos = position;
            }


            _current_pos = position;


            return position;
        }

    uint16_t Minimum_Jerk_Generator::set_pos(uint16_t position){ //return the next pos from generator
        if (!_state && position != _current_pos){
            _state = 1;
            _setted_pos = position;
            _sample_count = static_cast<uint16_t>(_travel_time*_frequency);
        }

        return do_step(_setted_pos);
    }
