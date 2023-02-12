/*
 * motors.cpp
 *
 *  Created on: Jan 27, 2023
 *      Author: Valery_Danilov
 *      Editor: Maxim Popov
 */
// This trajectory generator has uint16_t input and uint16_t output position of control object.
// When the command arrives, generator's input is blocked, generator is not controlled.
// Generator input unblocked after execution of previous command.
#ifndef _SERVO_TRAJECTORY_GENERANOR_H
#define _SERVO_TRAJECTORY_GENERANOR_H

#include "stm32f4xx_hal.h"

constexpr int a3 = 10;
constexpr int a4 = -15;
constexpr int a5 = 6;

    class Minimum_Jerk_Generator {
        public:
            Minimum_Jerk_Generator (
                    uint32_t frequency,
                    uint16_t current_pos
                    );
        public:
            void init();
            uint16_t set_pos(uint16_t position);


        private:
            uint16_t do_step(uint16_t position);

            float _travel_time;
            uint16_t _sample_count;   // count of dots on trajectory (linear approximation)
            uint16_t _current_sample;
            uint32_t _frequency;
            uint16_t _current_pos;
            uint16_t _setted_pos;
            bool _state;            // 1 - is BUSY      0 - READY
            bool _error;            // 1 - is ERROR     0 - OK
};

#endif
