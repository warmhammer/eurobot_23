/*
 * servo_description.cpp
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery Danilov
 *      Edited: Maxim Popov
 */

#include "servo_description.h"

#include <cassert>

namespace servo_description {
	Servo::Servo (
		float angle_range,
		uint16_t pwm_min,
		uint16_t pwm_max,
		float min_angle,
		float max_angle,
		float default_angle,
		uint8_t pca_channel
	) :
		_angle_range(angle_range),
		_min_angle(min_angle),
		_max_angle(max_angle),
		_default_angle(default_angle),
		_current_angle(default_angle),
		_pca_channel(pca_channel)
	{
		assert(0 <= min_angle && max_angle <= angle_range);
		assert(min_angle <= default_angle && default_angle <= max_angle);

		assert(pwm_min < pwm_max);

		_pwm_koef = (pwm_max - pwm_min) / (max_angle - min_angle);
		_pwm_bias = pwm_min;
	}

	void Servo::init() {
		set_angle(_default_angle);
	}

	float Servo::get_min_angle() const {
		return _min_angle;
	}

	float Servo::get_max_angle() const {
		return _max_angle;
	}

	float Servo::get_current_angle() const {
		return _current_angle;
	}

	PCA9685_STATUS Servo::set_angle(float angle) {
		if(angle < _min_angle) {
			angle = _min_angle;
		} else if(angle > _max_angle) {
			angle = _max_angle;
		}

		_current_angle = angle;

		uint16_t pwm_value = static_cast<uint16_t>((angle - _min_angle) * _pwm_koef) + _pwm_bias;

		return PCA9685_SetPin(_pca_channel, pwm_value, 0);
	}

//*************************************************
// SPECIFIC SERVOS

	RDS3225_Servo::RDS3225_Servo (
		uint8_t pca_channel,
		float default_angle,
		float min_angle,
		float max_angle
	) :
		Servo (
			270,
			78,
			504,
			min_angle,
			max_angle,
			default_angle,
			pca_channel
		)
    {}

	PDI_6225MG_300_Servo::PDI_6225MG_300_Servo (
		uint8_t pca_channel,
		float default_angle,
		float min_angle,
		float max_angle
	) :
		Servo (
			300,
			88,
			498,
			min_angle,
			max_angle,
			default_angle,
			pca_channel
		)
	{}
}
