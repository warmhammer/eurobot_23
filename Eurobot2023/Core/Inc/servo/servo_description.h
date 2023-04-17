/*
 * servo_description.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery Danilov
 *      Edited: Maxim Popov
 */

#ifndef INC_SERVO_DESCRIPTION_H_
#define INC_SERVO_DESCRIPTION_H_

#include <cstdint>

#include <stdexcept>

#include "pca9685.h"

namespace servo_description {
	class Servo {
		public:
			constexpr Servo(
				uint16_t angle_range,
				uint16_t pwm_min,
				uint16_t pwm_max,
				uint16_t min_angle,
				uint16_t max_angle,
				uint8_t pca_channel
			) noexcept :
				_angle_range(angle_range),
				_pwm_koef(angle_range > 0 ? (pwm_max - pwm_min) / angle_range : 0),
				_pwm_bias(pwm_min),
				_min_angle(min_angle),
				_max_angle(max_angle),
				_current_angle(-1),
				_pca_channel(pca_channel)
			{
				if (min_angle < 0 || angle_range < max_angle || angle_range <= 0) {
					throw std::logic_error("Error: min_angle < 0 || angle_range < max_angle || angle_range <= 0");
				}

				if (max_angle <= min_angle) {
					throw std::logic_error("Error: max_angle <= min_angle");
				}

				if (pwm_max <= pwm_min) {
					throw std::logic_error("Error: pwm_max <= pwm_min");
				}

				if (_pwm_koef == 0) {
					throw std::logic_error("Error: _pwm_koef == 0");
				}
			};


			uint16_t get_min_angle() const {
				return _min_angle;
			}

			uint16_t get_max_angle() const {
				return _max_angle;
			}

			uint16_t get_current_angle() const {
				return _current_angle;
			}

			PCA9685_STATUS set_angle(float angle) {
				if(angle < _min_angle) {
					angle = _min_angle;
				} else if(angle > _max_angle) {
					angle = _max_angle;
				}

				_current_angle = angle;

				uint16_t pwm_value = static_cast<uint16_t>(angle * _pwm_koef) + _pwm_bias;

				return PCA9685_SetPin(_pca_channel, pwm_value, 0);
			}

		private:
			const uint16_t _angle_range;

			const uint16_t _pwm_koef;
			const uint16_t _pwm_bias;

			const uint16_t _min_angle;
			const uint16_t _max_angle;
			uint16_t _current_angle;

			const uint8_t _pca_channel;
	};

//*************************************************
// SPECIFIC SERVOS

	class RDS3225_Servo : public Servo {
		public:
			constexpr RDS3225_Servo (
				uint8_t pca_channel,
				uint16_t min_angle = 5,
				uint16_t max_angle = 265
			)  :
				Servo (
					270,
					78,
					504,
					min_angle,
					max_angle,
					pca_channel
				)
			{}
	};

	class PDI_6225MG_300_Servo : public Servo {
		public:
			 constexpr PDI_6225MG_300_Servo (
				uint8_t pca_channel,
				uint16_t min_angle = 5,
				uint16_t max_angle = 295
			)  :
				Servo (
					300,
					88,
					498,
					min_angle,
					max_angle,
					pca_channel
				)
			{}
	};
}

#endif /* INC_SERVO_DESCRIPTION_H_ */
