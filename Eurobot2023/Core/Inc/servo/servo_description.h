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

#include "pca9685.h"

namespace servo_description {
	class Servo {
		public:
			Servo(
				float angle_range,
				uint16_t pwm_min,
				uint16_t pwm_max,
				float min_angle,
				float max_angle,
				float default_angle,
				uint8_t pca_channel
			);

			void init();

			float get_min_angle() const;
			float get_max_angle() const;
			float get_current_angle() const;

			PCA9685_STATUS set_angle(float angle);

		private:
			const float _angle_range;

			const float _pwm_koef;
			const uint16_t _pwm_bias;

			const float _min_angle;
			const float _max_angle;
			const float _default_angle;
			float _current_angle;

			const uint8_t _pca_channel;
	};

//*************************************************
// SPECIFIC SERVOS

	class RDS3225_Servo : public Servo {
		public:
			RDS3225_Servo (
				uint8_t pca_channel,
				float default_angle = 135,
				float min_angle = 5,
				float max_angle = 265
			);
	};

	class PDI_6225MG_300_Servo : public Servo {
		public:
			PDI_6225MG_300_Servo (
				uint8_t pca_channel,
				float default_angle = 150,
				float min_angle = 5,
				float max_angle = 295
			);
	};
}

#endif /* INC_SERVO_DESCRIPTION_H_ */
