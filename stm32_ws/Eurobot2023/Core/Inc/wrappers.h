/*
 * timer_wrapper.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Maxim Popov
 */

#ifndef INC_WRAPPERS_H_
#define INC_WRAPPERS_H_

#include "stm32f4xx_hal.h"

namespace wrappers {
	struct timer_wrapper {
		TIM_HandleTypeDef* const tim;
		const uint16_t channel;
	};

	struct pin_wrapper {
		GPIO_TypeDef* const port;
		const uint16_t pin;
	};
}

#endif /* INC_WRAPPERS_H_ */
