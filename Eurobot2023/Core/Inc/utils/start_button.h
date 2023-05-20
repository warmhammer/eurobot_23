/*
 * start_button.h
 *
 *  Created on: May 20, 2023
 *      Author: Maxim Popov
 */

#ifndef INC_UTILS_START_BUTTON_H_
#define INC_UTILS_START_BUTTON_H_

#include "stm32f4xx_hal.h"

#include "ros.h"
#include "std_msgs/Bool.h"

#include "wrappers.h"


namespace utils {
	class StartButton {
		public:
			StartButton (
				wrappers::pin_wrapper button_pin,
				ros::NodeHandle& node,
				const char* start_topic
			) :
				_button_pin(button_pin),
				_node(node),
				_start_publisher(start_topic, &is_started)
			{}

			void init() {
				if (_node.advertise(_start_publisher) == false) {
					_node.logwarn("Node advertise error");
				}

				is_started.data = false;
			}

			void publish() {
//				if (is_started.data == false && HAL_GPIO_ReadPin(_button_pin.port, _button_pin.pin) == GPIO_PIN_SET) {
//					is_started.data = true;
//				}

				is_started.data = HAL_GPIO_ReadPin(_button_pin.port, _button_pin.pin) == GPIO_PIN_RESET;

				_start_publisher.publish(&is_started);
			}

		private:
			wrappers::pin_wrapper _button_pin;
			ros::NodeHandle& _node;
			std_msgs::Bool is_started;

			ros::Publisher _start_publisher;

	};
}


#endif /* INC_UTILS_START_BUTTON_H_ */
