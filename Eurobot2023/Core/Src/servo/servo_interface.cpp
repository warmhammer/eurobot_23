/*
 * servo_interface.cpp
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery_Danilov
 *      Edited: Maxim_Popov
 */

#include "servo/servo_interface.h"
#include "servo/pca9685.h"

#include <string>

namespace servo_interface {
	Servo_Interface::Servo_Interface (
		std::initializer_list<servo_description::Servo> servos_list,
		ros::NodeHandle& node,
		const char* positions_topic
	) :
		_servos(servos_list),
		_node(node),
		_pos_topic_subscriber(positions_topic, [&, this](const std_msgs::Float32MultiArray& msg){this->_write(msg);})
	{
		if (_servos.size() == 0) {
			_node.logwarn("Servo_Interface: no servos were passed");
		}
	}

	void Servo_Interface::init(I2C_HandleTypeDef *hi2c) {
		_node.subscribe(_pos_topic_subscriber);

		PCA9685_Init(hi2c);

		for (auto& servo : _servos) {
			servo.init();
		}
	}

	bool Servo_Interface::_write(const std_msgs::Float32MultiArray& msg){
		if (msg.data_length != _servos.size()) {
			_node.logwarn("Servo_Interface: incorrect size of servos' positions MultiArray");

			return true;
		}

		bool return_flag = false;

		for (uint8_t i = 0; i < _servos.size(); i++) {
			float angle = msg.data[i];

			if (_servos[i].set_angle(angle) != PCA9685_OK) {
				_node.logwarn("Servo_Interface: pca9685 error");
				return_flag = true;
			}
		}

		return return_flag;
	}
}
