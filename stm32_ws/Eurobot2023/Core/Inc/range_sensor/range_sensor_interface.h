/*
 * range_sensor_interface.h
 *
 *  Created on: Feb 23, 2023
 *      Author: Valery_Danilov
 *      Editor: Maxim Popov
 */

#ifndef INC_RANGE_SENSOR_RANGE_SENSOR_INTERFACE_H_
#define INC_RANGE_SENSOR_RANGE_SENSOR_INTERFACE_H_

#include <cstdlib>
#include <vector>
#include <initializer_list>
#include <string>

#include "stm32f4xx_hal.h"

#include "ros.h"
#include "std_msgs/UInt16MultiArray.h"

#include "range_sensor_description.h"


namespace rs_interface {
    class VL53L0X_Interface{
        public:
    		VL53L0X_Interface (
				std::initializer_list<rs_description::VL53L0X_sensor> sensors_list,
				ros::NodeHandle& node,
				const char* scan_topic_name
            ) :
				_sensors (sensors_list),
				_node(node),
            	_scan_publisher(scan_topic_name, &_scan),
				restart_measurement(true)
			{}

            bool init(I2C_HandleTypeDef *hi2c) {
            	if (_node.advertise(_scan_publisher) == false) {
            		_node.logwarn("Node advertise error");
            	}

            	bool error = false;

            	for (size_t i = 0; i < _sensors.size(); i++) {
            		VL53L0X_Error status = _sensors[i].init(hi2c, i);

            		if (status) {
            			error = true;

            			_node.logwarn(
            				(
								"Range sensor "
								+ std::to_string(i)
								+ " init error: number "
								+ std::to_string(status)
							).data()
						);
            		}
            	}

            	return error;
            }

            bool start_measurement() {
            	bool error = false;

            	for (size_t i = 0; i < _sensors.size(); i++) {
            		if (_sensors[i].start_measurement() == true) {
            			error = true;

            			_node.logwarn (
            				("Range sensors start_measurement error: number " + std::to_string(i)).data()
						);
            		}
            	}

            	return error;
            }

            std::vector<uint16_t> get_data() {
            	std::vector<uint16_t> ranges;

            	for (size_t i = 0; i < _sensors.size(); i++) {
            		VL53L0X_RangingMeasurementData_t data;

            		if (_sensors[i].get_data(data) == true) {
//            			_node.logwarn (
//            					("Range sensors get_data error: number " + std::to_string(i)).data()
//						);

            			return {};
            		} else if (data.RangeMilliMeter >= data.RangeDMaxMilliMeter || data.RangeStatus != 0) {
            			ranges.push_back(-1);
            		} else {
            			ranges.push_back(data.RangeMilliMeter);
            		}
            	}

            	return ranges;
            }

            void publish() {
            	if (restart_measurement == true) {
            		restart_measurement = start_measurement();

            		return;
            	}

            	auto ranges = get_data();

            	if (ranges.size() != 0) {
            		restart_measurement = start_measurement();

            		_scan.data = ranges.data();
            		_scan.data_length = ranges.size();

            		_scan_publisher.publish(&_scan);
            	}
            }

        private:
            void _disable_all();
            void _enable_all();

            std::vector<rs_description::VL53L0X_sensor> _sensors;

            ros::NodeHandle& _node;
            ros::Publisher _scan_publisher;

            std_msgs::UInt16MultiArray _scan;

            bool restart_measurement;
    };
}

#endif /* INC_RANGE_SENSOR_RANGE_SENSOR_INTERFACE_H_ */
