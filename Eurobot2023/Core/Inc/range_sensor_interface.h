/*
 * range_sensor_interface.h
 *
 *  Created on: Feb 23, 2023
 *      Author: Valery_Danilov
 */

#ifndef INC_RANGE_SENSOR_INTERFACE_H_
#define INC_RANGE_SENSOR_INTERFACE_H_

#include <stdlib.h>
#include <vector>
#include <initializer_list>

#include "stm32f4xx_hal.h"

#include "ros.h"
#include "sensor_msgs/Range.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

    class Range_Sensor_Interface{
        public:
            Range_Sensor_Interface(
                    std::initializer_list<VL53L0X_Dev_t> sensors_params_list,
                    I2C_HandleTypeDef *hi2c,
                    ros::NodeHandle& node,
                    //const char* sensors_state_topic_name,
                    const char* range_sensors_topic_name
            );
            void update();
            void Init(I2C_HandleTypeDef *hi2c);

        private:
            //void _write(const std_msgs::Float32MultiArray& msg);

            std::vector<VL53L0X_Dev_t> _sensors;

            ros::Publisher _ranges_publisher;

            ros::NodeHandle& _node;

            //ros::Subscriber<> _servo_cmd_subscriber;
    };

#endif /* INC_RANGE_SENSOR_INTERFACE_H_ */
