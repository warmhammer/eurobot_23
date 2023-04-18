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

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "wrappers.h"
namespace VL53L0X_sensor {
    class Range_Sensor_Interface{
        public:
            Range_Sensor_Interface(
                    std::initializer_list<wrappers::pin_wrapper> XSHUT_Pins_list,
                    ros::NodeHandle& node,
                    const char* range_sensors_topic_name
            );
            void update();
            unsigned int get_dev_count();
            void init(I2C_HandleTypeDef *hi2c);
            VL53L0X_DEV get_dev(unsigned int dev_index);
            void start_range();
            VL53L0X_DeviceError get_error();
            void calibrate();

        private:
            void _disable_all();
            void _enable_all();
            //void _write(const std_msgs::Float32MultiArray& msg);
            //sensor_msgs::Range _range;

            std::vector<VL53L0X_Dev_t> _sensors;
            std::vector<wrappers::pin_wrapper> _XSHUT_Pins;

            //ros::Publisher _ranges_publisher;

            ros::NodeHandle& _node;

            //ros::Subscriber<> _servo_cmd_subscriber;
    };
}
#endif /* INC_RANGE_SENSOR_INTERFACE_H_ */
