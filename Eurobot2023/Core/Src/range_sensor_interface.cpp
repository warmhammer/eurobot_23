/*
 * range_sensor_interface.cpp
 *
 *  Created on: Feb 23, 2023
 *      Author: Valery_Danilov
 */

#include "range_sensor_interface.h"

Range_Sensor_Interface::Range_Sensor_Interface(
                    std::initializer_list<VL53L0X_Dev_t> sensors_params_list,
                    I2C_HandleTypeDef *hi2c,
                    ros::NodeHandle& node,
                    //const char* sensors_state_topic_name,
                    const char* range_sensors_topic_name
                    ){

                    }

void Range_Sensor_Interface::Init(I2C_HandleTypeDef *hi2c){

}
