/*
 * range_sensor_interface.cpp
 *
 *  Created on: Feb 23, 2023
 *      Author: Valery_Danilov
 */

#include "range_sensor_interface.h"

Range_Sensor_Interface::Range_Sensor_Interface(
                    ros::NodeHandle& node,
                    //const char* sensors_state_topic_name,
                    const char* range_sensors_topic_name,
                    const unsigned int dev_count
                    ) :
                    _node(node)
                    //_ranges_publisher(range_sensors_topic_name,&_range)
                    {
                        for (int i = 0; i < dev_count; i++){
                            _sensors.push_back({});
                        }
                    }

    void Range_Sensor_Interface::Init(I2C_HandleTypeDef *hi2c){
        for (int i = 0; i < _sensors.size(); i++){

            _sensors[i].I2cHandle = hi2c;
            _sensors[i].I2cDevAddr = 0x52; //TODO change

            VL53L0X_DataInit(&_sensors[i]); //TODO return status
            VL53L0X_StaticInit(&_sensors[i]);

                //TODO Remove calibration algoritm

               //Calibration algorithm BEGIN
               //VL53L0X_PerformRefSpadManagement(&_sensors[i], refSpadCount, isApertureSpads)
               //VL53L0X_PerformRefCalibration(Dev, pVhvSettings, pPhaseCal)
               //set white target and
               //do
               //VL53L0X_PerformOffsetCalibration(Dev, CalDistanceMilliMeter, pOffsetMicroMeter)
               //set gray target and
               //VL53L0X_PerformXTalkCalibration(Dev, XTalkCalDistance, pXTalkCompensationRateMegaCps)
               //Calibration algorithm END

           VL53L0X_SetDeviceMode(&_sensors[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);
           //VL53L0X_SetInterruptThresholds()
           VL53L0X_SetGpioConfig(&_sensors[i], 0, VL53L0X_DEVICEMODE_SINGLE_RANGING, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_LOW);
           VL53L0X_StartMeasurement(&_sensors[i]);
        }

}
    unsigned int Range_Sensor_Interface::get_dev_count(){
        return _sensors.size();
    }
    VL53L0X_DEV Range_Sensor_Interface::get_dev(unsigned int dev_index){

        if (dev_index < _sensors.size()-1 || dev_index < 0 || dev_index == 0){

            return 0; //TODO do this more clearly

        }else{
            return &_sensors[dev_index];
        }
    }

