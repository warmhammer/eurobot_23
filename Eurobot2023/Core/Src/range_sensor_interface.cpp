/*
 * range_sensor_interface.cpp
 *
 *  Created on: Feb 23, 2023
 *      Author: Valery_Danilov
 */
#include "stm32f4xx_hal.h"
#include "range_sensor_interface.h"

VL53L0X_RangingMeasurementData_t pRangingMeasurementData[6];

namespace VL53L0X_sensor {

    Range_Sensor_Interface::Range_Sensor_Interface(
                    std::initializer_list<wrappers::pin_wrapper> XSHUT_Pins_list,
                    ros::NodeHandle& node,
                    const char* range_sensors_topic_name
                    ) :
                    _XSHUT_Pins(XSHUT_Pins_list),
                    _node(node)
                    //_ranges_publisher(range_sensors_topic_name,&_range)
                    {
                        for (int i = 0; i < XSHUT_Pins_list.size(); i++){
                            _sensors.push_back({});
                        }
                    }

    void Range_Sensor_Interface::init(I2C_HandleTypeDef *hi2c){

        volatile VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
        uint32_t refSpadCount = 0 ;
        uint8_t isApertureSpads = 0;
        uint8_t pVhvSettings = 0;
        uint8_t pPhaseCal = 0;
        auto sensor_count = _sensors.size();
        uint8_t Dev_adress = 0;
        int32_t Offset_Dist = 0;


        _disable_all();

        for (int i = 0; i < sensor_count; i++){

            HAL_GPIO_WritePin(_XSHUT_Pins[i].port, _XSHUT_Pins[i].pin, GPIO_PIN_SET);
            HAL_Delay(3);                                                                    //BOOT_Delay
            _sensors[i].I2cHandle = hi2c;
            _sensors[i].I2cDevAddr = 0x52;

            Status = VL53L0X_DataInit(&_sensors[i]);                                                  //TODO return status
            Status = VL53L0X_StaticInit(&_sensors[i]);

            //Calibration algorithm BEGIN
            Status = VL53L0X_PerformRefSpadManagement(&_sensors[i], &refSpadCount, &isApertureSpads); //TODO Remove calibration algorithm
            Status = VL53L0X_PerformRefCalibration(&_sensors[i], &pVhvSettings, &pPhaseCal);
            //set white target and
            //do
            //VL53L0X_PerformOffsetCalibration(Dev, CalDistanceMilliMeter, pOffsetMicroMeter)
            VL53L0X_SetOffsetCalibrationDataMicroMeter(&_sensors[i], Offset_Dist);
            //set gray target and
            //VL53L0X_PerformXTalkCalibration(Dev, XTalkCalDistance, pXTalkCompensationRateMegaCps)

            //Calibration algorithm END

           Status = VL53L0X_SetDeviceAddress(&_sensors[i], 0x52 + 4*i + 2);

           _sensors[i].I2cDevAddr = 0x52 + 4*i + 2;

           //Status = VL53L0X_RdByte(&_sensors[i], VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, &Dev_adress); //read Dev i2c Address

           Status = VL53L0X_SetDeviceMode(&_sensors[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);
           Status = VL53L0X_SetGpioConfig(&_sensors[i], 0, VL53L0X_DEVICEMODE_SINGLE_RANGING, VL53L0X_GPIOFUNCTIONALITY_OFF, VL53L0X_INTERRUPTPOLARITY_LOW);
        }
        //_enable_all();
        Status = get_error();
        int k = 5;
}

    unsigned int Range_Sensor_Interface::get_dev_count(){
        return _sensors.size();
    }
    VL53L0X_DEV Range_Sensor_Interface::get_dev(unsigned int dev_index){

        return &_sensors[dev_index];
    }
    void Range_Sensor_Interface::start_range(){
        uint8_t pMeasurementDataReady = 0;
        auto sensors_count = _sensors.size();
        volatile VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;

        while(1){

        for (int i = 0; i < sensors_count; i++){
           Status = VL53L0X_StartMeasurement(&_sensors[i]); //TODO return status of starting
        }
        HAL_Delay(60);

        for (int i = 0; i < _sensors.size(); i++){
        VL53L0X_GetMeasurementDataReady(&_sensors[i], &pMeasurementDataReady);

        if(pMeasurementDataReady == 1){
            VL53L0X_GetRangingMeasurementData(&_sensors[i], &pRangingMeasurementData[i]);
            VL53L0X_ClearInterruptMask(&_sensors[i], 1);
            }
        }

       }
    }
    void Range_Sensor_Interface::_disable_all(){
        for (int i = 0; i < _XSHUT_Pins.size(); i++){
            HAL_GPIO_WritePin(_XSHUT_Pins[i].port, _XSHUT_Pins[i].pin, GPIO_PIN_RESET);
        }
    }
    void Range_Sensor_Interface::_enable_all(){
        for (int i = 0; i < _XSHUT_Pins.size(); i++){
            HAL_GPIO_WritePin(_XSHUT_Pins[i].port, _XSHUT_Pins[i].pin, GPIO_PIN_SET);
        }
    }
    VL53L0X_DeviceError Range_Sensor_Interface::get_error(){
        VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
        auto sensor_count = _sensors.size();

        for (int i = 0; i < sensor_count; i++){
           VL53L0X_GetDeviceErrorStatus(&_sensors[i], &Status);

           if (Status != VL53L0X_ERROR_NONE)
           {
               _node.logwarn("Range sensors ERROR occurs"); //TODO return index of error sensor
               return Status;
           }
        }
        return Status;
    }
    void Range_Sensor_Interface::calibrate(){

    }
}

