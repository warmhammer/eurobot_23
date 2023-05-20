/*
 * range_sensor_description.h
 *
 *  Created on: May 18, 2023
 *      Author: Maxim Popov
 */

#ifndef INC_RANGE_SENSOR_RANGE_SENSOR_DESCRIPTION_H_
#define INC_RANGE_SENSOR_RANGE_SENSOR_DESCRIPTION_H_

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "wrappers.h"

namespace rs_description {
	class VL53L0X_sensor{
		public:
			VL53L0X_sensor (wrappers::pin_wrapper xshut_pin)
			:
				_xshut_pin(xshut_pin),
				_device()
			{
				_disable();
			}

			VL53L0X_Error init(I2C_HandleTypeDef *hi2c, uint8_t device_num) {
				_enable();

				HAL_Delay(50);	// boot delay

				_device.I2cHandle = hi2c;
				_device.I2cDevAddr = 0x52;

				VL53L0X_Error error = 0;


				error = VL53L0X_DataInit(&_device);
				if (error) {
					return error;
				}

				error = VL53L0X_StaticInit(&_device);
				if(error) {
					return error;
				}

				error = VL53L0X_StaticInit(&_device);
				if (error) {
					return error;
				}

				error = _calibrate();
				if (error) {
					return error;
				}

				error = VL53L0X_SetDeviceAddress(&_device, 0x52 + 4 * device_num + 2);
				if (error) {
					return error;
				}

				_device.I2cDevAddr = 0x52 + 4 * device_num + 2;

				if (
						VL53L0X_SetDeviceMode(&_device, VL53L0X_DEVICEMODE_SINGLE_RANGING) != VL53L0X_ERROR_NONE 	||
						VL53L0X_SetGpioConfig (
							&_device,
							0,
							VL53L0X_DEVICEMODE_SINGLE_RANGING,
							VL53L0X_GPIOFUNCTIONALITY_OFF,
							VL53L0X_INTERRUPTPOLARITY_LOW
						) != VL53L0X_ERROR_NONE
				) {
					return true;
				}

				return false;
			}

			bool start_measurement() {
				return VL53L0X_StartMeasurement(&_device) != VL53L0X_ERROR_NONE;
			}

			bool get_data(VL53L0X_RangingMeasurementData_t& data) {
				bool status = 0;
				VL53L0X_GetMeasurementDataReady(&_device, reinterpret_cast<uint8_t*>(&status));

				if(status == true){
					VL53L0X_GetRangingMeasurementData(&_device, &data);
					VL53L0X_ClearInterruptMask(&_device, 1);
				}

				return !status;
			}

			VL53L0X_DeviceError get_error() {
				VL53L0X_DeviceError error = 0;

				VL53L0X_GetDeviceErrorStatus(&_device, &error);

				return error;
			}

		private:
			void _disable() {
				HAL_GPIO_WritePin(_xshut_pin.port, _xshut_pin.pin, GPIO_PIN_RESET);
			}

			void _enable() {
				HAL_GPIO_WritePin(_xshut_pin.port, _xshut_pin.pin, GPIO_PIN_SET);
			}

			bool _calibrate() {
				bool error = false;

				uint32_t refSpadCount = 0 ;
				uint8_t isApertureSpads = 0;
				uint8_t pVhvSettings = 0;
				uint8_t pPhaseCal = 0;

				error = VL53L0X_PerformRefSpadManagement(&_device, &refSpadCount, &isApertureSpads) != VL53L0X_ERROR_NONE; //TODO Remove calibration algorithm
				error = VL53L0X_PerformRefCalibration(&_device, &pVhvSettings, &pPhaseCal) != VL53L0X_ERROR_NONE;

				int32_t Offset_Dist = 0;
				error = VL53L0X_SetOffsetCalibrationDataMicroMeter(&_device, Offset_Dist) != VL53L0X_ERROR_NONE;

				return error;
			}


			wrappers::pin_wrapper _xshut_pin;

			VL53L0X_Dev_t _device;
	};
}



#endif /* INC_RANGE_SENSOR_RANGE_SENSOR_DESCRIPTION_H_ */
