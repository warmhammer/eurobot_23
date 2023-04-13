/*
 * pca9685.h
 *
 *  Created on: 20.01.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/Servos_PWM_STM32_HAL
 */

#ifndef PCA9685_H_
#define PCA9685_H_

#include "stm32f4xx_hal.h"

//
//	Enable Servo control mode
//
#define PCA9685_SERVO_MODE

//
//	Adjustable address 0x80 - 0xFE
//
#define PCA9685_ADDRESS 0x80

//
//	Registers
//
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 		0x0
#define PCA9685_PRESCALE 	0xFE

#define PCA9685_LED0_ON_L 	0x6
#define PCA9685_LED0_ON_H	0x7
#define PCA9685_LED0_OFF_L	0x8
#define PCA9685_LED0_OFF_H 	0x9

#define PCA9685_ALLLED_ON_L 	0xFA
#define PCA9685_ALLLED_ON_H 	0xFB
#define PCA9685_ALLLED_OFF_L 	0xFC
#define PCA9685_ALLLED_OFF_H 	0xFD

#define PCA9685_MODE1_ALCALL_BIT	0
typedef enum
{
	PCA9685_MODE1_SUB1_BIT 	= 3,
	PCA9685_MODE1_SUB2_BIT	= 2,
	PCA9685_MODE1_SUB3_BIT	= 1
}SubaddressBit;
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7


typedef enum
{
	PCA9685_OK 		= 0,
	PCA9685_ERROR	= 1
} PCA9685_STATUS;

PCA9685_STATUS PCA9685_SoftwareReset(void);
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable);
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable);

PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency);

PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert);

PCA9685_STATUS PCA9685_Init(I2C_HandleTypeDef *hi2c);

#endif /* PCA9685_H_ */
