/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define XSHUT_1_Pin GPIO_PIN_1
#define XSHUT_1_GPIO_Port GPIOC
#define XSHUT_2_Pin GPIO_PIN_2
#define XSHUT_2_GPIO_Port GPIOC
#define XSHUT_3_Pin GPIO_PIN_3
#define XSHUT_3_GPIO_Port GPIOC
#define XSHUT_4_Pin GPIO_PIN_7
#define XSHUT_4_GPIO_Port GPIOE
#define XSHUT_5_Pin GPIO_PIN_9
#define XSHUT_5_GPIO_Port GPIOE
#define XSHUT_6_Pin GPIO_PIN_11
#define XSHUT_6_GPIO_Port GPIOE
#define XSHUT_7_Pin GPIO_PIN_14
#define XSHUT_7_GPIO_Port GPIOE
#define DIR_L_Pin GPIO_PIN_8
#define DIR_L_GPIO_Port GPIOA
#define DIR_R_Pin GPIO_PIN_9
#define DIR_R_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_10
#define ENA_GPIO_Port GPIOA
#define START_BUTTON_Pin GPIO_PIN_0
#define START_BUTTON_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
