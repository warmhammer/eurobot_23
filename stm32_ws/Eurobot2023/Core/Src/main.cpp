/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ros.h"

#include "motors.h"
#include "wrappers.h"
#include "servo/servo_interface.h"
#include "range_sensor/range_sensor_interface.h"

#include "utils/start_button.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//------------------------------------------------define EncoderMotors perif BEGIN--------------------
#define dir_port_l 				GPIOA
#define dir_pin_l 				GPIO_PIN_8

#define ena_port  				GPIOA
#define ena_pin 				GPIO_PIN_10

#define	encoder_timer_l 		&htim2
#define encoder_timer_chanel1_l TIM_CHANNEL_3

#define speed_timer_l 			&htim3
#define speed_timer_chanel2_l  	TIM_CHANNEL_3

#define pwm_timer_l				&htim9
#define pwm_timer_chanel1_l  	TIM_CHANNEL_1

#define dir_port_r 				GPIOA
#define dir_pin_r				GPIO_PIN_9

#define	encoder_timer_r 		&htim5
#define encoder_timer_chanel1_r TIM_CHANNEL_2

#define speed_timer_r 			&htim4
#define speed_timer_chanel2_r  	TIM_CHANNEL_2

#define pwm_timer_r				&htim12
#define pwm_timer_chanel1_r  	TIM_CHANNEL_1

//------------------------------------------------define EncoderMotors perif END--------------------

//------------------------------------------------define Range_Sensors perif BEGIN------------------

//------------------------------------------------define Range_Sensors perif END--------------------



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----------------------------------------------------------------ROS------------------------------------

ros::NodeHandle node;

//-------------------------------------------------------------------------------------------------------

//------------------------------------------------------------GLOBAL OBJ---------------------------------

motors::EncoderMotor left_encoder_motor (
    {dir_port_l, dir_pin_l},
    {ena_port, ena_pin},
    {encoder_timer_l, encoder_timer_chanel1_l},
    {speed_timer_l, speed_timer_chanel2_l},
    {pwm_timer_l, pwm_timer_chanel1_l},
	false,
	node,
	"/dolly/left_wheel/angle32",
	"/dolly/left_wheel/cur_vel32",
	"/dolly/left_wheel/pwd32"
);

motors::EncoderMotor right_encoder_motor (
    {dir_port_r, dir_pin_r},
    {ena_port, ena_pin},
    {encoder_timer_r, encoder_timer_chanel1_r},
    {speed_timer_r, speed_timer_chanel2_r},
    {pwm_timer_r, pwm_timer_chanel1_r},
//	true,
	false,
	node,
	"/dolly/right_wheel/angle32",
	"/dolly/right_wheel/cur_vel32",
	"/dolly/right_wheel/pwd32"
);

//-----------------------------------------------------------Servos------------------------------

// ****** constexpr is used here to avoid logic error at compile time
constexpr servo_description::RDS3225_Servo left_gripper(0, 60, 130);
constexpr servo_description::RDS3225_Servo right_gripper(1, 55, 125);
constexpr servo_description::RDS3225_Servo lift(2, 15, 210);
constexpr servo_description::RDS3225_Servo plunger(3, 95, 150);

constexpr servo_description::PDI_6225MG_300_Servo left_limiter(4); // 55, 105
constexpr servo_description::PDI_6225MG_300_Servo right_limiter(5); // 180, 230
constexpr servo_description::PDI_6225MG_300_Servo cherry_spreader(6, 10, 285);
constexpr servo_description::PDI_6225MG_300_Servo cherry_separator(7, 115, 240);
constexpr servo_description::PDI_6225MG_300_Servo visor(8, 10, 235);

servo_interface::Servo_Interface servos(
	{
		left_gripper,
		right_gripper,
		lift,
		plunger,

		left_limiter,
		right_limiter,
		cherry_spreader,
		cherry_separator,
		visor

	},
	node,
	"servo_cmd_topic"
);

//-----------------------------------------------------------Range_Sensors-----------------------

rs_interface::VL53L0X_Interface range_sensors (
	{
		{{XSHUT_1_GPIO_Port, XSHUT_1_Pin}},
		{{XSHUT_2_GPIO_Port, XSHUT_2_Pin}},
		{{XSHUT_3_GPIO_Port, XSHUT_3_Pin}},
		{{XSHUT_4_GPIO_Port, XSHUT_4_Pin}},
		{{XSHUT_5_GPIO_Port, XSHUT_5_Pin}},
		{{XSHUT_6_GPIO_Port, XSHUT_6_Pin}},
		{{XSHUT_7_GPIO_Port, XSHUT_7_Pin}}
	},
	node,
	"range_sensors_topic"
);

//-----------------------------------------------------------Utils-------------------------------

utils::StartButton start_button(
	{START_BUTTON_GPIO_Port, START_BUTTON_Pin},
	node,
	"start_topic"
);

//------------------------------------------------------------SYSTEM UART func-------------------

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){

    uint8_t data[1];
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);

  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  //The most important thing when UART framing error occur/any error is restart the RX process
  //Restarting the RX, .. 1 byte. .. u8DATUartShortRxBuffer is My own rx buffer
    HAL_UART_Receive_IT(huart, data, 1);
}

void UART_check(UART_HandleTypeDef *huart){
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET){
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_ENABLE_IT(huart,UART_IT_ERR);

		if (__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) == SET){
			huart->Instance->CR3 |= 1 << 6;
			huart->Instance->CR3 |= 1 << 7;
		}
	}
}

//------------------------------------------------------------SYSTEM Transmit CallBack's-------------------

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    node.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    node.getHardware()->reset_rbuf();
}

//------------------------------------------------------------TIM SYSTEM Motors CallBack's-------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	left_encoder_motor.__set_velocity_to_null__(htim);
	right_encoder_motor.__set_velocity_to_null__(htim);
}

//--------------------------------------------------------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    //-------------------------------------------------------------ROS----------------------

    node.initNode();

    while (node.connected() == false) {
        // waiting for connection
        UART_check(&huart2);
    	HAL_Delay(1);
    	node.spinOnce();
    }
   //-----------------------------------------------------------ROS::Init_begin------------

    left_encoder_motor.init();
    right_encoder_motor.init();

//    servos.init(&hi2c1);

//    range_sensors.init(&hi2c2);

//    start_button.init();

//    HAL_Delay(3000);

   //-----------------------------------------------------------ROS::Init_end--------------
    node.getHardware()->flush();	// buffer flush

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    auto prev = HAL_GetTick();

    while (1) {
        if (node.connected() == true) {
            auto now = HAL_GetTick();

            if (now - prev >= 15) {
                prev = now;

                right_encoder_motor.publish();
                left_encoder_motor.publish();

//                range_sensors.publish();

//                start_button.publish();

            }
        }

        UART_check(&huart2);

        HAL_Delay(1);
        node.spinOnce();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
