/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "rf24.h"
#include "rf24_debug.h"

#include "radio_interface.h"
#include "inverse_kinematics.h"
#include "motor_activation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SERIAL_UART huart3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

command_t *p_cmd;
PID_TypeDef *p_PID;
rf24_dev_t *p_dev;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int __io_putchar(int ch);

int __io_getchar(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_SPI1_Init();
	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	printf("liguei\r\n");

	motor_init(htim1, MOTOR_TIMER);

	rf24_dev_t device; /* Module instance */
	p_dev = &device; /* Pointer to module instance */

	/* Device config */

	/* Get default configuration */
	rf24_get_default_config(p_dev);

	/* The SPI2 was chosen in Cube */
	p_dev->platform_setup.hspi = &hspi1;

	/* CSN on pin PC6 */
	p_dev->platform_setup.csn_port = RADIO_CSN_GPIO_Port;
	p_dev->platform_setup.csn_pin = RADIO_CSN_Pin;

	/* IRQ on pin PC7 */
	p_dev->platform_setup.irq_port = RADIO_IRQ_GPIO_Port;
	p_dev->platform_setup.irq_pin = RADIO_IRQ_Pin;

	/* CE on pin PC8 */
	p_dev->platform_setup.ce_port = RADIO_CE_GPIO_Port;
	p_dev->platform_setup.ce_pin = RADIO_CE_Pin;

	p_dev->payload_size = PAYLOAD_SIZE;

	rf24_init(p_dev);

	uint8_t addresses[2][5] = { { 0xB9, 0xB7, 0xE7, 0xE9, 0xC2 }, { 0xB9, 0xB7,
			0xE7, 0xE9, 0xC3 } };

	rf24_open_writing_pipe(p_dev, addresses[1]);
	rf24_open_reading_pipe(p_dev, 1, addresses[0]);

	rf24_start_listening(p_dev);
	//HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	command_t cmd;
	cmd.vx = 0;
	cmd.vy = 0;
	cmd.vtheta = 0;
	p_cmd = &cmd;

	PID_TypeDef TPIDVTheta;
	p_PID = &TPIDVTheta;

	inverse_kinematics_init();
	float *inverse_kinematics_result;

	uint8_t buffer[PAYLOAD_SIZE];

	rf24_status_t read_status;

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		read_status = RF24_UNKNOWN_ERROR;

		if ((rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
			while ((rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
				read_status = rf24_read(p_dev, buffer, p_dev->payload_size);
			}
		}

		if (read_status == RF24_SUCCESS)
			get_command_from_buffer(buffer, p_cmd, p_PID);

		/*
		 printf("[%d, %f, %f, %f, %d, %f, %f, %f]\r\n", buffer[0], cmd->vx,
		 cmd->vy, cmd->vtheta, buffer[13], uPID->Kp, uPID->Ki, uPID->Kd);
		 * */
		printf("vx: %f\r\n", p_cmd->vx);

		//calculate_wheel_speed(p_cmd, 0.0);
		/*]
		 printf("[%f, %f, %f, %f]\r\n", inverse_kinematics_result[0],
		 inverse_kinematics_result[1], inverse_kinematics_result[2],
		 inverse_kinematics_result[3]);
		 * */

		//write_speed_to_motors(MOTOR_TIMER, inverse_kinematics_result);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

int __io_getchar(void) {
	uint8_t ch = 0;

	/* Clear the Overrun flag just before receiving the first character */
	__HAL_UART_CLEAR_OREFLAG(&SERIAL_UART);

	/* Wait for reception of a character on the USART RX line and echo this
	 * character on console */
	HAL_UART_Receive(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/*
	 * */
	if (htim->Instance == TIM2) {

	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
