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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>

//#include <string.h>

#include "rf24.h"
#include "linked_list.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERIAL_UART huart3

#define PAYLOAD_SIZE 26

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t address[5] = { 0xB9, 0xB7, 0xE7, 0xE9, 0xC2 };

rf24_dev_t device; /* Module instance */
rf24_dev_t *p_dev = &device; /* Pointer to module instance */

uint8_t rx_buffer[100];

uint8_t robot_count = 0;

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
	MX_USART3_UART_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	p_dev->platform_setup.spi_timeout = 1000;
	p_dev->payload_size = PAYLOAD_SIZE;
	p_dev->addr_width = 5;
	p_dev->datarate = RF24_2MBPS;
	p_dev->channel = 76;

	for (uint8_t i = 0; i < RF24_ADDRESS_MAX_SIZE; i++) {
		p_dev->pipe0_reading_address[i] = 0;
	}

	p_dev->platform_setup.hspi = &hspi1;

	p_dev->platform_setup.csn_port = RADIO_CSN_GPIO_Port;
	p_dev->platform_setup.csn_pin = RADIO_CSN_Pin;

	p_dev->platform_setup.irq_port = RADIO_IRQ_GPIO_Port;
	p_dev->platform_setup.irq_pin = RADIO_IRQ_Pin;

	p_dev->platform_setup.ce_port = RADIO_CE_GPIO_Port;
	p_dev->platform_setup.ce_pin = RADIO_CE_Pin;

	rf24_init(p_dev);

	rf24_set_output_power(p_dev, RF24_18_dBm);

	rf24_open_writing_pipe(p_dev, address);

	printf("Waiting for robot count...\r\n");

	while (robot_count == 0) {
		HAL_UART_Receive(&SERIAL_UART, &robot_count, 1, 100);
	}

	uint32_t elapsed_time = 0;
	uint32_t last_time = 0;
	uint32_t current_time = 0;

	uint8_t message_size = robot_count * PAYLOAD_SIZE;

	uint8_t **robots = (uint8_t**) malloc(robot_count * sizeof(uint8_t*));
	for (int i = 0; i < robot_count; i++) {
		robots[i] = (uint8_t*) malloc(PAYLOAD_SIZE * sizeof(uint8_t));
		for (int j = 0; j < PAYLOAD_SIZE; j++) {
			robots[i][j] = 0;
		}
	}

	char *ready_message = "R";

	HAL_StatusTypeDef read_status;

	printf("Starting loop\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_UART_Transmit(&SERIAL_UART, (uint8_t*) ready_message, 1, 10);

		if ((read_status = HAL_UART_Receive(&SERIAL_UART, rx_buffer,
				message_size, 100)) == HAL_OK) {

			if (robot_count == 1) {

				for (int i = 0; i < 32; i++) {
					robots[0][i] = rx_buffer[i];
				}
				rf24_write(p_dev, robots[0], PAYLOAD_SIZE,
				false);

			} else if (robot_count == 2) {
				for (int i = 0; i < 32; i++) {
					robots[0][i] = rx_buffer[i];
					robots[1][i] = rx_buffer[i + 32];
				}
				rf24_write(p_dev, robots[0], PAYLOAD_SIZE,
				false);
				rf24_write(p_dev, robots[1], PAYLOAD_SIZE,
				false);

			} else if (robot_count == 3) {
				for (int i = 0; i < 32; i++) {
					robots[0][i] = rx_buffer[i];
					robots[1][i] = rx_buffer[i + 32];
					robots[2][i] = rx_buffer[i + 64];
				}
				rf24_write(p_dev, robots[0], PAYLOAD_SIZE,
				false);
				rf24_write(p_dev, robots[1], PAYLOAD_SIZE,
				false);
				rf24_write(p_dev, robots[2], PAYLOAD_SIZE,
				false);
			}

		} else {
			printf("uart error: %d\r\n", read_status);
		}
		current_time = HAL_GetTick();
		elapsed_time = current_time - last_time;
		last_time = current_time;
		printf("%lu ms\r\n", elapsed_time);
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return (ch);
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
		printf("Error");
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
