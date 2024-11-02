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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rf24.h"
#include <stdio.h>

#include "rf24_debug.h"
#include "mpu6050.h"

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
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int __io_putchar(int ch);

int __io_getchar(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calculate_IMU_error();

float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;


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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  printf("Initializing\r\n");
  HAL_Delay(500);
  while (MPU6050_Init(&hi2c1) == 1) {
  }
  printf("calculating error\r\n");
  calculate_IMU_error(MPU6050);

  rf24_dev_t device; /* Module instance */
  rf24_dev_t* p_dev = &device; /* Pointer to module instance */

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

  p_dev->payload_size = 15;

  rf24_init(p_dev);

  uint8_t addresses[2][5] = {{0xE7, 0xE7, 0xE7, 0xE7, 0xE8}, {0xC2, 0xC2, 0xC2, 0xC2, 0xC1}};

  rf24_set_output_power(p_dev, RF24_18_dBm);

  rf24_open_writing_pipe(p_dev, addresses[1]);
  rf24_open_reading_pipe(p_dev, 1, addresses[0]);

  uint8_t buffer[15];

  union {
  	uint8_t ints[4];
  	float value;
  } uint8_to_float;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	MPU6050_Read_All(&hi2c1, &MPU6050);
	uint8_to_float.value = MPU6050.Gx;

	buffer[0] = 1;
	buffer[1] = uint8_to_float.ints[3];
	buffer[2] = uint8_to_float.ints[2];
	buffer[3] = uint8_to_float.ints[1];
	buffer[4] = uint8_to_float.ints[0];

	// printf(">gyrox:%f,gyroy:%f,gyroz:%f\r\n", MPU6050.Gx - GyroErrorX,MPU6050.Gy - GyroErrorY, MPU6050.Gz - GyroErrorZ);

	HAL_Delay(100);

	if ((rf24_write(p_dev, buffer, sizeof(buffer), true)) == RF24_SUCCESS){
		printf("Virtual Hugs!\r\n");
		printf("uint8_to_float.value: %f\r\n", uint8_to_float.value);
	}else{
		printf("tentei\r\n");
	}



	  //rf24_debug_dump_registers(p_dev);

	  //rf24_debug_print_status(p_dev);

	  HAL_Delay(500);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int __io_getchar(void)
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&SERIAL_UART);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&SERIAL_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void calculate_IMU_error(MPU6050_t MPU6050) {
	int c = 0;
	int error_iterations = 10000;

	while (c < error_iterations) {
		MPU6050_Read_All(&hi2c1, &MPU6050);

		// Sum all readings
		AccErrorX = AccErrorX + MPU6050.Ax;
		AccErrorY = AccErrorY + MPU6050.Ay;
		AccErrorZ = AccErrorZ + (MPU6050.Az - 1);

		// Sum all readings
		GyroErrorX = GyroErrorX + MPU6050.Gx;
		GyroErrorY = GyroErrorY + MPU6050.Gy;
		GyroErrorZ = GyroErrorZ + MPU6050.Gz;
		c++;
	}
	//Divide the sum by 200 to get the error value
	AccErrorX = AccErrorX / error_iterations;
	AccErrorY = AccErrorY / error_iterations;
	AccErrorZ = AccErrorZ / error_iterations;
	//Divide the sum by 200 to get the error value
	GyroErrorX = GyroErrorX / error_iterations;
	GyroErrorY = GyroErrorY / error_iterations;
	GyroErrorZ = GyroErrorZ / error_iterations;
	// Print the error values on the Serial Monitor
}
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
