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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "radio_interface.h"

#include "inverse_kinematics.h"

#include "motor_activation.h"

#include "mpu6050.h"

#include "pid.h"

#define VERBOSE

#ifdef VERBOSE
#include <stdio.h>
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERIAL_UART huart3

#define HSPI hspi1

#define HTIM htim1

#define MOTOR_TIMER TIM1
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
#ifdef VERBOSE
int __io_putchar(int ch);
#endif

void calculate_IMU_error();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

command_t cmd;
command_t *p_cmd = &cmd;

rf24_dev_t radio_device;
rf24_dev_t *p_radio_dev = &radio_device; /* Pointer to module instance */

uint8_t address[5] = { 0xB9, 0xB7, 0xE7, 0xE9, 0xC2 };

float* inverse_kinematics_result;

MPU6050_t MPU6050;
float GyroErrorX;

PID_TypeDef TPIDVTheta;
float PIDOutVtheta;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	/////////////////////////////////////////////////////////////////////////////
	printf("Initializing motors\r\n");

	motor_init(htim1, MOTOR_TIMER);

	printf("Motors initialized\r\n");
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	printf("Initializing NRF24\r\n");

	//TODO: create a function for radio config and init
	//(for some reason it stops working in a function)
	/* RADIO CONFIGURATION AND INITIALIZATION */

	/* Get default configuration */
	p_radio_dev->platform_setup.spi_timeout = 1000;
	p_radio_dev->payload_size = PAYLOAD_SIZE;
	p_radio_dev->addr_width = 5;
	p_radio_dev->datarate = RF24_2MBPS;
	p_radio_dev->channel = 76;

	for (uint8_t i = 0; i < RF24_ADDRESS_MAX_SIZE; i++) {
		p_radio_dev->pipe0_reading_address[i] = 0;
	}

	/* Set spi to be used */
	p_radio_dev->platform_setup.hspi = &HSPI;

	p_radio_dev->payload_size = PAYLOAD_SIZE;

	p_radio_dev->platform_setup.csn_port = RADIO_CSN_GPIO_Port;
	p_radio_dev->platform_setup.csn_pin = RADIO_CSN_Pin;

	p_radio_dev->platform_setup.irq_port = RADIO_IRQ_GPIO_Port;
	p_radio_dev->platform_setup.irq_pin = RADIO_IRQ_Pin;

	p_radio_dev->platform_setup.ce_port = RADIO_CE_GPIO_Port;
	p_radio_dev->platform_setup.ce_pin = RADIO_CE_Pin;

	while (rf24_init(p_radio_dev) != RF24_SUCCESS)
		;

	//TODO: test different output power levels
	//rf24_set_output_power(p_radio_dev, output_power);

	rf24_status_t device_status = RF24_SUCCESS; /* Variable to receive the statuses returned by the functions */

	if (device_status == RF24_SUCCESS) {
		device_status = rf24_open_reading_pipe(p_radio_dev, 1, address);
	}

	if (device_status == RF24_SUCCESS) {
		device_status = rf24_start_listening(p_radio_dev);
	}

	if (device_status != RF24_SUCCESS) {
		printf("Error during nrf24 setup\r\n");
	}
	printf("Radio initialized\r\n");
	///////////////////////////////////////////////////////////////////////////// // @suppress("Line comments")

	/////////////////////////////////////////////////////////////////////////////
	printf("Initializing MPU6050\r\n");

	while (MPU6050_Init(&hi2c1) == 1)
		;

	printf("Calculating mean static error\r\n");

	calculate_IMU_error(MPU6050);

	printf("MPU6050 initialized\r\n");
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	printf("Creating PID control system\r\n");
	printf("Creating PID control system\r\n");

	//TODO: Calibrate pid values

	/*Angular velocity*/
	PID(&TPIDVTheta, &(MPU6050.Gz), &PIDOutVtheta, &(cmd.vtheta), 2, 5, 1,
			_PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&TPIDVTheta, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPIDVTheta, 100);
	PID_SetOutputLimits(&TPIDVTheta, -50, 50);

	printf("PID control system initialized\r\n");
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	printf("Initializing inverse kinematics\r\n");
	inverse_kinematics_result = inverse_kinematics_init();
	printf("Inverse kinematics initialized\r\n");

	//Start timer interrupt for reading radio values
//	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t currentTime = HAL_GetTick();
	uint32_t previousTime;
	uint32_t elapsedTime;

	HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);

	p_cmd->vx = 1;
	p_cmd->vy = 1;
	p_cmd->kik_sig = 1;

	while (1) {
    /* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/*		READ KICK SIGNAL AND INFRARED FOR KICKCING		*/
		if (p_cmd->kik_sig == 1) {
			int ir = HAL_GPIO_ReadPin(INFRARED_GPIO_Port, INFRARED_Pin);

			if (ir == 0) {
				HAL_GPIO_WritePin(KICKER_GPIO_Port, KICKER_Pin, GPIO_PIN_SET);
				HAL_Delay(3);
				HAL_GPIO_WritePin(KICKER_GPIO_Port, KICKER_Pin, GPIO_PIN_RESET);
			}
		}

		/*		READ DATA FROM MPU6050 and treat it		*/
		MPU6050_Read_Gyro(&hi2c1, &MPU6050);

		/*		Compute compensated values for Vtheta		*/
		PID_Compute(&TPIDVTheta);

		/*		Get wheel speed from inverse kinematics		*/
		calculate_wheel_speed(p_cmd);

		/*		Write speed to motors		*/
		write_speed_to_motors(MOTOR_TIMER, inverse_kinematics_result);

		previousTime = currentTime;
		currentTime = HAL_GetTick();
		elapsedTime = (currentTime - previousTime);

		printf(">elapsed_time:%lu\r\n", elapsedTime);
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
#ifdef VERBOSE
int __io_putchar(int ch) {
	HAL_UART_Transmit(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return (ch);
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		radio_read_and_update(p_radio_dev, p_cmd, &TPIDVTheta);
	}
}

void calculate_IMU_error(MPU6050_t MPU6050) {
	int error_iterations = 10000;

	for (int c = 0; c < error_iterations; c++) {
		MPU6050_Read_Gyro(&hi2c1, &MPU6050);

		// Sum all readings
		GyroErrorX = GyroErrorX + MPU6050.Gx;
	}
	//Divide the sum by 200 to get the error value
	GyroErrorX = GyroErrorX / error_iterations;
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
	while (1) {
		printf("Erro kkkkkkkkk\r\n");
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
