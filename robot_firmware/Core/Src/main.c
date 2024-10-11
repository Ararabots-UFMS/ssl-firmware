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

#define ADDRESSES {{0xE7, 0xE7, 0xE7, 0xE7, 0xE8}, {0xC2, 0xC2, 0xC2, 0xC2, 0xC1}}
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

int __io_getchar(void);
#endif

void calculate_IMU_error();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

command_t cmd;
command_t *p_cmd = &cmd;

float yaw = 0;

rf24_dev_t radio_device;
rf24_dev_t *p_radio_dev = &radio_device; /* Pointer to module instance */

inverse_kinematics_t inverse_kinematics;
inverse_kinematics_t *p_inverse_kinematics = &inverse_kinematics;

MPU6050_t MPU6050;
float AccErrorX;
float AccErrorY;
float AccErrorZ;
float GyroErrorX;
float GyroErrorY;
float GyroErrorZ;

PID_TypeDef TPIDVTheta;
float PIDOutVtheta;
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
	rf24_get_default_config(p_radio_dev);

	/* Set spi to be used */
	p_radio_dev->platform_setup.hspi = &HSPI;

	/* CSN pin */
	p_radio_dev->platform_setup.csn_port = RADIO_CSN_GPIO_Port;
	p_radio_dev->platform_setup.csn_pin = RADIO_CSN_Pin;

	/* IRQ pin */
	p_radio_dev->platform_setup.irq_port = RADIO_IRQ_GPIO_Port;
	p_radio_dev->platform_setup.irq_pin = RADIO_IRQ_Pin;

	/* CE pin */
	p_radio_dev->platform_setup.ce_port = RADIO_CE_GPIO_Port;
	p_radio_dev->platform_setup.ce_pin = RADIO_CE_Pin;

	p_radio_dev->payload_size = PAYLOAD_SIZE;

	while (rf24_init(p_radio_dev) != RF24_SUCCESS)
		;

	//TODO: change this to a constant with define
	uint8_t addresses[2][5] = { { 0xE7, 0xE7, 0xE7, 0xE7, 0xE8 }, { 0xC2, 0xC2,
			0xC2, 0xC2, 0xC1 } };

	//TODO: test different output power levels
	//rf24_set_output_power(p_radio_dev, output_power);

	rf24_status_t device_status = RF24_SUCCESS; /* Variable to receive the statuses returned by the functions */

	if (device_status == RF24_SUCCESS) {
		device_status = rf24_open_writing_pipe(p_radio_dev, addresses[0]);
	}

	if (device_status == RF24_SUCCESS) {
		device_status = rf24_open_reading_pipe(p_radio_dev, 1, addresses[1]);
	}

	if (device_status == RF24_SUCCESS) {
		device_status = rf24_start_listening(p_radio_dev);
	}

#ifdef VERBOSE
	if (device_status != RF24_SUCCESS) {
		printf("Error during nrf24 setup\r\n");
	}
#endif
	printf("Radio initialized\r\n");
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	printf("Initializing MPU6050\r\n");

	while (MPU6050_Init(&hi2c1) == 1)
		;

	printf("Calculating mean static error\r\n");

	calculate_IMU_error(MPU6050);

	printf("MPU6050 initialized\r\n");
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	printf("Creating PID control system");

	//TODO: Calibrate pid values

	/*Angular velocity*/
	PID(&TPIDVTheta, &(MPU6050.Gz), &PIDOutVtheta, &(cmd.vtheta), 2, 5, 1,
			_PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&TPIDVTheta, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPIDVTheta, 100);
	PID_SetOutputLimits(&TPIDVTheta, -50, 50);

	printf("PID control system initialized");
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	printf("Initializing inverse kinematics");
	inverse_kinematics_init(p_inverse_kinematics);
	printf("Inverse kinematics initialized");

	//Start timer interrupt for reading radio values
	HAL_TIM_Base_Start_IT(&htim2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t currentTime = HAL_GetTick();
	uint32_t previousTime;
	uint32_t elapsedTime;

	p_cmd->vx = 1;
	p_cmd->vy = 1;
	p_cmd->kik_sig = 1;

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/*		LED FOR DEBUGGING		*/
		HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);

		/*		READ DATA FROM MPU6050 and treat it		*/
		previousTime = currentTime;
		currentTime = HAL_GetTick();
		elapsedTime = (currentTime - previousTime);
		MPU6050_Read_All(&hi2c1, &MPU6050);
		yaw += ((MPU6050.Gz - GyroErrorZ) * elapsedTime) / 1000;
		if (yaw < -PI)
			yaw += 2 * PI;
		if (yaw > PI)
			yaw -= 2 * PI;

		/*		READ KICK SIGNAL AND INFRARED FOR KICKCING		*/
		if (p_cmd->kik_sig == 1) {
			int ir = HAL_GPIO_ReadPin(INFRARED_GPIO_Port, INFRARED_Pin);
			printf(">infrared:%d\r\n", ir);
			if (ir == 0) {
				HAL_GPIO_WritePin(KICKER_GPIO_Port, KICKER_Pin, GPIO_PIN_SET);
				HAL_Delay(3);
				HAL_GPIO_WritePin(KICKER_GPIO_Port, KICKER_Pin, GPIO_PIN_RESET);
			}
		}

		//TODO: merge_sensors();

		/*		Compute compensated values for Vtheta		*/
		PID_Compute(&TPIDVTheta);

		/*		Get wheel speed from inverse kinematics		*/
		calculate_wheel_speed(p_inverse_kinematics, yaw);

		/*		Write speed to motors		*/
		write_speed_to_motors(MOTOR_TIMER, p_inverse_kinematics);

		printf(">elapsed_time:%lu\r\n", elapsedTime);
		HAL_Delay(50);
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
#ifdef VERBOSE
int __io_putchar(int ch) {
	HAL_UART_Transmit(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return (ch);
}

int __io_getchar(void) {
	uint8_t ch = 0;

	/* Clear the Overrun flag just before receiving the first character */
	__HAL_UART_CLEAR_OREFLAG(&SERIAL_UART);

	/* Wait for reception of a character on the USART RX line and echo this
	 * character on console */
	HAL_UART_Receive(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&SERIAL_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return (ch);
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		radio_read_and_update(p_radio_dev, p_cmd, &TPIDVTheta, &yaw);
	}
}

void calculate_IMU_error(MPU6050_t MPU6050) {
	int error_iterations = 10000;

	for (int c = 0; c < error_iterations; c++) {
		MPU6050_Read_All(&hi2c1, &MPU6050);

		// Sum all readings
		AccErrorX = AccErrorX + MPU6050.Ax;
		AccErrorY = AccErrorY + MPU6050.Ay;
		AccErrorZ = AccErrorZ + (MPU6050.Az - 1);

		// Sum all readings
		GyroErrorX = GyroErrorX + MPU6050.Gx;
		GyroErrorY = GyroErrorY + MPU6050.Gy;
		GyroErrorZ = GyroErrorZ + MPU6050.Gz;
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
void Error_Handler(void) {
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
