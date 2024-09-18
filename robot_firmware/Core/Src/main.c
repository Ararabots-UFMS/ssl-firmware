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

#include "inverse_kinematics.h"


//#define RADIO

#ifdef RADIO
	#include "rf24.h"
#endif

#define VERBOSE

#ifdef VERBOSE
	#include <stdio.h>
#endif


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct command {
	float des_vx, des_vy;
	float des_orient;
	float cur_orient;
	uint8_t kik_sig;
} command_t;

typedef struct pidvalues {
	float p;
	float pi;
	float pd;
} pidvalues_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define SERIAL_UART huart1

#define PAYLOAD_SIZE 32			//size in bytes for radio communication

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

void get_command_from_buffer(uint8_t buffer[], command_t* result);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// command_t cmd;
// command_t *p_cmd = &cmd;

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	#ifdef RADIO
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

		p_dev->payload_size = PAYLOAD_SIZE;

		rf24_init(p_dev);

		uint8_t addresses[2][5] = {{0xE7, 0xE7, 0xE7, 0xE7, 0xE8}, {0xC2, 0xC2, 0xC2, 0xC2, 0xC1}};

		//No idea what output_power should be
		//rf24_set_output_power(p_dev, output_power);

		rf24_status_t device_status; /* Variable to receive the statuses returned by the functions */

		device_status = rf24_open_writing_pipe(p_dev, addresses[0]);
		device_status = rf24_open_reading_pipe(p_dev, 1, addresses[1]);

		device_status = rf24_start_listening(p_dev);
	#endif

  //uint8_t loop=0;

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////   RADIO SIMULATION  ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////

	// uint8_t buffer[32];
	// buffer[0] = 0x01;

	// buffer[1] = 0x3f;
	// buffer[2] = 0x80;
	// buffer[3] = 0x00;
	// buffer[4] = 0x00;

	// buffer[5] = 0x3f;
	// buffer[6] = 0x80;
	// buffer[7] = 0x00;
	// buffer[8] = 0x00;

	// buffer[9] = 0x3f;
	// buffer[10] = 0x80;
	// buffer[11] = 0x00;
	// buffer[12] = 0x00;

	// buffer[13] = 0x3f;
	// buffer[14] = 0x80;
	// buffer[15] = 0x00;
	// buffer[16] = 0x00;

	// buffer[17] = 0x00;
	  ////////////////////////////////////////////////////////////////////////////////////////////
	          ////////////////////////////////////     ///////////////////////////////////
	  ////////////////////////////////////////////////////////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float *wheel_speed;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);

	  //get_sensor_values();

	  //pid_control();

	  wheel_speed = get_wheel_speed(1.0, 0.0, 0.0);
	  printf("wheel_speed 1: %f, 2:%f, 3:%f, 4:%f\r\n", wheel_speed[0], wheel_speed[1], wheel_speed[2], wheel_speed[3]);
	  //write_speed_to_motors();


	  //switch (buffer[0]){
	  	  //Command message
	  	  //case 1:
			  //command_t cmd;
			  //command_t* p_cmd = &cmd;

			  //get_command_from_buffer(buffer, p_cmd);


			  //control_system();

			  //reverse_kinematics();

			  //activate_motors();

	  	  //case 2:

	  //}

	  //printf("des_vx: %f\r\n", p_msg->des_vx);
	  //printf("des_vy: %f\r\n", p_msg->des_vy);
	  //printf("des_orient: %f\r\n", p_msg->des_orient);
	  //printf("cur_orient: %f\r\n", p_msg->cur_orient);
	  //printf("kik_sig: %d\r\n", p_msg->kik_sig);


	  HAL_Delay(1000);
	#ifdef RADIO
	  uint8_t buffer[PAYLOAD_SIZE] = {"error"};
	  rf24_status_t device_status;
	  rf24_status_t read_status = 8;


	  if ((device_status = rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
	      while ((device_status = rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
            printf("available %d\t", device_status);
	          read_status = rf24_read(p_dev, buffer, p_dev->payload_size);
	      }
	  }
    else {
      printf("not able to read %d\t", device_status);
    }

	  if(read_status == RF24_SUCCESS){
		  printf("read ok\t");
		  des_vx = (buffer[0]<<8) + buffer[1];
		  des_vy = (buffer[2]<<8) + buffer[3];
		  des_orient = ((buffer[4]<<8) + buffer[5])>>1;
		  cur_orient = ((buffer[6]<<8) + buffer[7])>>1;
		  kik_sig = buffer[7] & 0b00000001;
		  printf("\ndes_vx: %d", des_vx);
		  printf("\ndes_vy: %d", des_vy);
		  printf("\ndes_orient: %d", des_orient);
		  printf("\ncur_orient: %d", cur_orient);
		  printf("\nkik_sig: %d", kik_sig);
	  }else{
		  printf("not able to read %d\t", read_status);
	  }

	  printf("%d - %s\r\n", loop++, buffer);
#endif
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


void get_command_from_buffer(uint8_t *buffer, command_t* result){

	union {
		uint8_t ints[4];
		float value;
	} uint8_to_float;

	uint8_to_float.ints[3] = buffer[1];
	uint8_to_float.ints[2] = buffer[2];
	uint8_to_float.ints[1] = buffer[3];
	uint8_to_float.ints[0] = buffer[4];
	result->des_vx = uint8_to_float.value;

	uint8_to_float.ints[3] = buffer[5];
	uint8_to_float.ints[2] = buffer[6];
	uint8_to_float.ints[1] = buffer[7];
	uint8_to_float.ints[0] = buffer[8];
	result->des_vy = uint8_to_float.value;

	uint8_to_float.ints[3] = buffer[9];
	uint8_to_float.ints[2] = buffer[10];
	uint8_to_float.ints[1] = buffer[11];
	uint8_to_float.ints[0] = buffer[12];
	result->des_orient = uint8_to_float.value;

	uint8_to_float.ints[3] = buffer[13];
	uint8_to_float.ints[2] = buffer[14];
	uint8_to_float.ints[1] = buffer[15];
	uint8_to_float.ints[0] = buffer[16];
	result->cur_orient = uint8_to_float.value;

	result->kik_sig = buffer[17] & 0b00000001;
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
