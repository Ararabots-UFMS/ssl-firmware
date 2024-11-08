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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //HAL_Delay (5000);  // wait for 1 beep
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  // HAL_GPIO_WritePin(INVERSE_GPIO_Port, INVERSE_Pin, GPIO_PIN_RESET);

  #ifdef Calibrate
    TIM1->CCR1 = 100;  // Set the maximum pulse (2ms)
    TIM1->CCR2 = 100;  // Set the maximum pulse (2ms)
    TIM1->CCR3 = 100;  // Set the maximum pulse (2ms)
    TIM1->CCR4 = 100;  // Set the maximum pulse (2ms)
    HAL_Delay (4005);  // wait for 1 beep
    TIM1->CCR1 = 50;   // Set the minimum Pulse (1ms)
    TIM1->CCR2 = 50;   // Set the minimum Pulse (1ms)
    TIM1->CCR3 = 50;   // Set the minimum Pulse (1ms)
    TIM1->CCR4 = 50;   // Set the minimum Pulse (1ms)
    HAL_Delay (2005);  // wait for 2 beeps
    TIM1->CCR1 = 0;    // reset to 0, so it can be controlled via ADC
    TIM1->CCR2 = 0;    // reset to 0, so it can be controlled via ADC
    TIM1->CCR3 = 0;    // reset to 0, so it can be controlled via ADC
    TIM1->CCR4 = 0;    // reset to 0, so it can be controlled via ADC


    /*
    HAL_Delay (4000);  // wait for 1 beep
    HAL_Delay (2000);  // wait for 2 beeps
    HAL_Delay (4000);  // wait for 1 beep
    HAL_Delay (2000);  // wait for 2 beeps
    HAL_Delay (4000);  // wait for 1 beep
    HAL_Delay (2000);  // wait for 2 beeps
    */
  #endif

   /*TIM1->CCR1 = 75;
   TIM1->CCR2 = 75;
   TIM1->CCR3 = 75;
   TIM1->CCR4 = 75;
   HAL_Delay (2000);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*for(int i=0; i<10; i+=2){
		  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		  	TIM1->CCR1 = 50+i;
			TIM1->CCR2 = 50+i;
			TIM1->CCR3 = 50+i;
			TIM1->CCR4 = 50+i;

			HAL_Delay(3000);
	  }*/

	  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	  	HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_RESET);

		TIM1->CCR1 = 60;
		TIM1->CCR2 = 60;
		TIM1->CCR3 = 60;
		TIM1->CCR4 = 60;

		HAL_Delay(3000);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_SET);

		TIM1->CCR1 = 60;
		TIM1->CCR2 = 60;
		TIM1->CCR3 = 60;
		TIM1->CCR4 = 60;

		HAL_Delay(3000);

		/*TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
		TIM1->CCR3 = 50;
		TIM1->CCR4 = 50;

		HAL_Delay(2000);*/

		/*HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_RESET);

		TIM1->CCR1 = 70;
		TIM1->CCR2 = 70;
		TIM1->CCR3 = 70;
		TIM1->CCR4 = 70;

		HAL_Delay(3000);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_SET);

		TIM1->CCR1 = 70;
		TIM1->CCR2 = 70;
		TIM1->CCR3 = 70;
		TIM1->CCR4 = 70;

		HAL_Delay(3000);

		/*TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
		TIM1->CCR3 = 50;
		TIM1->CCR4 = 50;

		HAL_Delay(2000);*/

		/*HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_RESET);

		TIM1->CCR1 = 80;
		TIM1->CCR2 = 80;
		TIM1->CCR3 = 80;
		TIM1->CCR4 = 80;

		HAL_Delay(3000);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_SET);

		TIM1->CCR1 = 80;
		TIM1->CCR2 = 80;
		TIM1->CCR3 = 80;
		TIM1->CCR4 = 80;

		HAL_Delay(3000);

		/*TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
		TIM1->CCR3 = 50;
		TIM1->CCR4 = 50;

		HAL_Delay(2000);*/

		/*HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_RESET);

		TIM1->CCR1 = 90;
		TIM1->CCR2 = 90;
		TIM1->CCR3 = 90;
		TIM1->CCR4 = 90;

		HAL_Delay(3000);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_SET);

		TIM1->CCR1 = 90;
		TIM1->CCR2 = 90;
		TIM1->CCR3 = 90;
		TIM1->CCR4 = 90;

		HAL_Delay(3000);

		/*TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
		TIM1->CCR3 = 50;
		TIM1->CCR4 = 50;

		HAL_Delay(2000);*/

		/*HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_RESET);

		TIM1->CCR1 = 99;
		TIM1->CCR2 = 99;
		TIM1->CCR3 = 99;
		TIM1->CCR4 = 99;

		HAL_Delay(3000);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_SET);

		TIM1->CCR1 = 99;
		TIM1->CCR2 = 99;
		TIM1->CCR3 = 99;
		TIM1->CCR4 = 99;

		HAL_Delay(3000);

		/*TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
		TIM1->CCR3 = 50;
		TIM1->CCR4 = 50;

		HAL_Delay(2000);*/

		/*HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		HAL_GPIO_WritePin(INVERSE1_GPIO_Port, INVERSE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE2_GPIO_Port, INVERSE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE3_GPIO_Port, INVERSE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVERSE4_GPIO_Port, INVERSE4_Pin, GPIO_PIN_SET);

		TIM1->CCR1 = 100;
		TIM1->CCR2 = 100;
		TIM1->CCR3 = 100;
		TIM1->CCR4 = 100;

		HAL_Delay(5000);

		/*TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
		TIM1->CCR3 = 50;
		TIM1->CCR4 = 50;

		HAL_Delay(2000);*/

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INVERSE1_Pin|INVERSE2_Pin|INVERSE3_Pin|INVERSE4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INVERSE1_Pin INVERSE2_Pin INVERSE3_Pin INVERSE4_Pin */
  GPIO_InitStruct.Pin = INVERSE1_Pin|INVERSE2_Pin|INVERSE3_Pin|INVERSE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
