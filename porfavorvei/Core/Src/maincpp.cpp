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
#include "led.h"

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
	return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}


int maincpp(TIM_HandleTypeDef htim4)
{
  /* USER CODE BEGIN 2 */


	//uint32_t AD_RES = 0;
	//uint32_t map_val = 0;
  Dout led0(LED0_GPIO_Port, LED0_Pin);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 700);
  HAL_Delay(10000);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 650);


  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);

  //HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  // Poll ADC1 Peripheral & TimeOut = 1mSec
		//HAL_ADC_PollForConversion(&hadc1, 1);
		// Read The ADC Conversion Result & Map It To PWM DutyCycle
		//AD_RES = HAL_ADC_GetValue(&hadc1);
		//map_val = MAP(AD_RES, 0, 4096, 700, 2000);
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, map_val);
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, map_val);

		//HAL_Delay(1);
	  led0.toggle();
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}
