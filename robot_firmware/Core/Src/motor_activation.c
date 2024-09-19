/*
 * motor_activation.c
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#include "tim.h"
#include "motor_activation.h"

#define MOTOR1 CCR1
#define MOTOR2 CCR2
#define MOTOR3 CCR3
#define MOTOR4 CCR4

#define motor_min  -100
#define motor_max 	100
#define pwm_min 	50
#define pwm_max		100

float map(float in_value, float in_min, float in_max, float out_min, float out_max){
  return (in_value - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

void motor_init(TIM_HandleTypeDef htim1, TIM_TypeDef *TIMER){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	TIMER->MOTOR1 = 100;  // Set the maximum pulse (2ms)
	HAL_Delay (2000);  // wait for 1 beep
	TIMER->MOTOR1 = 50;   // Set the minimum Pulse (1ms)
	HAL_Delay (1000);  // wait for 2 beeps
	TIMER->MOTOR1 = 0;    // reset to 0, so it can be controlled via ADC
	TIMER->MOTOR2 = 100;  // Set the maximum pulse (2ms)
	HAL_Delay (2000);  // wait for 1 beep
	TIMER->MOTOR2 = 50;   // Set the minimum Pulse (1ms)
	HAL_Delay (1000);  // wait for 2 beeps
	TIMER->MOTOR2 = 0;    // reset to 0, so it can be controlled via ADC
	TIMER->MOTOR3 = 100;  // Set the maximum pulse (2ms)
	HAL_Delay (2000);  // wait for 1 beep
	TIMER->MOTOR3 = 50;   // Set the minimum Pulse (1ms)
	HAL_Delay (1000);  // wait for 2 beeps
	TIMER->MOTOR3 = 0;    // reset to 0, so it can be controlled via ADC
	TIMER->MOTOR4 = 100;  // Set the maximum pulse (2ms)
	HAL_Delay (2000);  // wait for 1 beep
	TIMER->MOTOR4 = 50;   // Set the minimum Pulse (1ms)
	HAL_Delay (1000);  // wait for 2 beeps
	TIMER->MOTOR4 = 0;    // reset to 0, so it can be controlled via ADC

	TIMER->MOTOR1 = 75;
	TIMER->MOTOR2 = 75;
	TIMER->MOTOR3 = 75;
	TIMER->MOTOR4 = 75;
	HAL_Delay (2000);
}

void write_speed_to_motors(TIM_TypeDef *TIMER, float *wheel_speed){
	TIMER->MOTOR1 = map(wheel_speed[0], motor_min, motor_max, pwm_min, pwm_max);
	TIMER->MOTOR2 = map(wheel_speed[1], motor_min, motor_max, pwm_min, pwm_max);
	TIMER->MOTOR3 = map(wheel_speed[2], motor_min, motor_max, pwm_min, pwm_max);
	TIMER->MOTOR4 = map(wheel_speed[3], motor_min, motor_max, pwm_min, pwm_max);
}