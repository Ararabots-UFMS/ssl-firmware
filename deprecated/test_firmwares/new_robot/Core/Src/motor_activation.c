/*
 * motor_activation.c
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#include "tim.h"
#include "motor_activation.h"
#include "main.h"

#define MOTOR1 CCR1
#define MOTOR2 CCR2
#define MOTOR3 CCR3
#define MOTOR4 CCR4

#define motor_min 0.0
#define motor_max 170.0
#define pwm_min 58.0
#define pwm_max 100.0

GPIO_TypeDef *ports[4] = { RELAY1_GPIO_Port, RELAY2_GPIO_Port, RELAY3_GPIO_Port,
RELAY4_GPIO_Port };
uint16_t pins[4] = { RELAY1_Pin, RELAY2_Pin, RELAY3_Pin, RELAY4_Pin };

uint8_t last_values[4] = { 0, 0, 0, 0 };
int8_t last_direction[4] = { 1, 1, 1, 1 };

float map(float in_value) {
	if (in_value <= 0)
		return 0;
	else if (in_value > motor_max)
		return motor_max;
	return (in_value - motor_min) * (pwm_max - pwm_min + 1)
			/ (motor_max - motor_min + 1) + pwm_min;
}

void motor_init(TIM_HandleTypeDef htim1, TIM_TypeDef *TIMER) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	TIMER->MOTOR1 = 100; // Set the maximum pulse (2ms)
	TIMER->MOTOR2 = 100; // Set the maximum pulse (2ms)
	TIMER->MOTOR3 = 100; // Set the maximum pulse (2ms)
	TIMER->MOTOR4 = 100; // Set the maximum pulse (2ms)
	HAL_Delay(4005);	 // wait for 1 beep
	TIMER->MOTOR1 = 50;	 // Set the minimum Pulse (1ms)
	TIMER->MOTOR2 = 50;	 // Set the minimum Pulse (1ms)
	TIMER->MOTOR3 = 50;	 // Set the minimum Pulse (1ms)
	TIMER->MOTOR4 = 50;	 // Set the minimum Pulse (1ms)
	HAL_Delay(2005);	 // wait for 2 beeps
	TIMER->MOTOR1 = 0;	 // reset to 0, so it can be controlled via ADC
	TIMER->MOTOR2 = 0;	 // reset to 0, so it can be controlled via ADC
	TIMER->MOTOR3 = 0;	 // reset to 0, so it can be controlled via ADC
	TIMER->MOTOR4 = 0;	 // reset to 0, so it can be controlled via ADC
}

uint8_t is_safe(float current_value, uint8_t current_direction,
		uint8_t motor_idex) {
	if ((last_values[motor_idex] > 60.0 && current_value > 60.0)
			&& (last_direction[0] != current_direction)) {
		return 0;
	}
	return 1;
}

void write_speed_to_motors(TIM_TypeDef *TIMER, float *inverse_kinematics) {

	float wheel_speed;
	int8_t current_orientation;

	for (int i = 0; i < 4; i++) {

		if (inverse_kinematics[i] < 0) {
			current_orientation = -1;
			inverse_kinematics[i] *= -1;
		} else {
			current_orientation = 1;
		}

		wheel_speed = map(inverse_kinematics[i]);

		if (is_safe(wheel_speed, current_orientation, i)) {
			if (current_orientation < 0) {
				HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
			}
			last_values[i] = wheel_speed;
		} else {
			last_values[i] = 0;
		}
		last_direction[i] = current_orientation;
	}

	printf("[%d, %d, %d, %d]\r\n", last_values[0], last_values[1],
			last_values[2], last_values[3]);

	TIMER->MOTOR1 = last_values[0];
	TIMER->MOTOR2 = last_values[1];
	TIMER->MOTOR3 = last_values[2];
	TIMER->MOTOR4 = last_values[3];
}
