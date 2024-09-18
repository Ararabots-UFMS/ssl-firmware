/*
 * motor_activation.h
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#ifndef INC_MOTOR_ACTIVATION_H_
#define INC_MOTOR_ACTIVATION_H_

float map(float x, float in_min, float in_max, float out_min, float out_max);

void motor_init(TIM_HandleTypeDef htim1, TIM_TypeDef *TIMER);

void write_speed_to_motors(float *wheel_speed);

#endif /* INC_MOTOR_ACTIVATION_H_ */
