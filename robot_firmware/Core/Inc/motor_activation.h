/*
 * motor_activation.h
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#ifndef INC_MOTOR_ACTIVATION_H_
#define INC_MOTOR_ACTIVATION_H_

#include "inverse_kinematics.h"

void motor_init(TIM_HandleTypeDef htim1, TIM_TypeDef *TIMER);

void write_speed_to_motors(TIM_TypeDef *TIMER, float *inverse_kinematics);

#endif /* INC_MOTOR_ACTIVATION_H_ */
