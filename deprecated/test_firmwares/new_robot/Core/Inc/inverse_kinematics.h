/*
 * inverse_kinematics.h
 *
 *  Created on: Sep 17, 2024
 *      Author: allan
 */

#ifndef INC_INVERSE_KINEMATICS_H_
#define INC_INVERSE_KINEMATICS_H_

/*
 * inverse_kinematics.c
 *
 *  Created on: Sep 17, 2024
 *      Author: allan
 */

#include "main.h"


typedef struct inverse_kinematics {
	float *vx;
	float *vy;
	float *vtheta;
	float **p_jacobian;
	float **p_velocities;
	float **p_result;
} inverse_kinematics_t;

#define PI 3.14159265358979323846

void inverse_kinematics_init();

float* calculate_wheel_speed(command_t *p_cmd, float PIDOut);


#endif /* INC_INVERSE_KINEMATICS_H_ */
