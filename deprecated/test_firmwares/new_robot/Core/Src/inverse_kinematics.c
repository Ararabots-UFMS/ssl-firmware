	/*
 * inverse_kinematics.c
 *
 *  Created on: Sep 17, 2024
 *      Author: allan
 */
#include <stdlib.h>

#include "inverse_kinematics.h"
#include <math.h>

#define WHEEL_RADIUS 0.034

#define ROBOT_RADIUS 0.72

const float wheel_angles[4] = { PI * (5.0 / 6.0), PI * (5.0 / 4.0), PI
		* (7.0 / 4.0), PI * (1.0 / 6.0) };

float jacobian[4][3];

float *result;

void inverse_kinematics_init() {
	for (int i = 0; i < 4; i++) {
		jacobian[i][0] = cos(wheel_angles[i]);
		jacobian[i][1] = sin(wheel_angles[i]);
		jacobian[i][2] = ROBOT_RADIUS;
	}

	result = (float*) malloc(4 * sizeof(float));
}

float * calculate_wheel_speed(command_t *p_cmd, float PIDOut) {

	for (int i = 0; i < 4; i++) {
		result[i] = 0;
		result[i] = result[i] + jacobian[i][0] * p_cmd->vx;
		result[i] = result[i] + jacobian[i][1] * p_cmd->vy;
		result[i] = result[i] + jacobian[i][2] * p_cmd->vtheta;
		result[i] = (1 / WHEEL_RADIUS) * result[i];
	}
	return result;
}
