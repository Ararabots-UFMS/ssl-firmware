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

#include "inverse_kinematics.h"

#define PI 3.14159265358979323846

// function to multiply two matrices
void multiply_matrices(	float **matA, int rowsA, int colsA,
						float **matB, int rowsB, int colsB,
						float **result);

void multiply_matrix_scalar(float **result, int rowsR, int colsR,
							float scalar);

float* get_wheel_speed(float vx, float vy, float vtheta, float curtheta);


#endif /* INC_INVERSE_KINEMATICS_H_ */
