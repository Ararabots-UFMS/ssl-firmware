/*
 * inverse_kinematics.c
 *
 *  Created on: Sep 17, 2024
 *      Author: allan
 */
#include <stdlib.h>

#include "inverse_kinematics.h"
#include <math.h>

#define WHEEL_RADIUS 0.027

#define ROBOT_RADIUS 0.09

const float wheel_angles[] = {PI*(5.0/6.0), PI*(5.0/4.0), PI*(7.0/4.0), PI*(1.0/6.0)};

// function to multiply two matrices
void multiply_matrices(	float **matA, int rowsA, int colsA,
						float **matB, int rowsB, int colsB,
						float **result) {

    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            result[i][j] = 0;
            for (int k = 0; k < colsA; k++) {
                result[i][j] += matA[i][k] * matB[k][j];
            }
        }
    }
}

void multiply_matrix_scalar(float **result, int rowsR, int colsR,
							float scalar){
	for (int i = 0; i < rowsR; i++) {
		for (int j = 0; j < colsR; j++) {
			result[i][j] *= scalar;
		}
	}
}

void inverse_kinematics_init(inverse_kinematics_t* inverse_kinematics){

	/////////////////////////////////////////////////////////////////////////////

	inverse_kinematics->p_velocities = (float **)malloc(3 * sizeof(float *));

	for (int i = 0; i < 3; i++)
		inverse_kinematics->p_velocities[i] = (float *)malloc(1 * sizeof(float));

	inverse_kinematics->p_velocities[0] = inverse_kinematics->vx;
	inverse_kinematics->p_velocities[1] = inverse_kinematics->vy;
	inverse_kinematics->p_velocities[2] = inverse_kinematics->vtheta;

	/////////////////////////////////////////////////////////////////////////////

	inverse_kinematics->p_jacobian = (float **)malloc(4 * sizeof(float *));
	for (int i = 0; i < 4; i++) {
		inverse_kinematics->p_jacobian[i] = (float *)malloc(3 * sizeof(float));
	}

	/////////////////////////////////////////////////////////////////////////////
    inverse_kinematics->p_result = (float **)malloc(4 * sizeof(float *));
    for (int i = 0; i < 4; i++) {
    	inverse_kinematics->p_result[i] = (float *)malloc(1 * sizeof(float));
    }
	/////////////////////////////////////////////////////////////////////////////
}

void calculate_wheel_speed(inverse_kinematics_t* inverse_kinematics){

	float jacobian[4][3] = {{cos(wheel_angles[0]), sin(wheel_angles[0]), ROBOT_RADIUS},
	                        {cos(wheel_angles[1]), sin(wheel_angles[1]), ROBOT_RADIUS},
	                        {cos(wheel_angles[2]), sin(wheel_angles[2]), ROBOT_RADIUS},
							{cos(wheel_angles[3]), sin(wheel_angles[3]), ROBOT_RADIUS}};

	///////////////////////////////////////////////////////////
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			inverse_kinematics->p_jacobian[i][j] = jacobian[i][j];
		}
	}

	///////////////////////////////////////////////////////////
	multiply_matrices(inverse_kinematics->p_jacobian, 4, 3, inverse_kinematics->p_velocities, 3, 1, inverse_kinematics->p_result);
	multiply_matrix_scalar(inverse_kinematics->p_result, 4, 1, 1.0/WHEEL_RADIUS);
}
