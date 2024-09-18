/*
 * inverse_kinematics.c
 *
 *  Created on: Sep 17, 2024
 *      Author: allan
 */
#include <math.h>
#include <stdlib.h>

#include "inverse_kinematics.h"


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

float* get_wheel_speed(float vx, float vy, float vtheta){

	float velocities[3][1] = {{vx}, {vy}, {vtheta}};

	float wheel_radius = 0.027;

	float robot_radius = 0.09;

	float wheel_angles[] = {PI*(5.0/6.0), PI*(5.0/4.0), PI*(7.0/4.0), PI*(1.0/6.0)};

	float jacobian[4][3] = {{cos(wheel_angles[0]), sin(wheel_angles[0]), robot_radius},
	                        {cos(wheel_angles[1]), sin(wheel_angles[1]), robot_radius},
	                        {cos(wheel_angles[2]), sin(wheel_angles[2]), robot_radius},
							{cos(wheel_angles[3]), sin(wheel_angles[3]), robot_radius}};

	///////////////////////////////////////////////////////////
	float **p_velocities = (float **)malloc(3 * sizeof(float *));
	for (int i = 0; i < 3; i++) {
		p_velocities[i] = (float *)malloc(1 * sizeof(float));
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 1; j++) {
			p_velocities[i][j] = velocities[i][j];
		}
	}

	///////////////////////////////////////////////////////////
	float **p_jacobian = (float **)malloc(4 * sizeof(float *));
	for (int i = 0; i < 4; i++) {
		p_jacobian[i] = (float *)malloc(3 * sizeof(float));
	}

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			p_jacobian[i][j] = jacobian[i][j];
		}
	}

	///////////////////////////////////////////////////////////
	float **result;
    result = (float **)malloc(4 * sizeof(float *));
    for (int i = 0; i < 4; i++) {
        result[i] = (float *)malloc(1 * sizeof(float));
    }

	///////////////////////////////////////////////////////////
	multiply_matrices(p_jacobian, 4, 3, p_velocities, 3, 1, result);
	multiply_matrix_scalar(result, 4, 1, 1.0/wheel_radius);
	float* array = (float*)malloc(4 * sizeof(float));

	array[0] = result[0][0];
	array[1] = result[1][0];
	array[2] = result[2][0];
	array[3] = result[3][0];

	return array;
}
