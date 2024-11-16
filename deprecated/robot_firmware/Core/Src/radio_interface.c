/*
 * radio_interface.c
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#include "rf24.h"
#include "main.h"

#include "pid.h"

union {
	uint8_t ints[4];
	float value;
} uint8_to_float;

uint8_t my_name = 'J';

void get_command_from_buffer(uint8_t *buffer, command_t *result,
		PID_TypeDef *uPID) {

	// Creates an array of 1 byte ints that can be read as a float

	uint8_to_float.ints[0] = buffer[1];
	uint8_to_float.ints[1] = buffer[2];
	uint8_to_float.ints[2] = buffer[3];
	uint8_to_float.ints[3] = buffer[4];
	result->vx = uint8_to_float.value;

	uint8_to_float.ints[0] = buffer[5];
	uint8_to_float.ints[1] = buffer[6];
	uint8_to_float.ints[2] = buffer[7];
	uint8_to_float.ints[3] = buffer[8];
	result->vy = uint8_to_float.value;

	uint8_to_float.ints[0] = buffer[9];
	uint8_to_float.ints[1] = buffer[10];
	uint8_to_float.ints[2] = buffer[11];
	uint8_to_float.ints[3] = buffer[12];
	result->vtheta = uint8_to_float.value;

	result->kik_sig = buffer[13] & 0b00000001;

	uint8_to_float.ints[0] = buffer[14];
	uint8_to_float.ints[1] = buffer[15];
	uint8_to_float.ints[2] = buffer[16];
	uint8_to_float.ints[3] = buffer[17];
	uPID->Kp = uint8_to_float.value;

	uint8_to_float.ints[0] = buffer[18];
	uint8_to_float.ints[1] = buffer[19];
	uint8_to_float.ints[2] = buffer[20];
	uint8_to_float.ints[3] = buffer[21];
	uPID->Kd = uint8_to_float.value;

	uint8_to_float.ints[0] = buffer[22];
	uint8_to_float.ints[1] = buffer[23];
	uint8_to_float.ints[2] = buffer[24];
	uint8_to_float.ints[3] = buffer[25];
	uPID->Ki = uint8_to_float.value;
}

uint8_t buffer[26] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b00000000};

rf24_status_t device_status;
rf24_status_t read_status = RF24_UNKNOWN_ERROR;
//uint32_t elapsedTime = 0;
//uint32_t last_time = 0;
//uint32_t currentTime = 0;

rf24_status_t radio_read_and_update(rf24_dev_t *p_dev, command_t *cmd,
		PID_TypeDef *uPID) {
	//last_time = HAL_GetTick();


	read_status = RF24_UNKNOWN_ERROR;
	printf("%d", rf24_available(p_dev, NULL));

	//while (buffer[0] != my_name)
	//if ((rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
	//while ((rf24_available(p_dev, NULL)) == RF24_SUCCESS){
	//	read_status = rf24_read(p_dev, buffer, p_dev->payload_size);
	//	}
	//}
	if ((rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
	    while ((rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
	    	  printf("WHILE\r\n");
	    	  read_status = rf24_read(p_dev, buffer, p_dev->payload_size);
	      }
		get_command_from_buffer(buffer, cmd, uPID);

		HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
	  }

		printf("[%d, %f, %f, %f, %d, %f, %f, %f]\r\n", buffer[0], cmd->vx,
				cmd->vy, cmd->vtheta, buffer[13], uPID->Kp, uPID->Ki, uPID->Kd);

	//currentTime = HAL_GetTick();
	//elapsedTime = (currentTime - last_time);
	//printf(">%lu ms\r\n", elapsedTime);
	//return (read_status);
}
