/*
 * radio_interface.c
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */


#include "rf24.h"
#include "main.h"

#define PAYLOAD_SIZE 32			//size in bytes for radio communication

rf24_dev_t* radio_init(SPI_HandleTypeDef hspi){

	rf24_dev_t device; /* Module instance */
	rf24_dev_t* p_dev = &device; /* Pointer to module instance */

	/* Device config */

	/* Get default configuration */
	rf24_get_default_config(p_dev);

	/* The SPI2 was chosen in Cube */
	p_dev->platform_setup.hspi = &hspi;

	/* CSN on pin PC6 */
	p_dev->platform_setup.csn_port = RADIO_CSN_GPIO_Port;
	p_dev->platform_setup.csn_pin = RADIO_CSN_Pin;

	/* IRQ on pin PC7 */
	p_dev->platform_setup.irq_port = RADIO_IRQ_GPIO_Port;
	p_dev->platform_setup.irq_pin = RADIO_IRQ_Pin;

	/* CE on pin PC8 */
	p_dev->platform_setup.ce_port = RADIO_CE_GPIO_Port;
	p_dev->platform_setup.ce_pin = RADIO_CE_Pin;

	p_dev->payload_size = PAYLOAD_SIZE;


	while(rf24_init(p_dev)!=RF24_SUCCESS);

	uint8_t addresses[2][5] = {{0xE7, 0xE7, 0xE7, 0xE7, 0xE8}, {0xC2, 0xC2, 0xC2, 0xC2, 0xC1}};

	//No idea what output_power should be
	//rf24_set_output_power(p_dev, output_power);

	rf24_status_t device_status; /* Variable to receive the statuses returned by the functions */

	device_status = rf24_open_writing_pipe(p_dev, addresses[0]);

	if (device_status == RF24_SUCCESS){
		device_status = rf24_open_reading_pipe(p_dev, 1, addresses[1]);}

	if (device_status == RF24_SUCCESS){
		device_status = rf24_start_listening(p_dev);}

	#ifdef VERBOSE
		if (device_status != RF24_SUCCESS){
			printf("Error during nrf24 setup");
		}
	#endif

	return p_dev;
}

void get_command_from_buffer(uint8_t *buffer, command_t* result){

	union {
		uint8_t ints[4];
		float value;
	} uint8_to_float;

	uint8_to_float.ints[3] = buffer[1];
	uint8_to_float.ints[2] = buffer[2];
	uint8_to_float.ints[1] = buffer[3];
	uint8_to_float.ints[0] = buffer[4];
	result->des_vx = uint8_to_float.value;

	uint8_to_float.ints[3] = buffer[5];
	uint8_to_float.ints[2] = buffer[6];
	uint8_to_float.ints[1] = buffer[7];
	uint8_to_float.ints[0] = buffer[8];
	result->des_vy = uint8_to_float.value;

	uint8_to_float.ints[3] = buffer[9];
	uint8_to_float.ints[2] = buffer[10];
	uint8_to_float.ints[1] = buffer[11];
	uint8_to_float.ints[0] = buffer[12];
	result->des_orient = uint8_to_float.value;

	uint8_to_float.ints[3] = buffer[13];
	uint8_to_float.ints[2] = buffer[14];
	uint8_to_float.ints[1] = buffer[15];
	uint8_to_float.ints[0] = buffer[16];
	result->cur_orient = uint8_to_float.value;

	result->kik_sig = buffer[17] & 0b00000001;
}

void radio_read_and_update(rf24_dev_t* p_dev, command_t *cmd, pidvalues_t *pid){
	uint8_t buffer[PAYLOAD_SIZE];
	rf24_status_t device_status;
	rf24_status_t read_status ;

	while ((device_status = rf24_available(p_dev, NULL)) == RF24_SUCCESS) {
		read_status = rf24_read(p_dev, buffer, p_dev->payload_size);
	}

	if (read_status == RF24_SUCCESS){
		switch (buffer[0]) {
		case 101:
			//Command
			get_command_from_buffer(buffer, cmd);
			break;
		case 102:
			//pid
			break;
		default:
			#ifdef VERBOSE
				printf("Could not identify message recieved")
			#endif
			break;
		}
	}

	printf("%s\r\n", buffer);
}
