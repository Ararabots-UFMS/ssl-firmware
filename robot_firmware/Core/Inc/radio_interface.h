/*
 * radio_interface.h
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#ifndef INC_RADIO_INTERFACE_H_
#define INC_RADIO_INTERFACE_H_

#include "rf24.h"

rf24_dev_t* radio_init(SPI_HandleTypeDef hspi);

void radio_read_and_update(rf24_dev_t* p_dev, command_t *cmd, pidvalues_t *pid);

void get_command_from_buffer(uint8_t *buffer, command_t* result);

#endif /* INC_RADIO_INTERFACE_H_ */
