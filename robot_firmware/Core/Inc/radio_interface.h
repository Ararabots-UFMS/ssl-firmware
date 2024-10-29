/*
 * radio_interface.h
 *
 *  Created on: Sep 18, 2024
 *      Author: allan
 */

#ifndef INC_RADIO_INTERFACE_H_
#define INC_RADIO_INTERFACE_H_

#include "rf24.h"

#include "pid.h"

void radio_read_and_update(rf24_dev_t* p_dev, command_t *cmd, PID_TypeDef *uPID);


#endif /* INC_RADIO_INTERFACE_H_ */
