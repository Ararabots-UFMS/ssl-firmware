/*
 * linked_list.h
 *
 *  Created on: Oct 23, 2024
 *      Author: allan
 */

#ifndef INC_LINKED_LIST_H_
#define INC_LINKED_LIST_H_

#include "main.h"


typedef struct robot_command {
	uint8_t name;
	uint8_t name_command[32];
	struct robot_command* next;
}robot_command_t;

void append_robot(robot_command_t *head_ref, robot_command_t *new_node);

void delete_robot(robot_command_t *head_ref, uint8_t name);

robot_command_t* find_robot(robot_command_t *head, uint8_t name);

#endif /* INC_LINKED_LIST_H_ */
