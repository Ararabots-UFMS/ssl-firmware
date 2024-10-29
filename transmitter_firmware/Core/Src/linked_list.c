/*
 * linked_list.c
 *
 *  Created on: Oct 23, 2024
 *      Author: allan
 */

#include <stdlib.h>
#include "linked_list.h"

// Function to add a node at the end of the list
void append_robot(robot_command_t *head_ref, robot_command_t *new_node) {
	// Allocate memory for the new node
	robot_command_t *last = head_ref;  // Used to traverse the list

	// Assign data to the new node

	// If the list is empty, make the new node the head
	if (head_ref == NULL) {
		head_ref = new_node;
		return;
	}

	// Traverse the list to find the last node
	while (last->next != NULL) {
		last = last->next;
	}

	// Change the next of the last node to the new node
	last->next = new_node;
}

// Function to delete a node with a given key
void delete_robot(robot_command_t *head_ref, uint8_t name) {
	robot_command_t *temp = head_ref;
	robot_command_t *prev = NULL;

	// If the head node itself holds the key to be deleted
	if (temp != NULL && temp->name == name) {
		head_ref = temp->next;  // Change head
		free(temp);  // Free old head
		return;
	}

	// Search for the key to be deleted, keep track of the previous node
	while (temp != NULL && temp->name != name) {
		prev = temp;
		temp = temp->next;
	}

	// If the key was not present in the list
	if (temp == NULL)
		return;

	// Unlink the node from the linked list
	prev->next = temp->next;

	free(temp);  // Free memory
}

robot_command_t* find_robot(robot_command_t *head, uint8_t name) {
	robot_command_t *current = head;

	while (current != NULL) {
		if (current->name == name) {
			return (current);
		}
		current = current->next;
	}

	return NULL;  // Value not found
}
