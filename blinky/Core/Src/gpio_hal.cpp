/*
 * gpio.cpp
 *
 *  Created on: May 23, 2024
 *      Author: allan
 */

#include "gpio_hal.h"

bool GPIOBase::isInit = false;

GPIOBase::GPIOBase(){
	GPIOBase::init();
}

GPIOBase::~GPIOBase(){

}

void  GPIOBase::init(void){

	if (GPIOBase::isInit){
		return;
	}

	MX_GPIO_Init();
	GPIOBase::isInit=true;

}

Dout::Dout(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state)
			: _GPIOx(GPIOx), _GPIO_Pin(GPIO_Pin){
	write(state);
}

Dout::~Dout(){

}

void Dout::write(GPIO_PinState state){
	HAL_GPIO_WritePin(_GPIOx, _GPIO_Pin, state);
}

void Dout::toggle(void){
	HAL_GPIO_TogglePin(_GPIOx, _GPIO_Pin);
}

GPIO_PinState Dout::read(void){
	return HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin);
}
