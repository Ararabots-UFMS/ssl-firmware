/*
 * gpio_hal.h
 *
 *  Created on: May 23, 2024
 *      Author: allan
 */

#ifndef INC_GPIO_HAL_H_
#define INC_GPIO_HAL_H_

#include "gpio.h"

class GPIOBase{

	public:
		GPIOBase();
		~GPIOBase();

		static void init(void);

	private:

		static bool isInit;

};

class Dout : public GPIOBase {
	public:
		Dout(	GPIO_TypeDef *GPIOx,
				uint16_t GPIO_Pin,
				GPIO_PinState state = GPIO_PIN_SET);
		~Dout();

		void write(GPIO_PinState state);
		void toggle(void);
		GPIO_PinState read(void);

	private:
		GPIO_TypeDef *_GPIOx;
		uint16_t _GPIO_Pin;

};



#endif /* INC_GPIO_HAL_H_ */
