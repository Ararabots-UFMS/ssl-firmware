/*
 * led_handler.h
 *
 *  Created on: Jul 10, 2024
 *      Author: allan
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32f1xx_hal.h"

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

#endif /* INC_LED_H_ */
