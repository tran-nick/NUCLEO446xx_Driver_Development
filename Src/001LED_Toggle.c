/*
 * 001LED_Toggle.c
 *
 *  Created on: May 12, 2020
 *      Author: nhon_tran
 */
#include "stm32F446xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 50000 ; i++);
}



int main(void)
{
	GPIO_Handle_t GpioLed;
 	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}


	return 0;
}
