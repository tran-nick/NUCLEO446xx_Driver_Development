/*
 * 002LED_Button.c
 *
 *  Created on: May 13, 2020
 *      Author: nhon_tran
 */


#include "stm32f446xx.h"

#define HIGH	1
#define LOW		0
#define BTN_PRESSED	LOW


void delay(void)
{
	//changing delay period shorter seems to make button push more stable.
	//too big value, led never turns on.
	for(uint32_t i = 0 ; i < 50000/10 ; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed , GPIOBtn;
 	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLed);


	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay(); //button debouncing
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		}

	}





	return 0;
}
