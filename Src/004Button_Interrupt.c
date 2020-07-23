/*
 * 003LED_ExtButton.c
 *
 *  Created on: May 15, 2020
 *      Author: nhon_tran
 */



#include <string.h>
#include "stm32f446xx.h"

#define HIGH	1
#define LOW		0
#define BTN_PRESSED	LOW


void delay(void)
{
	//changing delay period shorter seems to make button push more stable.
	//too big value, led never turns on.
	for(uint32_t i = 0 ; i < 50000/10; i++);
}



int main(void)
{

	GPIO_Handle_t GpioLed , GPIOBtn;
	memset(&GpioLed , 0, sizeof(GpioLed));
	memset(&GPIOBtn , 0, sizeof(GPIOBtn));

	//LED Output
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLed);


	//BUTTON
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_Init(&GPIOBtn);



	//IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5 , ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5 ,NVIC_IRQ_PR15 );



	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay(); //for switch debouncing
	GPIO_IRQHandling(GPIO_PIN_NO_8);
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_0);
}

