/*
 * 014UART_tx.c
 *
 *  Created on: Jul 22, 2020
 *      Author: nhon_tran
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_usart_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>



//GLOBALS*****************************************************
char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t USART3Handle;

void delay(void);

void USART2_GPIOInit(void)
{
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOC;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Tx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USARTPins);

	//Rx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&USARTPins);
}


void USART2_Init(void)
{

	USART3Handle.pUSARTx = USART3;
	USART3Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART3Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART3Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART3Handle.USART_Config.USART_NoOfStopBits = 1;
	USART3Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART3Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&USART3Handle);

}

void GPIO_ButtonInit(void)
{
	//blue user button on NUCLUEO-446RE board
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}




int main(void)
{
	USART2_GPIOInit();

	USART2_Init();

	GPIO_ButtonInit();

	USART_PeripheralControl(USART3 , ENABLE);

	while(1)
	{
		while( GPIO_ReadFromInputPin(GPIOC , GPIO_PIN_NO_13) );

		delay();	//button debouncing

		USART_SendData(&USART3Handle , (uint8_t*)msg , strlen(msg));

	}

	return 0;

}
