/*
 * 015UART_case.c
 *
 *  Created on: Jul 23, 2020
 *      Author: nhon_tran
 */



#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_usart_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>



//GLOBALS*****************************************************

//we have 3 different messages that we transmit to arduino
//you can by all means add more messages
char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};

//reply from arduino will be stored here
char rx_buf[1024] ;

//This flag indicates reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = 0;

extern void initialise_monitor_handles();

USART_Handle_t USART3Handle;

void delay(void);

void USART2_GPIOInit(void)
{
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Tx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USARTPins);

	//Rx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USARTPins);
}


void USART2_Init(void)
{

	USART3Handle.pUSARTx = USART1;
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
	uint32_t cnt = 0;


	initialise_monitor_handles();

	USART2_GPIOInit();
	USART2_Init();

	USART_IRQInterruptConfig(IRQ_NO_USART1 , ENABLE);

	USART_PeripheralControl(USART1, ENABLE);

	printf("Application is running\n");

	//do forever
	while(1)
	{
		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC , GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//First lets enable the reception in interrupt mode
		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT(&USART3Handle , rx_buf , strlen(msg[cnt])) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
		USART_SendData(&USART3Handle , (uint8_t*)msg[cnt] , strlen(msg[cnt]));

		printf("Transmitted : %s\n",msg[cnt]);


		//Now lets wait until all the bytes are received from the arduino .
		//When all the bytes are received rxCmplt will be SET in application callback
		while(rxCmplt != SET);

		//just make sure that last byte should be null otherwise %s fails while printing
		rx_buf[strlen(msg[cnt])+ 1] = '\0';

		//Print what we received from the arduino
		printf("Received    : %s\n",rx_buf);

		//invalidate the flag
		rxCmplt = RESET;

		//move on to next message indexed in msg[]
		cnt ++;
	}

	return 0;
}


void USART2_IRQHandler(void)
{
	USART_IRQHandling(&USART3Handle);
}


void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
