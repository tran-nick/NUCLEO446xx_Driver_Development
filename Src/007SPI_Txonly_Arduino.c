/*
 * 007SPI_Txonly_Arduino.c
 *
 *  Created on: Jun 24, 2020
 *      Author: nhon_tran
 */


/*
 *
 * Alternate Function 5
 * PB12 - SPI_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 */
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include <string.h>

void SPI2_GPIOInits(void)
{
		GPIO_Handle_t SPIPins;

		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		//SCLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPins);

		//MISO
		//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		//GPIO_Init(&SPIPins);


		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPIPins);


}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generates sclk of 2MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);

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

void delay(void)
{
	//changing delay period shorter seems to make button push more stable.
	//too big value, led never turns on.
	for(uint32_t i = 0 ; i < 500000/3 ; i ++);
}


int main(void)
{

	char user_data[] = "Hello World";

	//initialize button for sending SPI data
	GPIO_ButtonInit();

	//Initialize GPIO pins to enable SPI2 functionality
	SPI2_GPIOInits();

	//Initialize SPI2 peripheral parameters
	SPI2_Inits();

	//Enable SSOE to enable NSS output
	SPI_SSOEConfig(SPI2, ENABLE);


	while(1)
	{

	while(GPIO_ReadFromInputPin(GPIOC , GPIO_PIN_NO_13));

	delay(); //button debouncing

	//Enable SPI2 peripheral after all config is done
	SPI_PeripheralControl(SPI2 , ENABLE);


	//sending length information
	uint8_t dataLen = strlen(user_data);
	SPI_SendData(SPI2 , &dataLen , 1 );


	//send data
	SPI_SendData(SPI2, (uint8_t*)user_data , strlen(user_data) );


	//checks if SPI is busy before disabling peripheral
	while( SPI_GetFlagStatus(SPI2 , SPI_BUSY_FLAG));

	//disable after data sent
	SPI_PeripheralControl(SPI2 , DISABLE);


	}

	return 0;
}
