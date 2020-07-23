/*
 * 006SPI_Tx_Testing.c
 *
 *  Created on: Jun 8, 2020
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
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);


}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		//serial clock of 8 MHz
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;	//software slave management enabled.

	SPI_Init(&SPI2Handle);	//parameter is pointer, so we pass address.

}


int main(void)
{

	char user_data[] = "Hello World";

	//Initialize GPIO pins to enable SPI2 functionality
	SPI2_GPIOInits();

	//Initialize SPI2 peripheral parameters
	SPI2_Inits();

	//Configuration of SSI to prevent MODF error
	SPI_SSIConfig(SPI2 , ENABLE);

	//Enable SPI2 peripheral after all config is done
	SPI_PeripheralControl(SPI2 , ENABLE);

	//send data
	SPI_SendData(SPI2, (uint8_t*)user_data , strlen(user_data) );

	//disable after data sent
	//SPI_PeripheralControl(SPI2 , DISABLE);

	while(1);

	return 0;
}
