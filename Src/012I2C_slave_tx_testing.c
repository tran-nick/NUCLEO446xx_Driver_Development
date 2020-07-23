/*
 * 012I2C_slave_tx_testing.c
 *
 *  Created on: Jul 15, 2020
 *      Author: nhon_tran
 */



#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MY_ADDR	0x68
#define SLAVE_ADDR MY_ADDR

//prototyping semihosting function
extern void initialise_monitor_handles();


//global variable, so we can access in main function
I2C_Handle_t I2C1handle;

//sample data to send
//arduino wire library does not accept more than 32 bytes per I2C transaction
uint8_t tx_buffer[32] = "STM32 slave mode testing.";



void I2C_GPIOInits(void)
{
		GPIO_Handle_t I2CPins;

		I2CPins.pGPIOx = GPIOB;
		I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
		I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		//SCLK
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&I2CPins);

		//SDA
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
		GPIO_Init(&I2CPins);


}


void I2C_Inits(void)
{

	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;		//reference I2C specification for reserved addresses
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;	//not used, placeholder value
	I2C1handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;	//standard mode, 100 kHz


	I2C_Init(&I2C1handle);

}



int main(void)
{

	initialise_monitor_handles();	//use printf statements after this line.

	printf("Application is running\n");

	//GPIO pin initialization
	I2C_GPIOInits();

	//peripheral initialization
	I2C_Inits();

	//IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV , ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER , ENABLE);

	I2C_SlaveManageCallBackEvents(I2C1, ENABLE);	//enable/disable interrupt control bits

	while(1);

}



void I2C1_EV_IRQHandler(void)
{
	IRQ_EV_IRQHandling(&I2C1handle);
}


void I2C1_ER_IRQHandler(void)
{
	IRQ_ERR_IRQHandling(&I2C1handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t commandcode = 0;
	static uint8_t count = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//send master data
		if(commandcode == 0x51)
		{
			//send data length
			I2C_SlaveSendData(pI2CHandle->pI2Cx , strlen((char*)tx_buffer));
		}
		else if(commandcode == 0x52)
		{
			//send data in tx_buffer
			I2C_SlaveSendData(pI2CHandle->pI2Cx , tx_buffer[count++]);
		}
	}
	else if(AppEv == I2C_EV_DATA_RCV)
	{
		//read data from master
		commandcode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		//NACK received, master doesnt want more data sent
		commandcode = 0xff;	//essentially setting to null
		count = 0;	//reset
	}
	else if(AppEv == I2C_EV_STOP)
	{
		//master has ended I2C comm with slave -- only occur during slave Rx
	}

}
