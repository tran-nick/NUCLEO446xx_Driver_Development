/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: May 30, 2020
 *      Author: nhon_tran
 */


#include "stm32f446xx_spi_driver.h"



/******************************************************************
 * Peripheral clock setup
 ******************************************************************/
void SPI_PeriCLockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}

	}


}


/******************************************************************
 * Init & De-init
 ******************************************************************/
void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	//enable peripheral clock
	SPI_PeriCLockControl(pSPIHandle->pSPIx,ENABLE);

//Configuring all settable variable handled in the SPI_Config_t structure

	//1.Configure device mode (master / slave)
	uint32_t temp = 0;

	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode cleared
		temp &= ~( 1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode set
		temp |= ( 1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		//BIBI mode must be cleared and RXONLY bit set
		temp &= ~( 1 << SPI_CR1_BIDIMODE);
		temp |= ( 1 << SPI_CR1_RXONLY);

	}

	//3. Configure clock speed
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure DFF
	temp |=  pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure CPOL
	temp |=  pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure CPHA
	temp |=  pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;


	temp |=  pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//can use assignment because first instance of initializing CR1 registers
	pSPIHandle->pSPIx->SPI_CR1 = temp;

}




void SPI_DeInit(SPI_RegDef_t* pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

}



uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx , uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/******************************************************************
 * Data send and receive
 ******************************************************************/
void SPI_SendData(SPI_RegDef_t* pSPIx , uint8_t* pTxBuffer , uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until Txe is set
		while(SPI_GetFlagStatus(pSPIx , SPI_TXE_FLAG) == FLAG_RESET);

		//2. checking DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//typecasting to pointer to uint16_t
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}


}



void SPI_ReceiveData(SPI_RegDef_t* pSPIx ,  uint8_t* pRxBuffer , uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx , SPI_RXNE_FLAG) == FLAG_RESET);

		//2. checking DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//typecasting to pointer to uint16_t
			//loading the data from DR to RxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->SPI_DR;
			Len--;
			pRxBuffer++;
		}
	}


}

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle , uint8_t* pTxBuffer , uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	//check if peripheral not busy sending data
	if(state != SPI_BUSY_IN_TX)
	{

		//1. save Tx buffer address and length information in variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Make SPI busy in transmission so that no other code will take over SPI peripheral during data Tx
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE  control bit to get an interrupt whenever TXE status flag set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_TXEIE);

		//4. Data transmissions will be taken care of by interrupt service routine code


	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle ,  uint8_t* pRxBuffer , uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	//if SPI peripheral is busy receiving, it cannot send data. Check if peripheral is free.
	if(state != SPI_BUSY_IN_RX)
	{

		//1. save Tx buffer address and length information in variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Make SPI busy in transmission so that no other code will take over SPI peripheral during data Tx
		pSPIHandle->TxState = SPI_BUSY_IN_RX;

		//3. Enable TXEIE  control bit to get an interrupt whenever TXE status flag set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_RXNEIE);

		//4. Data transmissions will be taken care of by interrupt service routine code


	}

	return state;


}



/**************************************************
 * IRQ Config and Handling
**************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//calc IPR register, reference Cortex Generic User Guide Section 4.2.7
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//IPRx * 4 because each IPRx is 32 bit wide, and IPRx is uint8_t
	//Also reference PM0214 from ST, Section 4.3.7 Table 47 processor only implements
	//bits [7:4]
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
use static to denote that these are private helper functions
doesnt allow for application level to call these functions
*/
static void spi_txe_interrupt_handle(SPI_Handle_t* pSPIHandle)
{

	//Checking DFF bit, code reused from RecieveData function
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF

		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( ! pSPIHandle->TxLen)
	{
		//Check if TxLen == 0 and close SPI transmissions and report Tx is over
		SPI__CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_TX_CMPLT);
	}
}



static void spi_rxne_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	//Checking DFF bit, code reused from RecieveData function
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF

		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;	//twice, because 16 bit, decrement by 2 bytes
	}
	else
	{
		//8 bit DFF
		*pSPIHandle->pTxBuffer = (uint8_t)pSPIHandle->pSPIx->SPI_DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if( ! pSPIHandle->RxLen)
	{
	SPI__CloseReception(pSPIHandle);
	SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_RX_CMPLT);
	}
}



static void spi_ovr_interrupt_handle(SPI_Handle_t* pSPIHandle)
{

	//1. Clear the OVR flag
	uint8_t temp;

	//check if peripheral is busy in Tx
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{

		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void) temp;
	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_OVR_ERR);
}


void SPI_IRQHandling(SPI_Handle_t* pSPIHandle)
{

	uint8_t temp1 , temp2;

	temp1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2 )
	{
		//Handles TXE interrupt
		spi_txe_interrupt_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2 )
	{

		//Handle RNXE interrupt
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//CRCERR, MODF, FRE interrupt dont need to worry about right now

	//Checking for OVR interrupt
	temp1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2 )
	{

		//Handle RNXE interrupt
		spi_ovr_interrupt_handle(pSPIHandle);
	}


}



/**************************************************
 * Other SPI Handling API's
**************************************************/
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}


/*
 * Sets SSI in CR1 to ENABLE to prevent MODF fault
 */
void SPI_SSIConfig(SPI_RegDef_t* pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}



/*
 * Sets SSOE in CR2
 */
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_SSOE);
		}


}



void SPI__CloseTransmission(SPI_Handle_t* pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_TXEIE); //clear TXEIE bit
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI__CloseReception(SPI_Handle_t* pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_RXNEIE); //clear TXEIE bit
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}


void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx)
{
	uint8_t temp;

	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;

	(void) temp;

}


__weak void SPI__ApplicationEventCallback(SPI_Handle_t *pSPIHandle , uint8_t AppEv)
{
	//function is suppose to be overridden by user
}
