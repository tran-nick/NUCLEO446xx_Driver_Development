/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Jul 9, 2020
 *      Author: nhon_tran
 */


#include "stm32f446xx_i2c_driver.h"





void delay(void)
{
	//changing delay period shorter seems to make button push more stable.
	//too big value, led never turns on.
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}



/**************************************************
 * Function Prototypes
 * compiler cross check function signatures before calling
 * also prevent errors of calling function before being defined
**************************************************/
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );
uint32_t  RCC_GetPLLOutputClock(void);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

/**************************************************
 * Private Helper Functions
**************************************************/


void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_START);
}



static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->I2C_DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->I2C_DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;

	//verifying device mode
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		//master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//only reading 1 byte
			if(pI2CHandle->RxSize == 1)
			{
				I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_DISABLE);	//disable ACKing

				//clear ADDR flag
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void) dummy_read;	//cast to void to avoid compiler unused error

			}
			else
			{
				//clear ADDR flag
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void) dummy_read;	//cast to void to avoid compiler unused error
			}
		}

		else
		{
			//slave mode
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void) dummy_read;	//cast to void to avoid compiler unused error
		}
	}
}


void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_STOP);
}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx , I2C_ACK_DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;	//read
		pI2CHandle->pRxBuffer++;	//move buffer
		pI2CHandle->RxLen--;		//decrement length
	}

	if(pI2CHandle->RxLen  == 0)
	{
		//close reception and inform application
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//close Rx
		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle)
{
	if(pI2CHandle->TxLen > 0 )
	{
		//1. Send data to DR data register
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

		//2. Decrease TxLen
		pI2CHandle->TxLen--;

		//3. Increment buffer address
		pI2CHandle->pTxBuffer++;
	}
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//disable interrupt control bits
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//reset handle parameters
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->pRxBuffer = NULL;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_ENABLE);
	}
}



void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//disable interrupt control bits
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//reset handle parameters
	pI2CHandle->TxRxState = I2C_READY;
	//dont need to reset TxLen because it will already be 0 from sending data, e.g. SR1 TXE = 1
	pI2CHandle->pTxBuffer = NULL;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_ENABLE);
	}
}



/**************************************************
 * Peripheral Clock setup
**************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}




/**************************************************
 * Init and De-init
**************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	I2C_PeriClockControl(pI2CHandle->pI2Cx , ENABLE);
	I2C_PeripheralControl(pI2CHandle->pI2Cx , ENABLE);

	uint32_t tempreg = 0;

	//Configure CR1 registers -- ACK enable
	I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_ENABLE);


	//Configure CR2 registers -- FREQ
	 tempreg = 0;
	 tempreg |= RCC_GetPCLK1Value() / 1000000U;
	 pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);	//bitmasking for first 5 bits


	 //Configure OAR1 registers
	 tempreg = 0;	//reset before writing
	 tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	 tempreg |= ( 1 << 14);	//maintain at 1 per reference manual
	 pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	 //Configure CCR registers
	 uint16_t ccr_value = 0;
	 tempreg = 0;

	 if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM	)
	 {
		 //standard mode
		 ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		 tempreg |= (ccr_value & 0xFFF);
	 }
	 else
	 {
		 //fast mode
		tempreg |= ( 1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_SCL_SPEED_FM2K )
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempreg |= (ccr_value & 0xFFF);
	 }

	 pI2CHandle->pI2Cx->I2C_CCR |= tempreg;


	 //Configure TRISE registers
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM )
	{
		//standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1 ;	//divide by 1 M
	}
	else
	{
		//fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300 )/ 1000000U) + 1 ;	//divide by 1 M
	}

	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/**************************************************
 * Data Send and Receive
**************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RepeatStart)
{
	//1. Generate START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Verify START condition by checking SB flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_SB));

	//3. Send slave address and R/W bit
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx ,SLAVE_ADDR);

	//Verify slave address by checking ADDR bit in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_ADDR));

	//Why does ADDR bit stay 1 when running code full speed? But bit ADDR
	//bit is cleared when stepping through function during debug?

	//5. Clear ADDR flag by reading SR1 & SR2
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send data until Len = 0
	while( Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_TXE));	//hand until data register empty

		pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;
		pTxbuffer++;
		Len--;

	}

	//7. Wait for TxE = 1 and BTF = 1 before generation stop condition
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_BTF));


	//8. Generate STOP condition
	if(RepeatStart == I2C_DISABLE_SR )
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr , uint8_t RepeatStart)
{
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Verify start condition by reading SB status
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_SB));

	//3. Slave address with R/W bit
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx ,SLAVE_ADDR);

	//4. Wait until address phase complete, check ADDR SR bit
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_ADDR));

	//reading only 1 byte
	if(Len == 1)
	{
		//disable ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE is 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_RXNE));

		//generate STOP condition
		if(RepeatStart == I2C_DISABLE_SR )
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->I2C_DR;	//reads 1 byte
	}


	//reading more than 1 byte
	if(Len > 1)
	{

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read data until Len become zero
		for(uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RxNE is 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_RXNE));

			if(i == 2)	//when Len == 2
			{
				//clear ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_DISABLE);

				//generate STOP condition
				if(RepeatStart == I2C_DISABLE_SR )
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->I2C_DR;	//reads 1 byte

			//increment buffer address
			pRxbuffer++;
		}
	}

	//re-enable ACKing if thats what user config is set to
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE )
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx ,I2C_ACK_ENABLE);
	}
}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RepeatStart)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	//check if busy in Tx / Rx
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = RepeatStart;

		//Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		/*******************************
		 * set all interrupt configs
		 *********************************/

		//Enable ITBUFFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2	|= ( 1 << I2C_CR2_ITBUFEN );

		//Enable ITEVFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2	|= ( 1 << I2C_CR2_ITEVTEN );

		//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2	|= ( 1 << I2C_CR2_ITERREN );

		/*
		 * Will generate SB event interrupt and call IRQ_EV_IRQHandling interrupt service routine
		 */
	}

	return busystate;

}


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t RepeatStart)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = RepeatStart;

		//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable ITBUFFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2	|= ( 1 << I2C_CR2_ITBUFEN );

		//Enable ITEVFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2	|= ( 1 << I2C_CR2_ITEVTEN );

		//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2	|= ( 1 << I2C_CR2_ITERREN );
	}

	return busystate;


}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx , uint8_t data)
{
	pI2Cx->I2C_DR = data;	//load data into DR to send

}


uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx )
{
	return ((uint8_t)pI2Cx->I2C_DR);	//read and return
}


/**************************************************
 * Other Peripheral Control APIs
**************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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


void IRQ_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1 , temp2 , temp3;

	//1. Handle interrupt for SB event
	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);

	if( temp1 && temp3)
	{
		//SB flag is set -- per reference manual applicable Master mode only
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx ,pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx ,pI2CHandle->DevAddr);
		}

	}

	//2. Handle interrupt for ADDR event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag set
		I2C_ClearADDRFlag(pI2CHandle);

	}



	//3. Handle event for BTF event

	//during debug step through, BTF shows 0, but running full speed BTF is 1
	//NOTE: see ERRATA for I2C limitation, mismatch in I2C sampling rate to data set-up time may result in undesired behavior.

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF flag set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_TxE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					//when BTF, TXE = 1, close transmission
					if(pI2CHandle->Sr == I2C_DISABLE_SR )
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset member elements of interrupt handle structure
					I2C_CloseSendData(pI2CHandle);

					//3. inform app tranmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		//during reception
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;	//do nothing
		}
	}


	//4. Handle event for STOPF event -- only applicable in slave mode.
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF flag set, clear by reading SR1, write to CR1. Reading SR1 handled by temp3 already.
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;	//writes to CR1, but doesn't change any bitfields

		//inform application level
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}


	//5. Handle event for TXE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE);
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
		{
			//verify if master
			//TXE flag set
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE))
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//slave mode
			//Verify that slave is in transmitter mode
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}


	//6. Handle event for RXNE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE);
	if(temp1 && temp3)
	{
		if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))	//verify if master
		{
			//RXNE flag set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//Verify that slave is in transmitter mode
			if( ! (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA) ) )
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}


void IRQ_ERR_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1 , temp2;


	temp1 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);

	//1. Handle Bus Error interrupt
	temp2 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//clear BERR
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);	//cleared by software per reference manual

		//inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);

	}

	//2. Handle Arbitration Loss interrupt
	temp2 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		//clear ARLO
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);	//cleared by software per reference manual

		//inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//3. Handle AF interrupt
	temp2 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		//clear AF
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);	//cleared by software per reference manual

		//inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//4. Handle Overrun interrupt
	temp2 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		//clear Overrun
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);	//cleared by software per reference manual

		//inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}


	//5. Handle Time out interrupt
	temp2 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		//clear Time Out
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);	//cleared by software per reference manual

		//inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}


void I2C_SlaveManageCallBackEvents(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
	}
	else
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	}

}

/**************************************************
 * Other Peripheral Control APIs
**************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{

	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_PE);
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}
	else if(EnorDi == I2C_ACK_DISABLE)
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK); //clear
	}
}


/**************************************************
 * Application Callback
**************************************************/
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{

}
