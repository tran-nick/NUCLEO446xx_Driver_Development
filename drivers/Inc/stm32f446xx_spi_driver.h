/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: May 30, 2020
 *      Author: nhon_tran
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"



typedef struct{

	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SclkSpeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;

}SPI_Config_t;




typedef struct{

	SPI_RegDef_t*	pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t*		pTxBuffer;		/*To store Tx buffer address	*/
	uint8_t*		pRxBuffer;		/*To store Rx buffer address	*/
	uint32_t		TxLen;			/*To store Tx len	*/
	uint32_t		RxLen;			/*To store Tx len	*/
	uint8_t			TxState;		/*To store Tx state	*/
	uint8_t			RxState;		/*To store Tx state	*/

}SPI_Handle_t;


/******************************************************************
 * @SPI_DevideMode
 * See bitfield 2 of control register 1
***************************************************************** */
#define	SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/******************************************************************
 * @SPI_BusConfig
 * See bitfields 14 - 15 of control register 1
 ******************************************************************/
#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_SIMPLEX_RX	3


/******************************************************************
 * @SPI_SclkSpeed
***************************************************************** */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7



/******************************************************************
 * @SPI_DFF Data frame format
 * see bitfield 11 of control register 1
***************************************************************** */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1


/******************************************************************
 * @CPOL Clock Polarity
 * See bitfield 1 of control register 1
***************************************************************** */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/******************************************************************
 * @CPHA Clock Phase
 * See bitfield 0 of control register 1
***************************************************************** */
#define SPI_CPHA_HIGH	1  //trailing edge data capture
#define SPI_CPHA_LOW	0	//leading edge data capture

/*******************************************************************
 * @SSM Slave Select Management
 * See bitfield 9, control register 1. Software mgmt disable  by default
 ******************************************************************/
#define SPI_SSM_EN	1
#define SPI_SSM_DI	0


/*******************************************************************
 * @SPI related status flags definition
 *
 ******************************************************************/
#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)


/*******************************************************************
 * @SPI Application States
 *
 ******************************************************************/
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2




/*******************************************************************
 * @SPI Application States
 *
 ******************************************************************/
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_CMPLT		4


/*********************************************************************
 * API's supported by this driver
 *********************************************************************
 */

/******************************************************************
 * Peripheral clock setup
 ******************************************************************/
void SPI_PeriCLockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);




/******************************************************************
 * Init & De-init
 ******************************************************************/
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);



/******************************************************************
 * Data send and receive
***************************************************************** */
void SPI_SendData(SPI_RegDef_t* pSPIx , uint8_t* pTxBuffer , uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx ,  uint8_t* pRxBuffer , uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle , uint8_t* pTxBuffer , uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle ,  uint8_t* pRxBuffer , uint32_t Len);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx , uint32_t FlagName);


/******************************************************************
 * IRQ Config and Handling
***************************************************************** */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pHandle);  //Why is this parameter name different?

/******************************************************************
 * Other SPI Handling Functions
***************************************************************** */
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx , uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx , uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx , uint8_t EnorDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx , uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx);
void SPI__CloseTransmission(SPI_Handle_t* pSPIHandle);
void SPI__CloseReception(SPI_Handle_t* pSPIHandle);


/******************************************************************
 * Application Callback
***************************************************************** */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle , uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
