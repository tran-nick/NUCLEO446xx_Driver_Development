/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Jul 22, 2020
 *      Author: nhon_tran
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t  RCC_GetPLLOutputClock();


#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
