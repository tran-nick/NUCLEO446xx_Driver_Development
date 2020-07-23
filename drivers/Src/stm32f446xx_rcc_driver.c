/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Jul 22, 2020
 *      Author: nhon_tran
 */


#include "stm32f446xx_rcc_driver.h"



uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB_PreScaler[4] = {2,4,8,16};



uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1 , SysClk;

	uint8_t clksrc , temp , ahbp , apb1 ;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SysClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SysClk = 8000000;
	}
	else if(clksrc == 2)
	{
		SysClk = RCC_GetPLLOutputClock();
	}


	//AHBP
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = APB_PreScaler[temp - 8];
	}


	//APB1
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB_PreScaler[temp - 4];
	}

	pclk1 = ((SysClk / ahbp) / apb1);

	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2 , SysClk;

	uint8_t clksrc , temp , ahbp , apb2 ;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SysClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SysClk = 8000000;
	}
	else if(clksrc == 2)
	{
		SysClk = RCC_GetPLLOutputClock();
	}


	//AHBP
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)	//0xxx: system clock not divided
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}


	//APB2
	temp = ((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7);
	if(temp < 4)	//0xx: AHB clock not divided
	{
		apb2 = 1;
	}
	else
	{
		apb2 = APB_PreScaler[temp - 4];
	}

	pclk2 = ((SysClk / ahbp) / apb2);

	return pclk2;
}


uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}

