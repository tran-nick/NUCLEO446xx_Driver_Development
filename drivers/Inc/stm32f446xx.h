/*
 * stm32f446xx.h
 *
 *  Created on: May 6, 2020
 *      Author: nhon_tran
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h> //need to include for notation uint32_t to compile correctly
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/***************************************************************************************************************
 *
 * Arm Cortex Mx Processor NVIC ISERx register addresses
 *
 */
#define NVIC_ISER0		((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10CU)

 /******************************************************************
  * Arm Cortex Mx Processor NVIC ISERx register addresses
 ***************************************************************** */
#define NVIC_ICER0		((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1		((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2		((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3		((__vo uint32_t*)0XE000E18CU)

 /******************************************************************
  * Arm Cortex Mx Processor Priority Address
 ***************************************************************** */
#define NVIC_PR_BASEADDR	((__vo uint32_t*)0xE000E400U)

#define NO_PR_BITS_IMPLEMENTED		4

/******************************************************************
 * Base addresses of flash and SRAM memories
 ******************************************************************/
//compiler treat number as signed by default, need to specify unsigned b/c
//addresses are not signed
#define FLASH_BASEADDR		0x08000000U		//flash base address
#define SRAM1_BASEADDR		0x20000000U		//112KB SRAM1
#define SRAM2_BASEADDR		0x2001C000U		//16 kb SRAM2
#define ROM					0x1FFF0000U		//System memory, Read Only Memory
#define SRAM				SRAM1_BASEADDR

/******************************************************************
 * AHBx and APBx bus peripheral base addresses
***************************************************************** */


#define PERIPH_BASEADDR			0x40000000U			//Why do we define PERIPH_BASE and reuse for APB1 instead of defining APB1 directly?
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR		//APB1 bus base address
#define APB2PERIPH_BASEADDR		0x40010000U			//APB2 bus base address
#define AHB1PERIPH_BASEADDR		0x40020000U			//AHB1 bus base address
#define AHB2PERIPH_BASEADDR		0x50000000U			//AHB2 bus base address
#define AHB3PERIPH_BASEADDR		0xA0001000U			//AHB3 bus base address



/******************************************************************
 * Base addresses of all peripherals which are on AHB1 bus
 ******************************************************************/

//offsets are positive values, no need for U at the end

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)		//GPIO A base address
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)		//GPIO B base address
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)		//GPIO C base address
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)		//GPIO D base address
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)		//GPIO E base address
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)		//GPIO F base address
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)		//GPIO G base address
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)		//GPIO H base address


/******************************************************************
 * Base addresses of all peripherals which are on APB1 bus
 ******************************************************************/

#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000)		//TIMER 2 base address
#define	TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400)		//TIMER 3 base address
#define	TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800)		//TIMER 4 base address
#define	TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00)		//TIMER 5 base address
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000)		//TIMER 6 base address
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400)		//TIMER 7 base address
#define	TIM12_BASEADDR			(APB1PERIPH_BASEADDR + 0x1800)		//TIMER 12 base address
#define	TIM13_BASEADDR			(APB1PERIPH_BASEADDR + 0x1C00)		//TIMER 13 base address
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR + 0x2000)		//TIMER 14 base address

#define RTC_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800)		//Real Time Clock base address

#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00)		//Window Watch Dog base address
#define	IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000)		//Independent Watch Dog base address

#define	SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)		//SPI2 base address -- Serial Peripheral Interface
#define	SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)		//SPI3 base address
#define SPDIFRX_BASEADDR		(APB1PERIPH_BASEADDR + 0x4000)		//SPDIF Receiver base address -- (Sony/Philips Digital Interface)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)		//USART 2 base address
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)		//USART	3 base address
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)		//UART 4 base address -- no "S", doesn't support synchronous communication
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)		//UART 5 base address

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)		//I2C 1 base address
#define	I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)		//I2C 2 base address
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)		//I2C 3 base address

#define CAN1_BASEADDR			(APB1PERIPH_BASEADDR + 0x6400)		//CAN 1 base address
#define CAN2_BASEADDR			(APB1PERIPH_BASEADDR + 0x6800)		//CAN 2 base address

#define HDMICEC_BASEADDR		(APB1PERIPH_BASEADDR + 0x6C00)		//HDMI-CEC Controller base address -- High-Definition Multimedia Interface - Consumer Electronics Control

#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000)		//Power Controller base address

#define DAC_BASEADDR			(APB1PERIPH_BASEADDR + 0x7400)		//Digital-to-analog Converter base address



/******************************************************************
 * Base addresses of all peripherals which are on APB2 bus
***************************************************************** */

#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000)		//TIMER 1 base address
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400)		//TIMER 8 base address

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)		//USART 1 base address
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)		//USART 6 base address

#define ADC1_BASEADDR			(APB2PERIPH_BASEADDR + 0x2000)		//Analog-to-digital converter 1 base address
#define ADC2_BASEADDR			(ADC1_BASEADDR + 0x100)				//ADC 2 base address
#define ADC3_BASEADDR			(ADC1_BASEADDR + 0x200)				//ADC 3 base address

#define SDMMC_BASEADDR			(APB2PERIPH_BASEADDR + 0x2C00)		//Secure Digital I/O base address. SDIO -- Secure Digital Input/Output  MMC -- MultimediaCard

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)		//SPI 1 base address
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)		//SPI 4 base address

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)		//System configuration controller base address

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)		//External Interrupt/Event Controller base address

#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000)		//TIMER 9 base address
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400)		//TIMER 10 base address
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800)		//TIMER 11 base address

#define SAI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x5800)		//SAI 1 base address -- Serial Audio Interface
#define SAI2_BASEADDR			(APB2PERIPH_BASEADDR + 0x5C00)		//SAI 2 base address


/******************************************************************
 * RCC peripheral registers definition
 ******************************************************************/

#define RCC_BASEADDR		0x40023800U
#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)


typedef struct
{
	__vo uint32_t CR;				/*RCC clock control register					Address offset 0x00	*/
	__vo uint32_t PLLCFGR;			/*RCC PLL configuration register				Address offset 0x04	*/
	__vo uint32_t CFGR;				/*RCC clock configuration register				Address offset 0x08	*/
	__vo uint32_t CIR;				/*RCC clock interrupt register					Address offset 0x0C	*/
	__vo uint32_t AHB1RSTR;			/*RCC AHB1 peripheral reset register 			Address offset 0x10	*/
	__vo uint32_t AHB2RSTR;			/*RCC AHB2 peripheral reset register			Address offset 0x14	*/
	__vo uint32_t AHB3RSTR;			/*RCC AHB3 peripheral reset register			Address offset 0x18	*/
	__vo uint32_t RESERVED0;		/*Reserved										Address offset 0x1C	*/
	__vo uint32_t APB1RSTR;			/*RCC APB1 peripheral reset register			Address offset 0x20	*/
	__vo uint32_t APB2RSTR;			/*RCC APB2 peripheral reset register			Address offset 0x24	*/
	__vo uint32_t RESERVED1;		/*Reserved										Address offset 0x28	*/
	__vo uint32_t RESERVED2;		/*Reserved										Address offset 0x2C	*/
	__vo uint32_t AHB1ENR;			/*RCC AHB1 peripheral clock enable register		Address offset 0x30	*/
	__vo uint32_t AHB2ENR;			/*RCC AHB2 peripheral clock enable register		Address offset 0x34	*/
	__vo uint32_t AHB3ENR;			/*RCC AHB3 peripheral clock enable register		Address offset 0x38	*/
	__vo uint32_t RESERVED3;		/*Reserved										Address offset 0x3C	*/
	__vo uint32_t APB1ENR;			/*RCC APB1 peripheral clock enable register		Address offset 0x40	*/
	__vo uint32_t APB2ENR;			/*RCC APB2 peripheral clock enable register		Address offset 0x44	*/
	__vo uint32_t RESERVED4;		/*Reserved										Address offset 0x48	*/
	__vo uint32_t RESERVED5;		/*Reserved										Address offset 0x4C	*/

} RCC_RegDef_t;



/******************************************************************
 * MACROS for  bitfields of RCC clock configuration register (RCC_CFGR)
 *******************************************************************/
#define RCC_CFGR_SW			0		//System clocks switch
#define RCC_CFGR_SWS		2		//System clock switch status
#define RCC_CFGR_HPRE		4		//AHB prescalar
#define RCC_CFGR_PPRE1		10		//APB1 low speed prescalar
#define RCC_CFGR_PPRE2		13		//APB2 high speed prescalar
#define RCC_CFGR_RTCPRE		16		//HSE division factor for RTC clock
#define RCC_CFGR_MCO1		21		//MCO1 prescalar
#define RCC_CFGR_MCO1PRE	24		//MCO1 prescalar
#define RCC_CFGR_MOC2PRE	27		//MCO2 prescalar
#define RCC_CFGR_MCO2		30		//microcontroller clock output 2




/******************************************************************
 * EXTI peripheral registers definition
 ******************************************************************/

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

typedef struct
{
	__vo uint32_t EXTI_IMR;			/*Interrupt mask register						Address offset 0x00	*/
	__vo uint32_t EXTI_EMR;			/*Event mask register							Address offset 0x04	*/
	__vo uint32_t EXTI_RTSR;		/*Rising trigger selection register				Address offset 0x08	*/
	__vo uint32_t EXTI_FTSR;		/*Falling trigger selection register			Address offset 0x0C	*/
	__vo uint32_t EXTI_SWIER;		/*Software interrupt event register 			Address offset 0x10	*/
	__vo uint32_t EXTI_PR;			/*Pending register								Address offset 0x14	*/


}EXTI_RegDef_t;


/******************************************************************
 * SYSCFG peripheral registers definition
***************************************************************** */

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

typedef struct
{
	__vo uint32_t MEMRMP;		/*SYSCFG memory remap register							Address offset 0x00	*/
	__vo uint32_t PMC;			/*SYSCFG peripheral mode configuration register			Address offset 0x04	*/
	__vo uint32_t EXTICR[4];	/*SYSCFG external interrupt configuration register 1-4	Address offset 0x08 0x0C 0x10 0x14*/
		 uint32_t RESERVED1[2];	/*Reserved												Address offset 0x18 0x1C*/
	__vo uint32_t CMPCR;		/*Compensation cell control register 					Address offset 0x20	*/
		 uint32_t RESERVED2[2];	/*Reserved												Address offset 0x24 0x28*/
	__vo uint32_t CFGR;			/*SYSCFG configuration register							Address offset 0x2C	*/


} SYSCFG_RegDef_t;


/******************************************************************
 * Clock ENABLE macros for SYSCNFG peripherals
 ******************************************************************/
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 14 ))


/******************************************************************
 * Clock DISABLE macros for SYSCNFG peripherals
 ******************************************************************/
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~( 1 >> 14 ))


/******************************************************************
 * GPIO peripheral registers definition
***************************************************************** */
#define GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t *)GPIOH_BASEADDR)


typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register					Address offset 0x00	*/
	__vo uint32_t OTYPER;		/*GPIO port output type register			Address offset 0x04	*/
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register			Address offset 0x08	*/
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register	 	Address offset 0x0C	*/
	__vo uint32_t IDR;			/*GPIO port input data register				Address offset 0x10	*/
	__vo uint32_t ODR;			/*GPIO port output data register			Address offset 0x14	*/
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register			Address offset 0x18	*/
	__vo uint32_t LCKR;			/*GPIO port configuration lock register		Address offset 0x1C	*/
	__vo uint32_t AFR[2];		/*GPIO alternate function low register		Address offset 0x20 - 0x24	*/
								//AFR[0} Alt Fn low register pins 0 - 7, AFR[1] Alt Fn high register pins 8 - 15

} GPIO_RegDef_t;


/******************************************************************
 * Clock ENABLE macros for GPIOx peripherals
 ******************************************************************/
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))


/*	https://stackoverflow.com/questions/47780281/when-does-gpio-peripheral-get-its-members-addresses-assigned
 * 	compiler suitable for embedded system on ARM platform is designed to interpret RCC->AHB1ENR, and will initialize structure with member variables
 * 	sequentially in order they appear in structure.
 *
 * 	My understanding is...
 * 	A pointer holds address of object its pointing to... so RCC->AHB1ENR interpreted as (*RCC_BASEADDR).AHB1ENR
 * 	We dereference RCC_BASEADDR which was type casted as pointer to access the struct pointed to by address 0x40023800U, as defined by macro
 * 	My next best guess is equivalent . operator implementation for -> for this compiler knows to add offset of struct variable to have correct address
 *
 *
 * 	Here's another explaination using struct for peripheral registers https://blog.feabhas.com/2019/01/peripheral-register-access-using-c-structs-part-1/
 */

/******************************************************************
 * Clock DISABLE macros for GPIOx peripherals
***************************************************************** */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))


/******************************************************************
 * Clock RESET macros for GPIOx peripherals
***************************************************************** */
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0)) ; (RCC->AHB1RSTR &= ~(1 << 0));} while(0)	//set to 1 to reset, set to 0 otherwise perpetually in reset mode
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1)) ; (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2)) ; (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3)) ; (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4)) ; (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5)) ; (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6)) ; (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7)) ; (RCC->AHB1RSTR &= ~(1 << 7));} while(0)


/******************************************************************
 * port code for SYSCFG of EXTI ownership
***************************************************************** */

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
										(x == GPIOE) ? 4 : \
										(x == GPIOF) ? 5 : \
										(x == GPIOG) ? 6 : \
										(x == GPIOH) ? 7 : 0)


/******************************************************************
 * SPI  peripheral registers definition
 ******************************************************************/

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

typedef struct{

	__vo uint32_t	SPI_CR1;			/*SPI control register 1			Address offset 0x00	*/
	__vo uint32_t	SPI_CR2;			/*SPI control register 2			Address offset 0x04	*/
	__vo uint32_t	SPI_SR;				/*SPI status register				Address offset 0x08	*/
	__vo uint32_t	SPI_DR;				/*SPI data register					Address offset 0x0C	*/
	__vo uint32_t	SPI_CRCPR;			/*SPI CRC polynomial register		Address offset 0x10	*/
	__vo uint32_t	SPI_RXCRCR;			/*SPI RX CRC register				Address offset 0x14	*/
	__vo uint32_t	SPI_TXCRCR;			/*SPI TX CRC register				Address offset 0x18	*/
	__vo uint32_t	SPI_I2SCFGR;		/*SPI_I2S configuration register	Address offset 0x1C	*/
	__vo uint32_t	SPI_I2SPR;			/*SPI_I2S prescaler register		Address offset 0x20	*/



}SPI_RegDef_t;


/******************************************************************
 * Clock ENABLE macros for SPIx peripherals
 ******************************************************************/
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 13))


/******************************************************************
 * Clock DISABLE macros for SPIx peripherals
***************************************************************** */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<13))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<15))


/******************************************************************
 * Clock RESET macros for SPIx peripherals
 *******************************************************************/
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 12)) ; (RCC->APB2RSTR &= ~(1 << 12));} while(0)	//set to 1 to reset, set to 0 otherwise perpetually in reset mode
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14)) ; (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15)) ; (RCC->APB1RSTR &= ~(1 << 15));} while(0)
#define SPI4_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 13)) ; (RCC->APB2RSTR &= ~(1 << 13));} while(0)


/******************************************************************
 * MACROS for initializing bitfields of  SPI control register 1
 *******************************************************************/
#define SPI_CR1_CPHA		0		//Clock phase
#define SPI_CR1_CPOL		1		//Clock polarity
#define SPI_CR1_MSTR		2		//Master selection
#define SPI_CR1_BR			3		//Baud rate control
#define SPI_CR1_SPE			6		//SPI enable
#define SPI_CR1_LSBFIRST	7		//Frame format
#define SPI_CR1_SSI			8		//Internal slave select
#define SPI_CR1_SSM			9		//Software slave management
#define SPI_CR1_RXONLY		10		//Receive only mode enable
#define SPI_CR1_DFF			11		//Data frame format
#define SPI_CR1_CRCNEXT		12		//CRC transfer next
#define SPI_CR1_CRCEN		13		//Hardware CRC calculation enable
#define SPI_CR1_BIDIOE		14		//Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE	15		//Bidirectional data mode enable


/******************************************************************
 * MACROS for initializing bitfields of  SPI control register 2
 *******************************************************************/
#define SPI_CR2_RXDMAEN		0		//Rx buffer DMA enable
#define SPI_CR2_TXDMAEN		1		//Tx buffer DMA enable
#define SPI_CR2_SSOE		2		//SS output enable
#define SPI_CR2_FRF			4		//Frame format
#define SPI_CR2_ERRIE		5		//Error interrupt enable
#define SPI_CR2_RXNEIE		6		//Rx buffer empty interrupt enable
#define SPI_CR2_TXEIE		7		//Tx buffer empty interrupt enable

/******************************************************************
 * MACROS for initializing bitfields of  SPI SR registers
 *******************************************************************/
#define SPI_SR_RXNE			0		//Receive buffer empty
#define SPI_SR_TXE			1		//Transmit buffer empty
#define SPI_SR_CHSIDE		2		//Channel side
#define SPI_SR_UDR			3		//Underrun flag
#define SPI_SR_CRCERR		4		//CRC error flag
#define SPI_SR_MODF			5		//Mode fault
#define SPI_SR_OVR			6		//Overrun flag
#define SPI_SR_BSY			7		//Busy flag
#define SPI_SR_FRE			8		//Frame error


/******************************************************************
 * SPI  peripheral registers definition
 ******************************************************************/

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

typedef struct{

	__vo uint32_t	I2C_CR1;				/*I2C control register 1				Address offset 0x00	*/
	__vo uint32_t	I2C_CR2;				/*I2C control register 2				Address offset 0x04	*/
	__vo uint32_t	I2C_OAR1;				/*I2C own address register 1			Address offset 0x08	*/
	__vo uint32_t	I2C_OAR2;				/*I2C own address register 2			Address offset 0x0C	*/
	__vo uint32_t	I2C_DR;					/*I2C data register						Address offset 0x10	*/
	__vo uint32_t	I2C_SR1;				/*I2C status register 1					Address offset 0x14	*/
	__vo uint32_t	I2C_SR2;				/*I2C status register 2					Address offset 0x18	*/
	__vo uint32_t	I2C_CCR;				/*I2C clock control register			Address offset 0x1C	*/
	__vo uint32_t	I2C_TRISE;				/*I2C Register for max rise time in fast/standard mode		Address offset 0x20	*/
	__vo uint32_t	I2C_FLTR;				/*I2C Register to select digital / analog noise filter on SDA / SCL inputs		Address offset 0x24	*/

}I2C_RegDef_t;


/******************************************************************
 * Clock ENABLE macros for I2Cx peripherals
***************************************************************** */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))


/******************************************************************
 * Clock DISABLE macros for I2Cx peripherals
 ******************************************************************/
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))


/******************************************************************
 * Clock RESET macros for I2Cx peripherals
 *******************************************************************/
#define I2C1_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 21)) ; (RCC->APB2RSTR &= ~(1 << 21));} while(0)	//set to 1 to reset, set to 0 otherwise perpetually in reset mode
#define I2C2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 22)) ; (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 23)) ; (RCC->APB1RSTR &= ~(1 << 23));} while(0)


/******************************************************************
 * MACROS for initializing bitfields of  I2C control register 1
 *******************************************************************/
#define I2C_CR1_PE				0		//Peripheral enable
#define I2C_CR1_SMBUS			1		//SMBus mode
#define I2C_CR1_SMBTYPE			3		//SMBus type
#define I2C_CR1_ENARP			4		//ARP enable
#define I2C_CR1_ENPEC			5		//PEC enable
#define I2C_CR1_ENGC			6		//General call enable
#define I2C_CR1_NOSTRETCH		7		//Clock stretching disable (slave mode)
#define I2C_CR1_START			8		//START generation
#define I2C_CR1_STOP			9		//STOP generation
#define I2C_CR1_ACK				10		//Acknowledge enable
#define I2C_CR1_POS				11		//Acknowledge / PEC Position (for data receoption)
#define I2C_CR1_PEC				12		//Packet error checking
#define I2C_CR1_ALERT			13		//SMBus alert
#define I2C_CR1_SWRST			15		//Software reset


/******************************************************************
 * MACROS for initializing bitfields of  I2C control register 2
 *******************************************************************/
#define I2C_CR2_FREQ		0		//Peripheral clock frequency
#define I2C_CR2_ITERREN		8		// Error interrupt enable
#define I2C_CR2_ITEVTEN		9		// Event interrupt enable
#define I2C_CR2_ITBUFEN		10		//Buffer interrupt enable
#define I2C_CR2_DMAEN		11		//DMA request enable
#define I2C_CR2_LAST		12		//DMA last transfer


/******************************************************************
 * MACROS for initializing bitfields of  I2C status register 1
 *******************************************************************/
#define I2C_SR1_SB				0		//Start bit (master mode)
#define I2C_SR1_ADDR			1		//Address sent (master mode) / matched (slave mode)
#define I2C_SR1_BTF				2		//Byte transfer finished
#define I2C_SR1_ADD10			3		//10-bit header sent (master mode)
#define I2C_SR1_STOPF			4		//Stop detection (slave mode)
#define I2C_SR1_RxNE			6		//Data register not empty (receivers)
#define I2C_SR1_TxE				7		//Data register empty (transmitters)
#define I2C_SR1_BERR			8		//Bus error
#define I2C_SR1_ARLO			9		//Arbitration lost (master mode)
#define I2C_SR1_AF				10		//Acknowledge failure
#define I2C_SR1_OVR				11		//Ovverrun/Underrun
#define I2C_SR1_PECERR			12		//PEC Error in reception
#define I2C_SR1_TIMEOUT			14		//Timeout or Tlow error
#define I2C_SR1_SMBALERT		15		//SMBus alert



/******************************************************************
 * MACROS for initializing bitfields of  I2C status register 2
 *******************************************************************/
#define I2C_SR2_MSL			0		//Master / slave
#define I2C_SR2_BUSY			1		//Bus busy
#define I2C_SR2_TRA				2		//Transmitter / receiver
#define I2C_SR2_GENCALL			4		//General call address (slave mode)
#define I2C_SR2_SMBDEFAULT		5		//SMBus device default address (slave mode)
#define I2C_SR2_SMBHOST			6		//SMBus host header (slave mode)
#define I2C_SR2_DUALF			7		//Dual flag (slave mode)
#define I2C_SR2_PEC				8		//Packet error checking register


/******************************************************************
 * MACROS for initializing bitfields of  I2C clock control register
 *******************************************************************/
#define I2C_CCR_CCR		0		//[11:0] clock control register in Fm/Sm mode (master mode)
#define I2C_CCR_DUTY	14		//Fm mode duty cycle
#define I2C_CCR_FS		15		//I2C master mode selection




/******************************************************************
 * USART  peripheral registers definition
 ******************************************************************/
#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)


typedef struct{

	__vo uint32_t	USART_SR;				/*USART status register							Address offset 0x00	*/
	__vo uint32_t	USART_DR;				/*USART data register							Address offset 0x04	*/
	__vo uint32_t	USART_BRR;				/*USART baud rate register						Address offset 0x08	*/
	__vo uint32_t	USART_CR1;				/*USART control register 1						Address offset 0x0C	*/
	__vo uint32_t	USART_CR2;				/*USART control register 2						Address offset 0x10	*/
	__vo uint32_t	USART_CR3;				/*USART control register 3						Address offset 0x14	*/
	__vo uint32_t	USART_GTPR;				/*USART guard time and prescaler register		Address offset 0x18	*/

}USART_RegDef_t;


/******************************************************************
 * Clock ENABLE macros for USARTx peripherals
 ******************************************************************/
#define USART1_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 4 ))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 18 ))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 19 ))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 20 ))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 5 ))


/******************************************************************
 * Clock DISABLE macros for USARTx peripherals
 ******************************************************************/
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~( 1 << 4 ))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 17 ))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 18 ))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 19 ))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 20 ))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~( 1 << 5 ))


/******************************************************************
 * Clock RESET macros for pUSARTx peripherals
 *******************************************************************/
#define USART1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 4)) ; (RCC->APB2RSTR &= ~(1 << 4));} while(0)	//set to 1 to reset, set to 0 otherwise perpetually in reset mode
#define USART2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 17)) ; (RCC->APB1RSTR &= ~(1 << 17));} while(0)
#define USART3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 18)) ; (RCC->APB1RSTR &= ~(1 << 18));} while(0)
#define UART4_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 19)) ; (RCC->APB1RSTR &= ~(1 << 19));} while(0)
#define UART5_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 20)) ; (RCC->APB1RSTR &= ~(1 << 20));} while(0)
#define USART6_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 4)) ; (RCC->APB2RSTR &= ~(1 << 4));} while(0)

/******************************************************************
 * MACROS for initializing bitfields of  USART status  register
 *******************************************************************/
#define USART_SR_PE				0		//Parity error
#define USART_SR_FE				1		//Framing error
#define USART_SR_NF				2		//Noise detected flag
#define USART_SR_ORE			3		//Overrun error
#define USART_SR_IDLE			4		//IDLE like detected
#define USART_SR_RXNE			5		//Read data register not empty
#define USART_SR_TC				6		//Transmission complete
#define USART_SR_TXE			7		//Transmit data register empty
#define USART_SR_LBD			8		//LIN break detection flag
#define USART_SR_CTS			9		//CTS flag


/******************************************************************
 * MACROS for initializing bitfields of  USART data  register
 *******************************************************************/
#define USART_DR_DR		0		//Date value, bit fields [8:0]


/******************************************************************
 * MACROS for initializing bitfields of  USART baud rate  register
 *******************************************************************/
#define USART_BRR_DIV_Fraction		0		//fraction of USARTDIV, bits 3:0
#define USART_BRR_DIV_Mantissa		4		//mantissa of USARTDIV, bits 15:4


/******************************************************************
 * MACROS for initializing bitfields of  USART control register 1
 *******************************************************************/
#define USART_CR1_SBK			0				//Send break
#define USART_CR1_RWU			1				//Receiver wakeup
#define USART_CR1_RE			2				//Receiver enable
#define USART_CR1_TE			3				//Transmitter enable
#define USART_CR1_IDLEIE		4				//IDLE interrupt enable
#define USART_CR1_RXNEIE		5				//RXNE interrupt enable
#define USART_CR1_TCIE			6				//Transmission complete interrupt enable
#define USART_CR1_TXEIE			7				//TXE interrupt enable
#define USART_CR1_PEIE			8				//PE interrupt enable
#define USART_CR1_PS			9				//Parity selection
#define USART_CR1_PCE			10				//Parity control enable
#define USART_CR1_WAKE			11				//Wakeup method
#define USART_CR1_M				12				//Word length
#define USART_CR1_UE			13				//USART enable
#define USART_CR1_OVER8			15				//Oversampling mode


/******************************************************************
 * MACROS for initializing bitfields of  USART control register 2
 *******************************************************************/
#define USART_CR2_ADD		0			//Address of USART node
#define USART_CR2_LBDL		5			//lin break detection length
#define USART_CR2_LBDIE		6			//LIN break detection interrupt enable
#define USART_CR2_LBCL		8			//Last bit clock pulse
#define USART_CR2_CPHA		9			//Clock phase
#define USART_CR2_CPOL		10			//Clock polarity
#define USART_CR2_CLKEN		11			//Clock enable
#define USART_CR2_STOP		12			//STOP bits
#define USART_CR2_LINEN		14			//LIN mode enable



/******************************************************************
 * MACROS for initializing bitfields of  USART control register 3
 *******************************************************************/
#define USART_CR3_EIE			0			//Error interrupt enable
#define USART_CR3_IRNE			1			//IrDA mode enable
#define USART_CR3_IRLP			2			//IrDA low-power
#define USART_CR3_HDSEL			3			//Half-duplex selection
#define USART_CR3_NACK			4			//Smartcard NACK mode enable
#define USART_CR3_SCEN			5			//Smartcard mode enable
#define USART_CR3_DMAR			6			//DMA enable receiver
#define USART_CR3_DMAT			7			//DMA enable transmitter
#define USART_CR3_RTSE			8			//RTS enable
#define USART_CR3_CTSE			9			//CTS enable
#define USART_CR3_CTSIE			10			//CTS interrupt enable
#define USART_CR3_ONEBIT		11			//One sample bit method enable



/******************************************************************
 * MACROS for initializing bitfields of  USART Guard time and prescaler register
 *******************************************************************/
#define USART_GTPR_PSC		0			//Prescalar value. bits 7:0
#define USART_GTPR_GT		8			//Guard time value, bits 15:8


/******************************************************************
 * IRQ(Interrupt Request) Numbers of STM32F446xx MCU
 ******************************************************************/
#define IRQ_NO_EXTI0		6	//Position No. in vector table
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71





/******************************************************************
 * macros for IRQ Priorities levels 0-15
 * There's 15 because only 4 most significant bits are implemented of
 * 8 bit field width of PR registers in ARM cortex-M processors
 * so that gives us range of 0000 - 1111 (0 - 15)
***************************************************************** */
#define NVIC_IRQ_PR0	0
#define NVIC_IRQ_PR1	1
#define NVIC_IRQ_PR2	2
#define NVIC_IRQ_PR3	3
#define NVIC_IRQ_PR4	4
#define NVIC_IRQ_PR5	5
#define NVIC_IRQ_PR6	6
#define NVIC_IRQ_PR7	7
#define NVIC_IRQ_PR8	8
#define NVIC_IRQ_PR9	9
#define NVIC_IRQ_P10	10
#define NVIC_IRQ_PR11	11
#define NVIC_IRQ_PR12	12
#define NVIC_IRQ_PR13	13
#define NVIC_IRQ_PR14	14
#define NVIC_IRQ_PR15	15


/******************************************************************
 * Generic MACROS
 ******************************************************************/
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET






//https://stackoverflow.com/questions/18903300/c-unknown-type-name-my-structure
//Need to place this include at the end, otherwise compiler see's include "stm32f446xx_gpio_driver.h"
//before GPIO_RegDef_t is defined
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
