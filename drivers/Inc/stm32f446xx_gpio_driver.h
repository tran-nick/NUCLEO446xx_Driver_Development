/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: May 9, 2020
 *      Author: nhon_tran
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for a GPIO pin. These are the configurable items of GPIO for user application
 */
typedef struct{

	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;				/*Possible values @GPIO_PIN_MODES*/
	uint8_t	GPIO_PinSpeed;				/*Possible values @GPIO_SPEED*/
	uint8_t	GPIO_PinPuPdControl;		/*Possible values @GPIO_PIN_OP_TYPE*/
	uint8_t	GPIO_PinOPType;				/*Possible values @GPIO_PIN_IP_TYPE*/
	uint8_t	GPIO_PinAltFunMode;			/*Possible values @GPIO_PIN_MODES*/

} GPIO_PinConfig_t ;


/*
 * Handle structure for GPIO pin
 */
typedef struct
{

	GPIO_RegDef_t*		pGPIOx;					// pointer to hold base address of GPIO peripheral
	GPIO_PinConfig_t	GPIO_PinConfig;			// holds GPIO pin configuration settings

} GPIO_Handle_t;


/*@GPIO_PIN_NUMBERS
 *
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*@GPIO_PIN_MODES
 *
 * GPIO pin possible modes.
 * We create macros in driver header file because modes are peripheral specific
 * Not appropriate to define in MCU header file
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		//interrupt falling edge
#define GPIO_MODE_IT_RT		5		//interrupt rising edge
#define GPIO_MODE_IT_RFT	6		//interrupt rising/falling edge

/*@GPIO_PIN_OP_TYPE
 *
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*@GPIO_SPEED
 *
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*@GPIO_PIN_IP_TYPE
 *
 * GPIO pin pull up pull down configuration macros
 */
#define GPIO_NO_PUPD	0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2



/**************************************************************************************
 * APIs supported by this driver
 * Function prototypes of functionality supported by this driver
 **************************************************************************************/
/*
 * Peripheral clock setup
 */
void GPIO_PeriCLockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/*
 * Initialize & De-initialize
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Data R/W
 */

/*
 * no bool type in older version of C, so uint8_t 0 or 1 could effectively represent TRUE / FALSE
 * However with modern C / C++, bool is more meaningful so would be better to use instead
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx); //port is 16 pins, need uint16_t
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value); //port is 16 pins, need uint16_t for value
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * IRQ config and handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);










#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
