/*!****************************************************************************
 * @file    	gpio.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	22.11.2016
 * @brief    	gpio driver for stm32 G0 microcontroller
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <stddef.h>
#include "stm32g0xx.h"
#include "bitbanding.h"
#include "board.h"
#include "gpio.h"

/*!****************************************************************************
* MEMORY
*/
pinMode_type const pinsMode[] = {
	/*0 */	makepin(GPIOC,	15,		outPushPull,				pullDisable,	1,	0),	// LED
	/*1 */	makepin(GPIOA,	11,		outPushPull,				pullDisable,	0,	0),	// EN1
	/*2 */	makepin(GPIOA,	12,		outPushPull,				pullDisable,	0,	0),	// RS485DE
	/*3 */	makepin(GPIOA,	2,		digitalInput,				pullUp,			0,	0),	// BT1_
	/*4 */	makepin(GPIOA,	3,		digitalInput,				pullUp,			0,	0),	// BT1
};
static const uint32_t pinNum = sizeof(pinsMode) / sizeof(pinMode_type);

static gpioCallback_type gpioCallback;

void irqSetCallback(gpioCallback_type callback){
	gpioCallback = callback;
}

/*!****************************************************************************
* InitAllGpio
*/
void gpio_init(void){
	pinMode_type *pgpios;
	pinMode_type *pgpiosEnd;

	pgpios = (pinMode_type*)pinsMode;
	pgpiosEnd = pgpios + pinNum;

	while(pgpios < pgpiosEnd){
		gppin_init(pgpios->p, pgpios->npin, pgpios->mode, pgpios->pull, pgpios->iniState, pgpios->nAF);
		pgpios++;
	}
}

/*!****************************************************************************
*
*/
void gppin_init(GPIO_TypeDef *port, uint8_t npin, gpioMode_type mode, gpioPull_type pull, uint8_t iniState, uint8_t nAF){
	//Clock enable
		if(port == GPIOA)   RCC->IOPENR    |= RCC_IOPENR_GPIOAEN;
	else if(port == GPIOB)   RCC->IOPENR    |= RCC_IOPENR_GPIOBEN;
	else if(port == GPIOC)   RCC->IOPENR    |= RCC_IOPENR_GPIOCEN;
	else if(port == GPIOD)   RCC->IOPENR    |= RCC_IOPENR_GPIODEN;
	/*else if(port == GPIOE)   RCC->IOPENR    |= RCC_IOPENR_GPIOEEN;*/
	else if(port == GPIOF)   RCC->IOPENR    |= RCC_IOPENR_GPIOFEN;
	/*else if(port == GPIOG)   RCC->IOPENR    |= RCC_IOPENR_GPIOGEN;
    else if(port == GPIOH)   RCC->IOPENR    |= RCC_IOPENR_GPIOHEN;*/

	if(iniState != 0){
		port->BSRR = (1 << npin);
	}else{
		port->BRR = (1 << npin);
	}

	/*
	 * Clear bit field
	 */
	port->MODER &= ~(0x03 << (2 * npin));
	port->OTYPER &= ~(1 << npin);
	port->PUPDR &= ~(GPIO_RESERVED << (2 * npin));
	port->AFR[npin / 8] &= ~(GPIO_AFRL_AFSEL0_Msk << (4 * (npin % 8)));

	//Set pull
	if(pull == pullUp){
		port->PUPDR |= GPIO_PULL_UP << (2 * npin);
	}else if(pull == pullDown){
		port->PUPDR |= GPIO_PULL_DOWN << (2 * npin);
	}

	//Set mode
	switch(mode){
		case analogMode:
			port->MODER |= GPIO_ANALOG_MODE << (2 * npin);
			break;

		case digitalInput:
			port->MODER &= ~(0x03 << (2 * npin));
			break;

		case outPushPull:
			port->MODER |= GPIO_GP_OUT << (2 * npin);
			port->OTYPER |= GPIO_PUSH_PULL << npin;
			break;

		case outOpenDrain:
			port->MODER |= GPIO_GP_OUT << (2 * npin);
			port->OTYPER |= GPIO_OPEN_DRAIN << npin;
			break;

		case alternateFunctionPushPull:
			port->MODER |= GPIO_AF_MODE << (2 * npin);
			port->OTYPER |= GPIO_PUSH_PULL << npin;
			port->OSPEEDR |= 3 << (2 * npin);   //High speed
			break;

		case alternateFunctionOpenDrain:
			port->MODER |= GPIO_AF_MODE << (2 * npin);
			port->OTYPER |= GPIO_OPEN_DRAIN << npin;
			break;
	}

	//Set number alternate function
	port->AFR[npin / 8] |= nAF << (4*(npin % 8));
}

/******************************** END OF FILE ********************************/
