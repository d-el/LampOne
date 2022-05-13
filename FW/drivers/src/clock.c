/*!****************************************************************************
 * @file    	clock.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	09.01.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include "stm32g0xx.h"
#include "clock.h"

/*!****************************************************************************
* MEMORY
*/
clock_type	clock;

/*!****************************************************************************
*
*/
void clock_init(void){
	useGen_type useGen;

	//RCC->PLLCFGR &=~(RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLSRC);		//CLear

	// HSI
	RCC->CR = RCC_CR_HSION;						//HSI oscillator ON
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	useGen = clock_useHsi;

	// PLL config
	RCC->PLLCFGR =	1 << RCC_PLLCFGR_PLLR_Pos | // R: /2
					RCC_PLLCFGR_PLLREN |		// PLLRCLK clock output enable
					1 << RCC_PLLCFGR_PLLP_Pos | // P: /2
					8 << RCC_PLLCFGR_PLLN_Pos | // N: *8
					0 << RCC_PLLCFGR_PLLM_Pos | // M: /1
					RCC_PLLCFGR_PLLSRC_1; // HSI16

	// PLL enable
	RCC->CR |= RCC_CR_PLLON;					// Enable PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0);		// Wait PLL

	// Set flash wait states
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;

	// Select PLL as system clk
	RCC->CFGR   &= ~RCC_CFGR_SW;				// Clear SW0, SW1
	RCC->CFGR |= RCC_CFGR_SW_1;					// Select system clock - PLLRCLK
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1);	// Wait on switch PLL

	clock.useGen = useGen;
}

/******************************** END OF FILE ********************************/
