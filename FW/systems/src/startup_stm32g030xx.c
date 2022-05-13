/*!****************************************************************************
 * @file		startup_stm32g030xx.c
 * @author		d_el - Storozhenko Roman
 * @version		V1.0
 * @date		10.01.2022
 * @copyright	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 * @brief
 */

/*!****************************************************************************
* Include
*/
#include "stdint.h"

extern void __libc_init_array(void);	///Initialization routines
extern void __libc_fini_array(void);	///Cleanup routines
extern void SystemInit(void);			///System initialization function
extern int main(void);					///Entry point for the application

extern uint32_t _estack;				///Highest address of the user mode stack
extern uint32_t _sdata;					///RAM data start
extern uint32_t _edata;					///RAM data end
extern uint32_t _sidata;				///ROM data start
extern uint32_t _sbss;					///RAM bss start
extern uint32_t _ebss;					///RAM bss end

typedef void(*intVector_type)(void);	///Interrupt service routine type

/*!****************************************************************************
 * @brief  Initialization .bss section
 */
void __initializeBss(uint32_t *bssStart, uint32_t *bssEnd){
	uint32_t* pData = bssStart;

	///Verify align by 4
	if(((uint32_t) bssStart % 4) || ((uint32_t) bssEnd % 4)){
		while(1)
			;
	}

	while(pData < bssEnd){
		*pData++ = 0;
	}
}

/*!****************************************************************************
 * @brief  Initialization .data section
 */
void __initializeData(uint32_t *dataStart, uint32_t *dataEnd, uint32_t *src){
	uint32_t *pData = dataStart;

	///Verify align by 4
	if(((uint32_t) dataStart % 4) || ((uint32_t) dataEnd % 4)||((uint32_t) src % 4)){
		while(1)
			;
	}

	while(pData < dataEnd){
		*pData++ = *src++;
	}
}

/*!****************************************************************************
 * @brief	Program entry point
 */
void Reset_Handler(void){
	__initializeData(&_sdata, &_edata, &_sidata);
	__initializeBss(&_sbss, &_ebss);
	__libc_init_array();
	SystemInit();
	main();
	__libc_fini_array();
	while(1);
}

/*!****************************************************************************
 * @brief	Default interrupt handler
 */
void Default_Handler(void){
	/*
	 * IPSR bit assignments
	 * [8:0]	ISR_NUMBER
	 * This is the number of the current exception:
	 * 0 = Thread mode
	 * 1 = Reserved
	 * 2 = NMI
	 * ...
	 */
	while(1);
}

///Cortex-M0 Processor Exceptions Numbers
void NMI_Handler						(void) __attribute__((weak, alias ("Default_Handler")));
void HardFault_Handler					(void) __attribute__((weak, alias ("Default_Handler")));
void SVC_Handler						(void) __attribute__((weak, alias ("Default_Handler")));
void PendSV_Handler						(void) __attribute__((weak, alias ("Default_Handler")));
void SysTick_Handler					(void) __attribute__((weak, alias ("Default_Handler")));
///STM32 specific Interrupt Numbers
void WWDG_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler"))); /* Window WatchDog              */
void RTC_TAMP_IRQHandler				(void) __attribute__((weak, alias ("Default_Handler"))); /* RTC through the EXTI line    */
void FLASH_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler"))); /* FLASH                        */
void RCC_IRQHandler						(void) __attribute__((weak, alias ("Default_Handler"))); /* RCC                          */
void EXTI0_1_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler"))); /* EXTI Line 0 and 1            */
void EXTI2_3_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler"))); /* EXTI Line 2 and 3            */
void EXTI4_15_IRQHandler				(void) __attribute__((weak, alias ("Default_Handler"))); /* EXTI Line 4 to 15            */
void DMA1_Channel1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler"))); /* DMA1 Channel 1               */
void DMA1_Channel2_3_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler"))); /* DMA1 Channel 2 and Channel 3 */
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler"))); /* DMA1 Channel 4 to Channel 5, DMAMUX1 overrun */
void ADC1_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* ADC1                         */
void TIM1_BRK_UP_TRG_COM_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));/* TIM1 Break, Update, Trigger and Commutation */
void TIM1_CC_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* TIM1 Capture Compare         */
void TIM3_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* TIM3                         */
void TIM14_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* TIM14                        */
void TIM16_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* TIM16                        */
void TIM17_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* TIM17                        */
void I2C1_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* I2C1                         */
void I2C2_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* I2C2                         */
void SPI1_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* SPI1                         */
void SPI2_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* SPI2                         */
void USART1_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* USART1                       */
void USART2_IRQHandler					(void) __attribute__((weak, alias ("Default_Handler")));/* USART2                       */

/*!****************************************************************************
* Interrupt vector table
*/
intVector_type intVector[] __attribute__ ((section (".isr_vector"))) = {
	///Stack
	(intVector_type)&_estack,
	///Cortex-M4 Processor Exceptions
	Reset_Handler,
	NMI_Handler,
	HardFault_Handler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler,
	WWDG_IRQHandler,
	0,
	RTC_TAMP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_1_IRQHandler,
	EXTI2_3_IRQHandler,
	EXTI4_15_IRQHandler,
	0,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_3_IRQHandler,
	DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler,
	ADC1_IRQHandler,
	TIM1_BRK_UP_TRG_COM_IRQHandler,
	TIM1_CC_IRQHandler,
	0,
	TIM3_IRQHandler,
	0,
	0,
	TIM14_IRQHandler,
	0,
	TIM16_IRQHandler,
	TIM17_IRQHandler,
	I2C1_IRQHandler,
	I2C2_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	0
};

/******************************** END OF FILE ********************************/
