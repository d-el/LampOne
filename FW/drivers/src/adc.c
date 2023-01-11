/*!****************************************************************************
 * @file		adc.c
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date		12-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <stddef.h>
#include "gpio.h"
#include "board.h"
#include "adc.h"

/*!****************************************************************************
* MEMORY
*/
adcStct_type adcStct = {
	.sampleRate = 10000,	// Default sample Rate
};

/*!****************************************************************************
* TIM1 -> ADC1 -> DMA2_Channel3 -> DMA1_Channel1_IRQHandler
*/
void adc_init(void){
	adcStct.adc = ADC1;
	adcStct.com = ADC1_COMMON;
	adcStct.tim = TIM1;

	/**********************************
	 * IO
	 */
	//Analog Input
	gppin_init(GPIOA, 4, analogMode, pullDisable, 0, 0);
	//Analog Input
	gppin_init(GPIOA, 5, analogMode, pullDisable, 0,  0);
	//Analog Input
	gppin_init(GPIOA, 7, analogMode, pullDisable, 0, 0);

	for(int i = 0; i < 360000; i++) __NOP();

	/**********************************
	 * Clock
	 */
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;					// ADC clock Enable
	RCC->APBRSTR2 |= RCC_APBRSTR2_ADCRST;				// ADC reset
	RCC->APBRSTR2 &= ~RCC_APBRSTR2_ADCRST;

	/**********************************
	 * ADC
	 */
	ADC1->CR |= ADC_CR_ADVREGEN;				// ADC Voltage Regulator Enable
	for(int i = 0; i < 360000; i++) __NOP();

	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;			// Hardware trigger detection on the rising edge
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTSEL;			// External triggers TIM1_TRGO2

	ADC1->CFGR1 |= ADC_CFGR1_DMACFG;			// DMA circular mode selected

	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1;			// ADC clock PCLK/4

	// Select channel
	ADC1->CHSELR |= ADC_CHSELR_CHSEL4 |
					ADC_CHSELR_CHSEL5 |
					ADC_CHSELR_CHSEL7 |
					ADC_CHSELR_CHSEL12| // Temperature
					ADC_CHSELR_CHSEL13; // Vref
	while((ADC1->ISR & ADC_ISR_CCRDY) == 0);
	ADC1->ISR = ADC_ISR_CCRDY;

	// Set sample time
	ADC1->SMPR =	5 << ADC_SMPR_SMPSEL4_Pos |
					1 << ADC_SMPR_SMPSEL5_Pos |
					1 << ADC_SMPR_SMPSEL7_Pos |
					1 << ADC_SMPR_SMPSEL12_Pos |
					1 << ADC_SMPR_SMPSEL13_Pos |
					3 << ADC_SMPR_SMP1_Pos |	// 12.5 ADC clock cycles
					5 << ADC_SMPR_SMP2_Pos;		// 39.5 ADC clock cycles

	ADC1_COMMON->CCR |= ADC_CCR_TSEN;			// Temperature sensor enable
	ADC1_COMMON->CCR |= ADC_CCR_VREFEN;			// V REFINT enable
	ADC1_COMMON->CCR |= ADC_CCR_PRESC_1;		// Input ADC clock divided by 4

	ADC1->CR |= /*ADC_CR_ADEN |*/ ADC_CR_ADCAL;	// ADC calibration
	while((ADC1->ISR & ADC_ISR_EOCAL) == 0);	// Wait for Calibration has completed and the offsets have been updated
	ADC1->ISR = ADC_ISR_EOCAL;

	for(int i = 0; i < 36000; i++) __NOP();
	ADC1->CR |= ADC_CR_ADEN;					// Enable
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
	ADC1->ISR = ADC_ISR_ADRDY;

	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;

	/**********************************
	 * DMA Init
	 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;							// DMA Clock Enable
	DMAMUX1_Channel0->CCR = 5 << DMAMUX_CxCR_DMAREQ_ID_Pos;		// ADC
	DMA1_Channel1->CCR = 0;
	DMA1_Channel1->CCR |= DMA_CCR_PL_0;							// Channel priority level - Medium
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;						// Memory size 16 bit
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;						// Peripheral size 16 bit
	DMA1_Channel1->CCR |= DMA_CCR_MINC;							// Memory increment mode enabled
	DMA1_Channel1->CCR &= ~DMA_CCR_PINC;						// Peripheral increment mode disabled
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;							// Circular mode enabled
	DMA1_Channel1->CCR &= ~DMA_CCR_DIR;							// Read from peripheral
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;							// Transfer complete interrupt enable
	DMA1_Channel1->CNDTR = CH_NUMBER;							// Number of data
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);				// Peripheral address
	DMA1_Channel1->CMAR = (uint32_t)&adcStct.adcreg[0];			// Memory address
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 15/*Priority*/);
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	/**********************************
	 * TIM Init
	 */
	RCC->APBENR2	|= RCC_APBENR2_TIM1EN;				// Enable clock
	RCC->APBRSTR2	|= RCC_APBRSTR2_TIM1RST;			// Timer reset
	RCC->APBRSTR2	&= ~RCC_APBRSTR2_TIM1RST;
	TIM1->PSC		= ADC_TIM_FREQUENCY / 1000000 - 1;	// Set prescaler
	TIM1->CR1		|= TIM_CR1_ARPE;					// TIMx_ARR register is buffered
	TIM1->CR2		|= TIM_CR2_MMS2_1; 					// Update event is selected as trigger output (TRGO2)
	TIM1->ARR		= adcStct.sampleRate;				// SampleRate, [us]

	ADC1->CR |= ADC_CR_ADSTART;
}

/*!****************************************************************************
 *
 */
void adc_startSampling(void){
	TIM1->CR1 |= TIM_CR1_CEN;
}

/*!****************************************************************************
 *
 */
void adc_stopSampling(void){
	TIM1->CR1 &= ~TIM_CR1_CEN;
}

/*!****************************************************************************
 *
 */
void adc_setSampleRate(uint16_t us){
	adcStct.sampleRate = us;
	TIM1->ARR = us;
}

/*!****************************************************************************
 *
 */
void adc_setCallback(adcCallback_type tcHoock){
	adcStct.tcHoock = tcHoock;
}

/*!****************************************************************************
*---> DMA for SAADC Interrupt Handler
*/
void DMA1_Channel1_IRQHandler(void){
	if(adcStct.tcHoock != NULL){
		adcStct.tcHoock(&adcStct);
	}
	DMA1->IFCR = DMA_IFCR_CTCIF1;
}

/******************************** END OF FILE ********************************/
