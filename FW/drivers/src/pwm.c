/*!****************************************************************************
 * @file		pwm.c
 * @author		Storozhenko Roman - D_EL
 * @version		V1.0
 * @date		12.05.2022
 * @copyright	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include "stm32g0xx.h"
#include "board.h"
#include "gpio.h"
#include "pwm.h"

static pwmUpdateCallback_t pwmUpdateCallback;

/*!****************************************************************************
* @brief
*/
void pwm_init(void){
	gppin_init(GPIOA, 6, alternateFunctionPushPull, pullDisable, 0, 1);
	gppin_init(GPIOB, 0, alternateFunctionPushPull, pullDisable, 0, 1);

	RCC->APBENR1 |= RCC_APBENR1_TIM3EN;						// Clock enable
	RCC->APBRSTR1 |= RCC_APBRSTR1_TIM3RST;					// Timer reset
	RCC->APBRSTR1 &= ~RCC_APBRSTR1_TIM3RST;

	TIM3->PSC = 1 - 1;										// Prescaler
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;		// PWM mode 1 (NORMAL PWM)
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;							// Output compare preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;		// PWM mode 1 (NORMAL PWM)
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;							// Output compare preload enable
	TIM3->ARR = 430/*APB1_FREQ / PWM_FREQ*/;				// Auto reload register  // 128 kHz
	TIM3->CR1 |= TIM_CR1_ARPE;								// TIMx_ARR register is buffered
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;			// CH Output Enable
	TIM3->DIER |= TIM_DIER_UIE;								// Update interrupt enable

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 14/*Priority*/);

	TIM3->CR1 |= TIM_CR1_CEN;								// Counter enable
}

/*!****************************************************************************
* @brief
*/
void pwm1set(uint16_t ccr){
	if(ccr > TIM3->ARR){
		ccr = TIM3->ARR;
	}
	TIM3->CCR1 = ccr;
}

void pwm2set(uint16_t ccr){
	if(ccr > TIM3->ARR){
		ccr = TIM3->ARR;
	}
	TIM3->CCR3 = ccr;
}

void TIM3_IRQHandler(void){
	if(pwmUpdateCallback){
		pwmUpdateCallback();
	}
	TIM3->SR = ~TIM_SR_UIF; // Clear flag
}

void pwm_UpdateCallbackSet(pwmUpdateCallback_t c){
	pwmUpdateCallback = c;
}

/******************************** END OF FILE ********************************/
