/*!****************************************************************************
 * @file		adc
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date		12-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include "stm32g0xx.h"

/*!****************************************************************************
* User define
*/
#define ADC_TIM_FREQUENCY			(64000000)  //[Hz]

/*!****************************************************************************
* User typedef
*/
enum{
	CH_UINADC,
	CH_ILED1,
	CH_ILED2,
	CH_TEMPERATURE,
	CH_VREF,
	CH_NUMBER
};

typedef struct adcStct{
	ADC_TypeDef *adc;
	ADC_Common_TypeDef *com;
	TIM_TypeDef *tim;

	uint16_t		sampleRate;
	uint16_t 		adcreg[CH_NUMBER];
	void (*tcHoock)(struct adcStct *adc);
}adcStct_type;

typedef void (*adcCallback_type)(adcStct_type *adc);

/*!****************************************************************************
* Prototypes for the functions
*/
void adc_init(void);
void adc_startSampling(void);
void adc_stopSampling(void);
void adc_setSampleRate(uint16_t us);
void adc_setCallback(adcCallback_type tcHoock);

#ifdef __cplusplus
}
#endif

#endif //ADC_H
/******************************** END OF FILE ********************************/
