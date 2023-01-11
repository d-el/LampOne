/*!****************************************************************************
 * @file    	adcTSK.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	17-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */
#ifndef ADC_TSK_H
#define ADC_TSK_H

/*!****************************************************************************
 * Include
 */

/*!****************************************************************************
 * User typedef
 */
typedef struct {
	struct {
		uint16_t vin;
		uint16_t iled1;
		uint16_t temperature;
		uint16_t vref;
	}filtered;
	uint16_t		targetcurrent;
	bool 			overload;
} adcTaskStct_type;

/*!****************************************************************************
 * External variables
 */
extern adcTaskStct_type adcTaskStct;

/*!****************************************************************************
 * Prototypes for the functions
 */
void adcTSK(void *pPrm);

#endif //ADC_TSK_H
/******************************** END OF FILE ********************************/
