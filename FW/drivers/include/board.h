/*!****************************************************************************
* @file    		board.h
* @author  		d_el
* @version 		V1.0
* @date    		01.05.2016, Storozhenko Roman
* @copyright 	GNU Public License
*
* TIM12_CH1 (PB14)             	-> LED
* TIM3_CH1 						-> ADC TRIGGER
* TIM4_CH2 (PA12)			    -> FAN_PWM
*
* SDADC1_IN4P 					-> UDC_MEAS
* SDADC1_IN5P 					-> I_MEAS
* SDADC1_IN6P 					-> U_MEAS
*
* UART3 TX - (PD8)              -> 1Wire
* UART1 TX - (PA9), RX - (PA10)	-> UART CONNECT
*
* GPIO (PB5) 					-> ON_OFF
* GPIO (PA0)                    -> CC_CV
*/
#ifndef board_H
#define board_H

/*!****************************************************************************
* Include
*/
#include "gpio.h"

/*!****************************************************************************
* User define
*/
#define SYSTEM_FREQ         64000000    //[Hz]
#define APB1_FREQ           64000000    //[Hz]
#define APB2_FREQ           64000000    //[Hz]

/*!****************************************************************************
* Macro functions
*/
#define LED_ON()            gppin_set(GP_LED)
#define LED_OFF()           gppin_reset(GP_LED)

/*!****************************************************************************
* Prototypes for the functions
*/

#endif //board_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/
