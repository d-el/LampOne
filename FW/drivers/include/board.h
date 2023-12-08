/*!****************************************************************************
* @file    		board.h
* @author  		d_el
* @version 		V1.0
* @date    		01.05.2016, Storozhenko Roman
* @copyright 	GNU Public License
*
* TIM1 						-> ADC TRIGGER
* TIM4_CH2 (PA12)			-> FAN_PWM
*
* ADC1_CH4 					-> UIN_MEAS
* ADC1_CH5 					-> ILED1
* ADC1_CH7 					-> ILED2
*
* UART1 TX/RX - (PB6)		-> UART CONNECT
*
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
