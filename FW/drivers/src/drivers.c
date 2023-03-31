/*!****************************************************************************
 * @file    	drivers.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	05.07.2013
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include "board.h"
#include "clock.h"
#include "uart.h"
#include "adc.h"
#include "pwm.h"
#include "flash.h"
#include "drivers.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
*
*/
void hardInit(void){
	clock_init();
	gpio_init();
}

/******************************** END OF FILE ********************************/
