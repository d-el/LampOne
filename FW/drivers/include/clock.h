/*!****************************************************************************
 * @file    	clock.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	09.01.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef clock_H
#define clock_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include "stdint.h"

/*!****************************************************************************
* User define
*/
#define RCC_CRYSTAL_OSCILLATOR_FREQ 	24000000
#define RCC_RC_OSCILLATOR_FREQ			8000000
#define HSE_PLL_MUL						2           //2-16, Mul PLL
#define HSE_PLL_PREDIV					2           //1-16, Div PLL
#define HSI_PLL_MUL						6           //2-16, Mul PLL
#define HSI_PLL_PREDIV					1           //1-16, Div PLL
#define RCC_WAIN_TIMEOUT				100000

/*!****************************************************************************
* User enum
*/
typedef enum{
	clock_useHsi,
	clock_useHse
}useGen_type;

/*!****************************************************************************
* User typedef
*/
typedef struct{
	uint32_t        currentSysFrec;
	useGen_type     useGen;
}clock_type;

/*!****************************************************************************
* External variables
*/
extern clock_type clockSource;

/*!****************************************************************************
* Prototypes for the functions
*/
void clock_init(void);

#ifdef __cplusplus
}
#endif

#endif //clock_H
/******************************** END OF FILE ********************************/
