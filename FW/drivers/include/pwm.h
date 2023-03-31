/*!****************************************************************************
 * @file		pwm.h
 * @author		Storozhenko Roman - D_EL
 * @version		V1.0
 * @date		12.05.2022
 * @copyright	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */
#ifndef pwm_H
#define pwm_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdint.h>

/*!****************************************************************************
* User define
*/
typedef void (*pwmUpdateCallback_t)(void);

/*!****************************************************************************
* Prototypes for the functions
*/
void pwm_init(void);
void pwm1set(uint16_t ccr);
void pwm2set(uint16_t ccr);
void pwm_UpdateCallbackSet(pwmUpdateCallback_t c);

#ifdef __cplusplus
}
#endif

#endif //pwm_H
/******************************** END OF FILE ********************************/
