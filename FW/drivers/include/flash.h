/*!****************************************************************************
 * @file		flash.h
 * @author		Storozhenko Roman - D_EL
 * @version		V1.0
 * @date		11.01.2022
 * @copyright	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */
#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdint.h>

/*!****************************************************************************
* User typedef
*/
typedef enum{
	flash_ok,
	flash_signatureError,
	flash_CRCError,
	flash_ErrorSizeMem,
	flash_align,
	flash_error
}flashState_type;


/*!****************************************************************************
* Prototypes for the functions
*/
void flash_unlock(void);
void flash_lock(void);
void flash_eraseAllPages(void);
void flash_erasePage(void *address);
flashState_type flash_write(void *dst, void *src, size_t num);

#ifdef __cplusplus
}
#endif

#endif //FLASH_H
/******************************** END OF FILE ********************************/
