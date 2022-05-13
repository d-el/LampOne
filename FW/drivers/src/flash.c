/*!****************************************************************************
 * @file    	flash.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	11.01.2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include "stm32g0xx.h"
#include "flash.h"

#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xCDEF89AB

/*!****************************************************************************
* Memory
*/

/*!****************************************************************************
* @brief	Unlock flash
*/
void flash_unlock(void){
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}

/*!****************************************************************************
* @brief	Lock flash
*/
void flash_lock(void){
	FLASH->CR |= FLASH_CR_LOCK;
}

/*!****************************************************************************
 * @brief	Returned not 0 if flash busy
 */
uint32_t flash_busy(void){
	return (FLASH->SR & FLASH_SR_BSY1);
}

/*!****************************************************************************
* @brief	Erase all pages
*/
void flash_eraseAllPages(void){
	FLASH->CR |= FLASH_CR_MER1;
	FLASH->CR |= FLASH_CR_STRT;
	while(flash_busy());
	FLASH->CR &= FLASH_CR_MER1;
}

/*!****************************************************************************
* @brief		Erase one page
* @param[in]	addr - address allocable in page
*/
void flash_erasePage(void *addr){
	FLASH->CR|= FLASH_CR_PER;
	FLASH->CR &= ~FLASH_CR_PNB;
	FLASH->CR |= (((size_t)addr - FLASH_BASE) / 2048) << FLASH_CR_PNB_Pos;
	FLASH->CR|= FLASH_CR_STRT;
	while(flash_busy());
	FLASH->CR&= ~FLASH_CR_PER;
}

/*!****************************************************************************
* @brief	Write data
* @param	dst[in] - destination align by 8 byte
* @param	src[in] - source
* @param	num[in] - number double word (8 byte)
*/
flashState_type flash_write(void *dst, void *src, size_t num){
	if((size_t)dst & 0x7 || num & 0x7){
		return flash_align;
	}

	FLASH->CR |= FLASH_CR_PG;
	while(flash_busy());

	uint32_t* rd = src;
	uint32_t* wr = dst;
	uint32_t* end = wr + num / 4;

	while(wr < end){
		*wr++ = *rd++;
		__DSB();
		while(flash_busy());
	}

	FLASH->CR &= ~FLASH_CR_PG;
	return flash_ok;
}

/******************************** END OF FILE ********************************/
