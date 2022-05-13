/*!****************************************************************************
 * @file		prmSystemCallback.c
 * @author		d_el
 * @version		V1.0
 * @date		17.03.2021
 * @brief
 */

/*!****************************************************************************
 * Include
 */
#include "prmSystemCallback.h"
#include <version.h>

void getFwVer(Prm::Val<uint16_t>& prm, bool read, void *arg){
	(void)read;
	(void)arg;

	switch(reinterpret_cast<uint32_t>(prm.getarg())){
		case 0:
			prm.val = getVersionMajor();
			break;
		case 1:
			prm.val = getVersionMinor();
			break;
		case 2:
			prm.val = getVersionPatch();
			break;
		case 3:
			prm.val = getVersionTweak();
			break;
	}
}

/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/
