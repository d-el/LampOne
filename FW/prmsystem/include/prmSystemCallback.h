/*!****************************************************************************
 * @file		prmSystemCallback.h
 * @author		d_el
 * @version		V1.0
 * @date		17.03.2021
 * @brief
 */

#ifndef prmSystemCallback_H
#define prmSystemCallback_H

/*!****************************************************************************
 * Include
 */
#include "prmSystem.h"

/*!****************************************************************************
 * Function declaration
 */
void getFwVer(Prm::Val<uint16_t>& prm, bool read, void *arg);
void setCurrent(Prm::Val<uint16_t>& prm, bool read, void *arg);
void saveParameter(Prm::Val<uint16_t>& prm, bool read, void *arg);
void calibrateCurrent(Prm::Val<uint16_t>& prm, bool read, void *arg);

#endif //prmSystemCallback_H

/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/
