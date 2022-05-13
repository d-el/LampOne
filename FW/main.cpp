/*!****************************************************************************
* @mainpage                                                                   *
* @file         main.cpp                                                      *
* @author       Storozhenko Roman - D_EL                                      *
* @version      V1.0                                                          *
* @date         10-01-2022                                                    *
* @brief        Main program body                                             *
* @copyright    The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman   *
******************************************************************************/

/*!****************************************************************************
* Include
*/
#include <drivers.h>
#include <systemTSK.h>

/*!****************************************************************************
* @brief    main function
*/
int main(void){
	hardInit();
	OSinit();
	while(1);
	return 0;
}

/******************************** END OF FILE ********************************/
