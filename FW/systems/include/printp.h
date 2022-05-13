/*!****************************************************************************
 * @file    	printp.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	06.12.2017
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef printp_H
#define printp_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
 * Include
 */

/*!****************************************************************************
 * Typedef
 */
typedef enum{
	stdOut_semihost,
	stdOut_rtt,
	stdOut_uart,
}stdOutInterface_type;

/*!****************************************************************************
 * Exported variables
 */

/*!****************************************************************************
 * Macro functions
 */
#ifndef NDEBUG           /* required by ANSI standard */

	#define printp(...) l_print(__VA_ARGS__)

	#define report(test, ...) 				\
		do { 								\
			if(test){ 						\
				l_print(__VA_ARGS__); 		\
			} 								\
		}while(0);

	#define stopif(assertion, error_action, ...) {              \
		if (assertion){                                         \
			l_print(__VA_ARGS__); 								\
			{													\
				error_action;									\
			}                  									\
		} }

#else //NDEBUG

	#define printp(...) ((void)0)

	#define report(test, ...) ((void)0)

	#define stopif(assertion, error_action, ...) ((void)0)

#endif /* !NDEBUG */

/*!****************************************************************************
 * Function declaration
 */
void print_init(stdOutInterface_type stdoi);
void l_print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif //printp_H
/******************************** END OF FILE ********************************/
