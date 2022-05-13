/*!****************************************************************************
 * @file    	printp.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	06.12.2017
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
/*!****************************************************************************
 * Include
 */
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "debugCore.h"
#include "semihosting.h"
#include <uart.h>
#include "printp.h"

/*!****************************************************************************
 * MEMORY
 */
#define DEBUG_BUF_LEN	64
static int fdstdout;
static char g_buf[DEBUG_BUF_LEN];

/*!****************************************************************************
 * Local prototypes for the functions
 */
int _write(int fd, const void *buf, size_t count);

/*!****************************************************************************
 * @brief
 */
void print_init(stdOutInterface_type stdoi){
	switch(stdoi){
		case stdOut_semihost:
			break;
		case stdOut_rtt:
			break;
		case stdOut_uart:
			break;
	}

	fdstdout = stdoi;
}

/*!****************************************************************************
 * @brief    print debug message, printf format
 */
void l_print(const char *fmt, ...){
	va_list va;
	va_start(va, fmt);
	vsiprintf(g_buf, fmt, va);
	va_end(va);
	_write(fdstdout, g_buf, strlen(g_buf));
}

/*!****************************************************************************
 * @brief writes count bytes from buffer into the standard output
 * @param fd: file descriptor of file into which data is written
 * @param buf: data to be written
 * @param count: number of bytes
 * @return returns the number of bytes actually written
 */
int _write(int fd, const void *buf, size_t count){
	switch(fd){
		case stdOut_semihost:
			if(count > DEBUG_BUF_LEN + 1){
				return -1;
			}
			memcpy(g_buf, buf, count);
			g_buf[count] = 0;
			sh_sendString(g_buf);
			break;
		case stdOut_rtt:
			break;
		case stdOut_uart:
			break;
		default:
			return -1;
	}

	return count;
}

/******************************** END OF FILE ********************************/
