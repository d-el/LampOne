#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "plog.h"

#define dbgBufferSize 	1024
static char dbg_Buff[dbgBufferSize];

int _write( int file, const void *ptr, unsigned int len );

void hexdump(const uint8_t *buffer, uint16_t length){
	size_t len = 0;
	for(size_t i = 0; i < dbgBufferSize / 3 && i < length; i++){
		len += snprintf(dbg_Buff + len, dbgBufferSize - len, "%02X ", buffer[i]);
	}
	len += snprintf(dbg_Buff + len, dbgBufferSize - len, "\r\n");
	//plog_write(0, "", dbg_Buff);
	_write(0, dbg_Buff, len);
}

