/* ----------------------- Standard includes --------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

/* ----------------------- Defines  -----------------------------------------*/
#if MB_ASCII_ENABLED == 1
#define BUF_SIZE    513         /* must hold a complete ASCII frame. */
#else
#define BUF_SIZE    256         /* must hold a complete RTU frame. */
#endif

/* ----------------------- Static variables ---------------------------------*/
static BOOL bRxEnabled;
static BOOL bTxEnabled;

static unsigned char rxBuffer[BUF_SIZE];
static unsigned char *txPointer;

//static int rxCount;
static int txCount;

static int rxSize = 0;

static int uiRxBufferPos;
static int uiTxBufferPos;

static int rxDone = 0;
static int txNotEmpty = 0;


void mbSlaveSetReceive(void *data, size_t len){
	memcpy(rxBuffer, data , len);
	rxSize = len;
	rxDone = 1;
	xMBPortEventPost(EV_FRAME_RECEIVED);
}

size_t mbSlaveGetTransmit(void *data){
	int len = 0;
	if(txNotEmpty){
		memcpy(data, txPointer, txCount);
		len = txCount;
		txNotEmpty = 0;
		txCount = 0;
	}

	if(len > 0){
	}
	return len;
}

int getReceiveData(void *data){
	int len = 0;
	if(rxDone){
		memcpy(data, rxBuffer, rxSize);
		len = rxSize;
		rxDone = 0;
		rxSize = 0;
	}
	return len;
}

int setTransmitData(void *data, int len){
	txPointer = data;
	txNotEmpty = 1;
	txCount = len;
	return 0;
}

/* ----------------------- Begin implementation -----------------------------*/
void vMBPortSerialEnable(BOOL bEnableRx, BOOL bEnableTx){
	/* it is not allowed that both receiver and transmitter are enabled. */
	assert( !bEnableRx || !bEnableTx );

	if(bEnableRx){
		uiRxBufferPos = 0;
		bRxEnabled = TRUE;
	}else{
		bRxEnabled = FALSE;
	}
	if(bEnableTx){
		bTxEnabled = TRUE;
		uiTxBufferPos = 0;
	}else{
		bTxEnabled = FALSE;
	}
}

BOOL xMBPortSerialInit(UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity){
	(void)ucPort;
	(void)ulBaudRate;
	(void)ucDataBits;
	(void)eParity;
	vMBPortSerialEnable( FALSE, FALSE );
	return TRUE;
}

void vMBPortClose(void){

}

BOOL xMBPortSerialPoll(){
	if(bRxEnabled && rxDone){
		if(rxSize > uiRxBufferPos){
			pxMBFrameCBByteReceived();
		}
	}

	BOOL bStatus = TRUE;
	return bStatus;
}

BOOL xMBPortSerialPutByte(CHAR ucByte){
	(void)ucByte;
	return FALSE;
}

BOOL xMBPortSerialGetByte(CHAR *pucByte){
	assert( uiRxBufferPos < BUF_SIZE );
	*pucByte = rxBuffer[uiRxBufferPos];
	uiRxBufferPos++;
	if(uiRxBufferPos >= rxSize){
		xMBPortEventPost(EV_FRAME_RECEIVED);
	}
	return TRUE;
}
