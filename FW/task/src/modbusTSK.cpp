/*!****************************************************************************
 * @file		uartTSK.c
 * @author		d_el
 * @version		V1.2
 * @date		13.12.2020
 * @brief		connect interface with regulator
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <uart.h>
#include <crc.h>
#include <mb.h>
#include <prmSystem.h>
#include <board.h>

#define PIECE_BUF_RX		256	//[bytes]
#define connectUart			uart1

/*!****************************************************************************
* MEMORY
*/
static SemaphoreHandle_t connUartRxSem;
static bool needSave;

/******************************************************************************
 * Local function declaration
 */
static void uartTxHook(uart_type *puart);
static void uartRxHook(uart_type *puart);

/*!****************************************************************************
* @brief	Connect program task
*/
void modbusTSK(void *pPrm){
	(void)pPrm;

	// Create Semaphore for UART
	vSemaphoreCreateBinary(connUartRxSem);
	xSemaphoreTake(connUartRxSem, portMAX_DELAY);
	assert(connUartRxSem != NULL);

//	while(1){
//		vTaskDelay(5);
//	}

	uart_init(uart1, 115200);
	uart_setCallback(connectUart, uartTxHook, uartRxHook);
	eMBInit(MB_RTU, 0x01, 0, 115200, MB_PAR_NONE);
	eMBEnable();

	while(1){
		uart_read(connectUart, connectUart->pRxBff, PIECE_BUF_RX);
		BaseType_t res = xSemaphoreTake(connUartRxSem, portMAX_DELAY);
		size_t numRx = PIECE_BUF_RX - uartGetRemainRx(connectUart);

		if((numRx != 0)&&(res == pdTRUE)){
			//LED_ON();
			mbSlaveSetReceive(connectUart->pRxBff, numRx);
			if(eMBPoll() == MB_ENOERR){
				uint8_t txsize = mbSlaveGetTransmit(connectUart->pTxBff);
				if(txsize > 0){
					gppin_set(GP_RS485DE);
					uart_write(connectUart, connectUart->pTxBff, txsize);
					xSemaphoreTake(connUartRxSem, pdMS_TO_TICKS(100));
				}
	}
	//LED_OFF();
}
	}
}

/*!****************************************************************************
 * @brief
 */
#define P_LOGW(...)
#define P_LOGD(...)

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode){
	usAddress--;
	switch(eMode){
		/* Pass current register values to the protocol stack. */
		case MB_REG_READ:
			while(usNRegs > 0){
				auto *ph = Prm::getbyaddress(usAddress);
				if(ph == nullptr){
					P_LOGW(logTag, "read: illegal register address [%04X], regs %u", usAddress, usNRegs);
					return MB_ENOREG;
				}

				auto prmsize = ph->getsize();
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "read: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				//char string[32];
				//ph->tostring(string, sizeof(string));
				//P_LOGD(logTag, "read: [%04X] %u %s %s %s", usAddress, usNRegs, ph->label, string, ph->units);

				(*ph)(true, nullptr);
				uint8_t buffer[4] = {};
				ph->serialize(buffer);
				switch(prmsize){
					case 1:
					case 2:
						*pucRegBuffer++ = buffer[1];
						*pucRegBuffer++ = buffer[0];
						usAddress++;
						usNRegs--;
						break;
					case 4:
						*pucRegBuffer++ = buffer[1];
						*pucRegBuffer++ = buffer[0];
						*pucRegBuffer++ = buffer[3];
						*pucRegBuffer++ = buffer[2];
						usAddress += 2;
						usNRegs -= 2;
						break;
				}
			}
			break;

			/* Update current register values with new values from the
			 * protocol stack. */
		case MB_REG_WRITE:
			while(usNRegs > 0){
				auto *ph = Prm::getbyaddress(usAddress);
				if(ph == nullptr){
					P_LOGW(logTag, "write: illegal register address [%04X]", usAddress);
					return MB_ENOREG;
				}

				auto prmsize = ph->getsize();
				if(prmsize == 4 && usNRegs < 2){
					P_LOGW(logTag, "write: [%04X] usNRegs < 2 in 4 Byte parameter", usAddress);
					return MB_EINVAL;
				}

				uint8_t buffer[4];
				switch(prmsize){
					case 1:
					case 2:
						buffer[1] = *pucRegBuffer++;
						buffer[0] = *pucRegBuffer++;
						usAddress++;
						usNRegs--;
						break;

					case 4:
						buffer[1] = *pucRegBuffer++;
						buffer[0] = *pucRegBuffer++;
						buffer[3] = *pucRegBuffer++;
						buffer[2] = *pucRegBuffer++;
						usAddress += 2;
						usNRegs -= 2;
						break;
				}

				if(!ph->deserialize(buffer)){
					P_LOGW(logTag, "write [%04X]: out of range", usAddress);
					return MB_EINVAL;
				}
				(*ph)(false, nullptr);
				//char string[32];
				//ph->tostring(string, sizeof(string));
				//P_LOGD(logTag, "write: [%04X] %u %s %s %s", usAddress, usNRegs, ph->label, string, ph->units);

				if(ph->getsave() == Prm::savesys){
					needSave = true;
				}
			}
			break;
	}
	return MB_ENOERR;
}

/*!****************************************************************************
 * @brief
 */
bool modbus_needSave(bool clear){
	bool cuurentState = needSave;
	if(clear){
		needSave = false;
	}
	return cuurentState;
}

/*!****************************************************************************
 * @brief	uart RX TX callback
 */
static void uartRxHook(uart_type *puart){
	(void)puart;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(connUartRxSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void uartTxHook(uart_type *puart){
	(void)puart;
	gppin_reset(GP_RS485DE);
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(connUartRxSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/******************************** END OF FILE ********************************/
