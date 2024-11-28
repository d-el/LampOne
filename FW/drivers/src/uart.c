/*!****************************************************************************
 * @file		uart.c
 * @author		d_el
 * @version		V1.6
 * @date		29.11.2024
 * @brief		Driver for uart STM32G0 MCUs
 * @copyright	The MIT License (MIT). Copyright (c) 2024 Storozhenko Roman
 */

/*!****************************************************************************
 * Include
 */
#include <stdio.h>
#include "gpio.h"
#include "uart.h"
#include "board.h"

/*!****************************************************************************
 * uart1 memory
 */
#if (UART1_USE > 0)
uart_type uart1Sct;
uart_type *uart1 = &uart1Sct;
uint8_t uart1TxBff[UART1_TxBffSz];
uint8_t uart1RxBff[UART1_RxBffSz];
#endif //UART1_USE

/*!****************************************************************************
 * uart2 memory
 */
#if (UART2_USE > 0)
uart_type uart2Sct;
uart_type *uart2 = &uart2Sct;
uint8_t uart2TxBff[UART2_TxBffSz];
uint8_t uart2RxBff[UART2_RxBffSz];
#endif //UART2_USE

/*!****************************************************************************
 * uart3 memory
 */
#if (UART3_USE > 0)
uart_type uart3Sct;
uart_type *uart3 = &uart3Sct;
uint8_t uart3TxBff[UART3_TxBffSz];
uint8_t uart3RxBff[UART3_RxBffSz];
#endif //UART3_USE

/*!****************************************************************************
 * @brief
 */
void uart_init(uart_type *uartx, uint32_t baudRate){
	#if(UART1_USE > 0)
	if(uartx == uart1){
		/************************************************
		 * Memory setting
		 */
		uartx->pUart			= USART1;
		uartx->pTxBff			= uart1TxBff;
		uartx->pRxBff			= uart1RxBff;
		uartx->pUartTxDmaCh		= DMA1_Channel2;
		uartx->pUartRxDmaCh		= DMA1_Channel3;
		uartx->dmaIfcrTx		= &DMA1->IFCR;
		uartx->dmaIfcrRx		= &DMA1->IFCR;
		uartx->dmaIfcrMaskTx	= DMA_IFCR_CTCIF2;
		uartx->dmaIfcrMaskRx	= DMA_IFCR_CTCIF3;
		uartx->frequency		= APB2_FREQ;
		uartx->driverEnable		= UART1_DRIVER_ENABLE;

		/************************************************
		 * IO
		 */
		gppin_init(GPIOB, 6, alternateFunctionPushPull, pullDisable, 0, UART1_PINAFTX);			//USART1_TX
		#if(UART1_HALFDUPLEX == 0)
		gppin_init(GPIOB, 7, alternateFunctionPushPull, pullUp, 0, UART1_PINAFRX);				//USART1_RX
		#else
		uartx->halfDuplex = 1;
		#endif
		#if(UART1_DRIVER_ENABLE > 0)
		//gppin_init(GPIOA, 12, alternateFunctionPushPull, pullDisable, 0, UART1_PINAFDE);		//USART1_DE
		#endif

		/************************************************
		 * NVIC
		 */
		NVIC_EnableIRQ(USART1_IRQn);
		NVIC_SetPriority(USART1_IRQn, UART1_TXIRQPrior);

		/************************************************
		 * USART clock
		 */
		RCC->APBENR2 |= RCC_APBENR2_USART1EN;
		RCC->APBRSTR2 |= RCC_APBRSTR2_USART1RST;
		RCC->APBRSTR2 &= ~RCC_APBRSTR2_USART1RST;

		/************************************************
		 * DMA clock
		 */
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;

		// DMAMUX
		DMAMUX1_Channel1->CCR = 51 << DMAMUX_CxCR_DMAREQ_ID_Pos; // USART1_TX
		DMAMUX1_Channel2->CCR = 50 << DMAMUX_CxCR_DMAREQ_ID_Pos; // USART1_RX
	}
	#endif //UART1_USE

	#if(UART2_USE > 0)
	if(uartx == uart2){
		/************************************************
		 * Memory setting
		 */
		uartx->pUart			= USART2;
		uartx->pTxBff			= uart2TxBff;
		uartx->pRxBff			= uart2RxBff;
		uartx->pUartTxDmaCh		= DMA1_Channel7;
		uartx->pUartRxDmaCh		= DMA1_Channel6;
		uartx->dmaIfcrTx		= &DMA1->IFCR;
		uartx->dmaIfcrRx		= &DMA1->IFCR;
		uartx->dmaIfcrMaskTx	= DMA_IFCR_CTCIF4;
		uartx->dmaIfcrMaskRx	= DMA_IFCR_CTCIF5;
		uartx->frequency		= APB1_FREQ;
		uartx->driverEnable		= UART2_DRIVER_ENABLE;

		/************************************************
		 * IO
		 */
		gppin_init(GPIOA, 2, alternateFunctionPushPull, pullDisable, 0, UART2_PINAFTX);
		#if(UART2_HALFDUPLEX == 0)
		gppin_init(GPIOA, 3, alternateFunctionPushPull, pullUp, 0, UART2_PINAFRX);
		#else
		uartx->halfDuplex = 1;
		#endif

		/************************************************
		 * NVIC
		 */
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_SetPriority(USART2_IRQn, UART2_TXIRQPrior);

		/************************************************
		 * USART clock
		 */
		RCC->APBENR1 |= RCC_APBENR1_USART2EN;
		RCC->APBRSTR1 |= RCC_APBRSTR1_USART2RST;
		RCC->APBRSTR1 &= ~RCC_APBRSTR1_USART2RST;

		/************************************************
		 * DMA clock
		 */
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;

		// DMAMUX
		DMAMUX1_Channel3->CCR = 53 << DMAMUX_CxCR_DMAREQ_ID_Pos; // USART2_TX
		DMAMUX1_Channel4->CCR = 52 << DMAMUX_CxCR_DMAREQ_ID_Pos; // USART2_RX
	}
	#endif //UART2_USE

	#if(UART3_USE > 0)
	if(uartx == uart3){
		/************************************************
		 * Memory setting
		 */
		uartx->pUart			= USART3;
		uartx->pTxBff			= uart3TxBff;
		uartx->pRxBff			= uart3RxBff;
		uartx->pUartTxDmaCh		= DMA1_Channel6;
		uartx->pUartRxDmaCh		= DMA1_Channel7;
		uartx->dmaIfcrTx		= &DMA1->IFCR;
		uartx->dmaIfcrRx		= &DMA1->IFCR;
		uartx->dmaIfcrMaskTx	= DMA_IFCR_CTCIF6;
		uartx->dmaIfcrMaskRx	= DMA_IFCR_CTCIF7;
		uartx->frequency		= APB1_FREQ;
		uartx->driverEnable		= UART3_DRIVER_ENABLE;

		/************************************************
		 * IO
		 */
		gppin_init(GPIOB, 8, alternateFunctionOpenDrain, pullDisable, 0, UART3_PINAFTX);
		#if(UART3_HALFDUPLEX == 0)
		gppin_init(GPIOB, 9, alternateFunctionPushPull, pullUp, 0, UART3_PINAFRX);
		#else
		uartx->halfDuplex = 1;
		#endif

		/************************************************
		 * NVIC
		 */
		NVIC_EnableIRQ(USART3_4_IRQn);
		NVIC_SetPriority(USART3_4_IRQn, UART3_TXIRQPrior);

		/************************************************
		 * USART clock
		 */
		RCC->APBENR1 |= RCC_APBENR1_USART3EN;
		RCC->APBRSTR1 |= RCC_APBRSTR1_USART3RST;
		RCC->APBRSTR1 &= ~RCC_APBRSTR1_USART3RST;

		/************************************************
		 * DMA clock
		 */
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;

		// DMAMUX
		DMAMUX1_Channel5->CCR = 55 << DMAMUX_CxCR_DMAREQ_ID_Pos; // USART3_TX
		DMAMUX1_Channel6->CCR = 54 << DMAMUX_CxCR_DMAREQ_ID_Pos; // USART3_RX
	}
	#endif //UART3_USE

	/************************************************
	 * USART
	 */
	if(uartx->halfDuplex != 0){
		uartx->pUart->CR3 |= USART_CR3_HDSEL;								//Half duplex mode is selected
	}
	uartx->pUart->CR1 |= USART_CR1_UE;										//UART enable
	uartx->pUart->CR1 &= ~USART_CR1_M;										//8bit
	uartx->pUart->CR2 &= ~USART_CR2_STOP;									//1 stop bit

	uart_setBaud(uartx, baudRate);											//Baud rate
	uartx->pUart->CR3 |= USART_CR3_DMAT;									//DMA enable transmitter
	uartx->pUart->CR3 |= USART_CR3_DMAR;									//DMA enable receiver
	if(uartx->driverEnable != 0){
		uartx->pUart->CR3 |= USART_CR3_DEM;	// Driver enable mode
		uartx->pUart->CR1 |= 2 << USART_CR1_DEAT_Pos; // Driver Enable assertion time
		uartx->pUart->CR1 |= 2 << USART_CR1_DEDT_Pos; // Driver Enable deassertion time
	}
	uartx->pUart->CR1 |= USART_CR1_TE;										//Transmitter enable
	uartx->pUart->CR1 |= USART_CR1_RE;										//Receiver enable
	uartx->pUart->ICR = USART_ICR_TCCF | USART_ICR_IDLECF;					//Clear the flags
	uartx->pUart->ICR = USART_ICR_IDLECF;									//Clear flag
	uartx->pUart->CR1 |= USART_CR1_IDLEIE;
	uartx->pUart->CR1 |= USART_CR1_TCIE;									//Enable the interrupt transfer complete

	/************************************************
	 * DMA
	 */
	//DMA Channel USART TX
	uartx->pUartTxDmaCh->CCR = 0;
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->pUartTxDmaCh->CCR |= DMA_CCR_PL_0;									//Channel priority level - Medium
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_MSIZE;									//Memory size - 8 bit
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_PSIZE;									//Peripheral size - 8 bit
	uartx->pUartTxDmaCh->CCR |= DMA_CCR_MINC;									//Memory increment mode enabled
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_PINC;									//Peripheral increment mode disabled
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_CIRC;									//Circular mode disabled
	uartx->pUartTxDmaCh->CCR |= DMA_CCR_DIR;									//Read from memory
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_TCIE;									//Transfer complete interrupt disable
	uartx->pUartTxDmaCh->CNDTR = 0;												//Number of data
	uartx->pUartTxDmaCh->CPAR = (uint32_t) &(uartx->pUart->TDR);				//Peripheral address
	uartx->pUartTxDmaCh->CMAR = (uint32_t) NULL;								//Memory address

	//DMA Channel USART RX
	uartx->pUartRxDmaCh->CCR = 0;
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->pUartRxDmaCh->CCR |= DMA_CCR_PL_0;									//Channel priority level - Medium
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_MSIZE;									//Memory size - 8 bit
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_PSIZE;									//Peripheral size - 8 bit
	uartx->pUartRxDmaCh->CCR |= DMA_CCR_MINC;									//Memory increment mode enabled
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_PINC;									//Peripheral increment mode disabled
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_CIRC;									//Circular mode disabled
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_DIR;									//Read from peripheral
	uartx->pUartRxDmaCh->CCR |= DMA_CCR_TCIE;									//Transfer complete interrupt enable
	uartx->pUartRxDmaCh->CNDTR = 0;												//Number of data
	uartx->pUartRxDmaCh->CPAR = (uint32_t) &(uartx->pUart->RDR);				//Peripheral address
	uartx->pUartRxDmaCh->CMAR = (uint32_t) NULL;								//Memory address
}

/*!****************************************************************************
 * @brief
 */
void uart_deinit(uart_type *uartx){
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
}

/*!****************************************************************************
 * @brief	 transfer data buffer
 */
void uart_setBaud(uart_type *uartx, uint32_t baudRate){
	if(uartx->baudRate != baudRate){
		uartx->pUart->BRR = uartx->frequency / baudRate;
		uartx->baudRate = baudRate;
	}
}

/*!****************************************************************************
 * @brief	Set callback uart
 */
void uart_setCallback(uart_type *uartx, uartCallback_type txHoock, uartCallback_type rxHoock){
	uartx->txHoock = txHoock;
	uartx->rxHoock = rxHoock;
}

/*!****************************************************************************
 * @brief
 */
void uart_write(uart_type *uartx, const void *src, uint16_t len){
	uartx->pUart->ICR = USART_ICR_TCCF;
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->pUartTxDmaCh->CMAR = (uint32_t) src;									//Memory address
	uartx->pUartTxDmaCh->CNDTR = len;											//Number of data
	uartx->pUartTxDmaCh->CCR |= DMA_CCR_EN;										//Channel enabled
	uartx->txState = uartTxRun;
}

/******************************************************************************
 * @brief
 */
void uart_read(uart_type *uartx, void *dst, uint16_t len){
	uartx->pUart->RQR = USART_RQR_RXFRQ;
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->pUartRxDmaCh->CMAR = (uint32_t) dst;									//Memory address
	uartx->pUartRxDmaCh->CNDTR = len;											//Number of data.
	uartx->pUartRxDmaCh->CCR |= DMA_CCR_EN;										//Channel enabled
	uartx->rxState = uartRxRun;
}

/******************************************************************************
 * @brief
 */
void uart_stopRead(uart_type *uartx){
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->rxState = uartRxStop;
}

/******************************************************************************
 * Transfer complete interrupt (USART TX and IDLE RX)
 */
void USART_IRQHandler(uart_type *uartx){
	uint16_t uartsr = uartx->pUart->ISR;

	/************************************************
	 * USART TRANSFER COMPLETE
	 */
	if((uartsr & USART_ISR_TC) != 0){
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;								//Channel disabled
		uartx->txCnt++;
		uartx->txState = uartTxSuccess;
		if(uartx->txHoock != NULL){
			uartx->txHoock(uartx);
		}
		*uartx->dmaIfcrTx = uartx->dmaIfcrMaskTx;								//Clear flag
		uartx->pUart->ICR |= USART_ICR_TCCF;
	}
	/************************************************
	 * USART IDLE LINE interrupt
	 */
	else if((uartsr & USART_ISR_IDLE) != 0){
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;								//Channel disabled
		uartx->rxCnt++;
		uartx->rxState = uartRxSuccess;
		if(uartx->rxHoock != NULL){
			uartx->rxHoock(uartx);
		}
		*uartx->dmaIfcrRx = uartx->dmaIfcrMaskRx;								//Clear flag
		//Clear IDLE flag by sequence (read USART_SR register followed by a read to the USART_DR register)
		uartx->pUart->ICR = USART_ICR_IDLECF;
	}

	uartx->pUart->ICR = 0xFFFFFFFF;
}

/******************************************************************************
 * Transfer complete interrupt USART1_IRQn (USART1 TX and IDLE RX)
 */
#if (UART1_USE > 0)
void USART1_IRQHandler(void){
	USART_IRQHandler(uart1);
}
#endif //UART1_USE

/******************************************************************************
 * Transfer complete interrupt USART2_IRQn (USART2 TX and IDLE RX)
 */
#if (UART2_USE > 0)
void USART2_IRQHandler(void){
	USART_IRQHandler(uart2);
}
#endif //UART2_USE

/******************************************************************************
 * Transfer complete interrupt USART3_IRQn (USART3 TX and IDLE RX)
 */
#if (UART3_USE > 0)
void USART3_4_IRQHandler(void){
	USART_IRQHandler(uart3);
}
#endif //UART3_USE

/******************************** END OF FILE ********************************/
