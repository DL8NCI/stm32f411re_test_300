/*
 * StdIoConnector.h
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#ifndef STDIOCONNECTOR_H_
#define STDIOCONNECTOR_H_

#include "stm32f4xx_hal.h"

#define STDIO_CONNECTOR_STATUS_IDLE 0
#define STDIO_CONNECTOR_STATUS_BUSY -1

#define BUFSIZE 1024


struct STDIOC_txBufferInt {
	uint8_t buf[BUFSIZE];
	uint8_t status;
	uint16_t i_from, i_to;
	};

struct STDIOC_txBuffer {
	struct STDIOC_txBufferInt b1,b2;
	struct STDIOC_txBufferInt *sendingBuffer, *fillingBuffer;
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_usart_tx;
	};

static struct STDIOC_txBuffer txBuffer;

void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx);


#endif /* STDIOCONNECTOR_H_ */
