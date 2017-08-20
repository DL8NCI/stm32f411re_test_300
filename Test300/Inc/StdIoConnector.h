/*
 * StdIoConnector.h
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#ifndef STDIOCONNECTOR_H_
#define STDIOCONNECTOR_H_

#include "stm32f4xx_hal.h"

#define BUFSIZE 4096

// static enum EBufferStatus { stTXEmpty = 0, stSending, stSent };

struct TBuffer {
	uint8_t buf[BUFSIZE];		// the buffer array
	uint16_t i_from;			// start (including)
	uint16_t i;					// current (next index to write to)
	uint16_t n;					// number of bytes buffered by now
	uint16_t i_flush;			// not yet executed flush excluding this position
	uint16_t n_flush;			// number of bytes to be flushed
	uint16_t i_to;				// end + 1 (excluding)
	volatile uint8_t n_tx_done;	// last buffer has been send out
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_usart_tx;
	UART_HandleTypeDef *huart_alt;
	};

void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx);
void STDIOC_init_alt(UART_HandleTypeDef *_huart_alt);
void STDIOC_idle();

#endif /* STDIOCONNECTOR_H_ */
