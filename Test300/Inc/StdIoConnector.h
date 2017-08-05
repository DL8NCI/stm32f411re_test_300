/*
 * StdIoConnector.h
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#ifndef STDIOCONNECTOR_H_
#define STDIOCONNECTOR_H_

#include "stm32f4xx_hal.h"


#define BUFSIZE 2048

static enum EBufferStatus { stSuccess = 0, stIdle, stIn, stOut, stReady, stLocked, stBusy, stTimeout };


static struct TBufferStatus {
	enum EBufferStatus status;
	uint16_t i_from;	// start (including)
	uint16_t i;			// current (next index to write to)
	uint16_t n;			// number of bytes buffered by now
	uint16_t i_to;		// end + 1 (excluding)
	};

static struct TBuffer {
	volatile enum EBufferStatus status;
	volatile uint8_t nTXDone;
	uint8_t buf[BUFSIZE];
	struct TBufferStatus b1;
	struct TBufferStatus b2;
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_usart_tx;
	};

static struct TBuffer buffer;
static UART_HandleTypeDef *huart;

void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx);

//static void reset();
//static struct TBufferStatus *currentInBuffer();
//static struct TBufferStatus *currentOutBuffer();
//static enum EBufferStatus lock(uint32_t timeout);
//static void unlock();
//static void __TransferCompleteInterruptDisable();
//static void __TransferCompleteInterruptEnable();


void STDIOC_init_alt(UART_HandleTypeDef *_huart);
//static void outc(uint8_t c);

void STDIOC_idle();

#endif /* STDIOCONNECTOR_H_ */
