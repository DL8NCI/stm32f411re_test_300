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


void STDIOC_init(UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_hdma_usart_tx );

static txBuffer[BUFSIZE];
static UART_HandleTypeDef *huart;
static DMA_HandleTypeDef *hdma_usart_tx;
static int status;

#endif /* STDIOCONNECTOR_H_ */
