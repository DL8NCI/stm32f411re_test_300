/*
 * StdIoConnector.c
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#include "StdIoConnector.h"
#include <sys/unistd.h>
#include <errno.h>

#include "ArduinoPins.h"
#include "stm32f4xx_hal.h"



// --> http://www.openstm32.org/Importing+a+STCubeMX+generated+project
// --> http://wiki.eclipse.org/EGit/User_Guide#Creating_a_Repository

int maxOverflow;
struct TBuffer buffer;

UART_HandleTypeDef *diag;


uint8_t checkTransferComplete();
int copyToBuffer(char *data, int len);
void checkSendBuffer();
uint8_t getNTxDone();
void setNTxDone(uint8_t x);

void STDIOC_putChar(uint8_t b);
void STDIOC_TxComplete();


uint8_t getCnt();
void clrCnt();

uint32_t transferCompleteInterruptDisable();
void transferCompleteInterruptRestore(uint32_t crBak);



void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx) {

	buffer.i_from = 0;
	buffer.i = 0;
	buffer.n = 0;
	buffer.i_flush = 0;
	buffer.n_flush = 0;
	buffer.i_to = 0;
	buffer.n_capacity = BUFSIZE;

	buffer.hdma_usart_tx = hdma_usart_tx;
	buffer.huart = huart;
	buffer.n_tx_done = 0;

	buffer.cnt = 0;

	maxOverflow = 0;
	}

void STDIOC_initDiag(UART_HandleTypeDef *huart) {
	diag = huart;
	}

void STDIOC_putChar(uint8_t b) {
	HAL_StatusTypeDef rc = HAL_UART_Transmit(diag, &b, 1, 100);
	}


void STDIOC_idle() {
	if (checkTransferComplete() != 0) return;
	checkSendBuffer();
	}

int STDIOC_getMaxOverflow() {
	return maxOverflow;
	}

int _write(int file, char *data, int len) {

	int idx = 0;
	int m;
	int n = len;

	checkTransferComplete();
	checkSendBuffer();

	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

	do {
		m = copyToBuffer(&data[idx],n);
		n -= m;
		idx += m;
		checkSendBuffer();

		// blocking mode if buffer has not enough capacity
		if (n!=0) {
			if (n>maxOverflow) maxOverflow = n;
			while (checkTransferComplete()!=0) { }
			checkSendBuffer();
			}

		} while (n!=0);

	return idx;
    }

uint32_t transferCompleteInterruptDisable() {
	uint32_t crBak = buffer.hdma_usart_tx->Instance->CR & (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
	buffer.hdma_usart_tx->Instance->CR &= ~(crBak);
	return crBak;
	}

void transferCompleteInterruptRestore(uint32_t crBak) {
	if (crBak==0) return;
	buffer.hdma_usart_tx->Instance->CR |= crBak;
	}

void STDIOC_TxComplete() {
	if (buffer.n_tx_done==1) buffer.n_tx_done = 2;
	buffer.cnt++;
	}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(((huart->hdmatx->Instance->CR) & (uint32_t)(DMA_SxCR_DBM)) != RESET) STDIOC_putChar('D'); // double memory mode
	if ((huart->hdmatx->Instance->CR & DMA_SxCR_CIRC) != RESET) STDIOC_putChar('C'); // circular mode
//	HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_RESET);

	if (buffer.n_tx_done==1) buffer.n_tx_done = 2;
	buffer.cnt++;
	if (buffer.cnt!=1) {
		if (buffer.cnt<10) STDIOC_putChar(buffer.cnt+0x30); else STDIOC_putChar('Y');
		}
//	HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_SET);
	}


uint8_t getNTxDone() {
	volatile uint32_t crBak = transferCompleteInterruptDisable();
	uint8_t n_tx_done = buffer.n_tx_done;
	transferCompleteInterruptRestore(crBak);
    return n_tx_done;
	}

void setNTxDone(uint8_t x) {
	volatile uint32_t crBak = transferCompleteInterruptDisable();
	buffer.n_tx_done = x;
	transferCompleteInterruptRestore(crBak);
	}



void clrCnt() {
	volatile uint32_t crBak = transferCompleteInterruptDisable();
	buffer.cnt = 0;
	transferCompleteInterruptRestore(crBak);
	}

uint8_t getCnt() {
	volatile uint32_t crBak = transferCompleteInterruptDisable();
	uint8_t cnt = buffer.cnt;
	transferCompleteInterruptRestore(crBak);
    return cnt;
	}


void checkSendBuffer() {

	uint16_t
		tx_from,
		tx_n;

	if (getNTxDone() != 0) return;		// tx pending
	if (buffer.n_flush == 0) return;	// nothing to send

	setNTxDone(1);
	HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_SET);

	tx_from = buffer.i_from;

	// with or without wrap around
	if ((buffer.i_flush > buffer.i_from) || (buffer.i_flush == 0)) {
		tx_n = buffer.n_flush;
		buffer.i_from = buffer.i_flush;
		}
	else {
		tx_n = BUFSIZE - buffer.i_from;
		buffer.i_from = 0;
		}

	buffer.n -= tx_n;
	buffer.n_flush -= tx_n;
	buffer.n_capacity = BUFSIZE - tx_n;

	if (HAL_UART_Transmit_DMA(buffer.huart, &buffer.buf[tx_from], tx_n)!=HAL_OK) {
		errno = EIO;
		return;
    	}
	}


uint8_t checkTransferComplete() {
	uint8_t n_tx_done = getNTxDone();
	if (n_tx_done != 2) return n_tx_done;
	buffer.i_to = buffer.i_from;
	buffer.n_capacity = BUFSIZE;

	uint8_t h = getCnt();
	if (h!=1) STDIOC_putChar('X');
	clrCnt();
	setNTxDone(0);
	return 0;
	}

/* copies from data to buffer.buf[] at most len bytes.
 * returns index in buf where to continue
 */
int copyToBuffer(char *data, int len) {

	for (uint16_t i = 0; i<len; i++) {
		if (buffer.n == buffer.n_capacity) {
			buffer.i_flush = buffer.i;
			return i;
			}
		buffer.buf[buffer.i] = data[i];
		buffer.i++;
		buffer.n++;
		buffer.n_flush++;
		if (buffer.i >= BUFSIZE) buffer.i = 0;
		}
	buffer.i_flush = buffer.i;
	return len;
	}


/*
int _read(int file, char *data, int len) {
	if (file != STDIN_FILENO) {
        errno = EBADF;
        return -1;
    	}

    if (HAL_UART_Receive(&huart2, (uint8_t*)data, len, 1000)!=HAL_OK) {
    	errno = EIO;
    	return -1;
    	}

    return len;
	}
*/

