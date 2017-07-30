/*
 * StdIoConnector.c
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#include "StdIoConnector.h"
#include <sys/unistd.h>
#include <errno.h>


void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx) {

	reset();

	buffer.hdma_usart_tx = hdma_usart_tx;
	buffer.huart = huart;
	buffer.nTXDone = 0;
	}


static void reset() {

	buffer.b1.status = stIn;
	buffer.b1.i_from = 0;
	buffer.b1.i = 0;
	buffer.b1.n = 0;
	buffer.b1.i_to = BUFSIZE-1;

	buffer.b2.status = stIdle;
	buffer.b2.i_from = 0;
	buffer.b2.i = 0;
	buffer.b2.n = 0;
	buffer.b2.i_to = 0;

	buffer.status = stReady;
	}

void STDIOC_init_alt(UART_HandleTypeDef *_huart) {
	huart = _huart;

	outc('\x1b');
	outc('[');
	outc('2');
	outc('J');
	}

static void outc(uint8_t c) {
	uint8_t b = c;
	HAL_UART_Transmit(huart, &b, 1, 100);
	}


static void __TransferCompleteInterruptDisable() {
//	buffer.hdma_usart_tx->Instance->CR &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
	}

static void __TransferCompleteInterruptEnable() {
//	buffer.hdma_usart_tx->Instance->CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
	}

static struct TBufferStatus *currentInBuffer() {
	if (buffer.b1.status == stIn) return &(buffer.b1); else return &(buffer.b2);
	}

static struct TBufferStatus *currentOutBuffer() {
	if (buffer.b1.status != stIn) return &(buffer.b1); else return &(buffer.b2);
	}

static enum EBufferStatus lock(uint32_t timeout) {

	uint32_t t = HAL_GetTick();
	uint32_t dt = 0;
	enum EBufferStatus rc = stBusy;

	do {
		__TransferCompleteInterruptDisable();
		if (buffer.status == stReady) {
			buffer.status = stLocked;
			__TransferCompleteInterruptEnable();
			rc = stSuccess;
			}
		else{
			__TransferCompleteInterruptEnable();
			dt = HAL_GetTick() - t;
			}
		} while ((rc == stBusy) && (dt <= timeout));

	if (dt>timeout) rc = stTimeout;
	return rc;
	}


static void unlock() {
/*
	__TransferCompleteInterruptDisable();
	buffer.status = stReady;
	__TransferCompleteInterruptEnable();
*/
	}


int _write(int file, char *data, int len) {

	struct TBufferStatus *ib;
	struct TBufferStatus *ob;

	outc('A');
// some basic checks
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

// try to get a lock
/*
	if (lock(1000)!=stSuccess) {
		errno = EBUSY;
		return -1;
		}
*/

// the parameters of the input and output buffers
	ib = currentInBuffer();
	ob = currentOutBuffer();

// any pending txDone event?
	__TransferCompleteInterruptDisable();
	outc('B');
	if (buffer.nTXDone != 0) {
		// perform txDone process
		outc('C');
		if (ib->n == 0) {
			outc('D');

			reset();
			ib = currentInBuffer();
			ob = currentOutBuffer();
			}
		else {
			outc('E');
			ob->status = stIdle;
			ib->i_to = ib->i_from;
			if (ib->i_to == 0) ib->i_to = BUFSIZE - 1; else ib->i_to--;
			}
		buffer.nTXDone = 0;
		}
	__TransferCompleteInterruptEnable();



// copy buffer content
	outc('F');
	for (uint16_t i = 0; i<len; i++) {
		if (ib->i == ib->i_to) {
			errno = ENOMEM;
			unlock();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			outc('H');
			return -1;
			}
		outc('G');
		buffer.buf[ib->i] = data[i];
		ib->i++;
		ib->n++;
		if (ib->i >= BUFSIZE) ib->i = 0;
		}

	outc('J');

// pending DMA? - we can not yet send - done
	__TransferCompleteInterruptDisable();
	if (ob->status == stOut) {
		__TransferCompleteInterruptEnable();
		unlock();
		outc('K');
		return len;
		}

	outc('L');

// we can use DMA - swap in and out buffer
	ib->status = stOut;
	ib->i_to = ib->i;
	ib->i = ib->i_from;

	ob->status = stIn;
	ob->i_from = ib->i_to + 1;
	if (ob->i_from >= BUFSIZE) ob->i_from = 0;
	ob->i = ob->i_from;
	ob->n = 0;
	ob->i_to = ib->i_from;
	if (ob->i_to == 0) ob->i_to = BUFSIZE-1; else ob->i_to--;

	__TransferCompleteInterruptEnable();
	ob = ib;
	unlock();

// number of bytes to be sent - toDo
//	uint16_t l;
//	if (ob->i_to > ob->i_from) l = ob->i_to - ob->i_from; else l = BUFSIZE - ob->i_from;

// setup DMA transfer
	if (HAL_UART_Transmit_DMA(buffer.huart, &buffer.buf[ob->i_from], ob->n)!=HAL_OK) {
    	errno = EIO;
    	outc('M');
    	return -1;
    	}
    else {
    	outc('N');
    	return len;
    	}
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance != buffer.huart->Instance) return;
// DMA transmission for TX is done
	buffer.nTXDone++;
//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	outc('I');
	}


