/*
 * StdIoConnector.c
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#include "StdIoConnector.h"
#include <sys/unistd.h>
#include <errno.h>


static void outc(uint8_t c) {
	uint8_t b = c;
	HAL_UART_Transmit(huart, &b, 1, 100);
	}

static void reset() {

	buffer.b1.status = stIn;
	buffer.b1.i_from = 0;			// including
	buffer.b1.i = 0;				// ready to write on this index
	buffer.b1.n = 0;				// nu,ber of bytes in buffer
	buffer.b1.i_to = 0;				// excluding
	buffer.b1.i_flush = 0;			// Data up to i_flush are ready to be send (e.g. during next iteration

	buffer.b2.status = stIdle;
	buffer.b2.i_from = 0;
	buffer.b2.i = 0;
	buffer.b2.n = 0;
	buffer.b2.i_to = 0;
	buffer.b2.i_flush = 0;

	buffer.status = stReady;

	outc('R');
	}


void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx) {

	reset();

	buffer.hdma_usart_tx = hdma_usart_tx;
	buffer.huart = huart;
	buffer.nTXDone = 0;
	}



static void outString(char *c) {
	uint16_t i = 0;
	while (c[i]!=0) outc(c[i++]);
	}



void STDIOC_init_alt(UART_HandleTypeDef *_huart) {
	huart = _huart;
	outString("\x1b[2J\x1b[H");
	}

static void outBufferStatus() {
/*
	char b[200];
	char s1,s2;

	switch (buffer.b1.status) {
		case stIdle:
			s1 = 'D';
			break;
		case stIn:
			s1 = 'I';
			break;
		case stOut:
			s1 = 'O';
			break;
		default:
			s1 = '?';
		}

	switch (buffer.b2.status) {
		case stIdle:
			s2 = 'D';
			break;
		case stIn:
			s2 = 'I';
			break;
		case stOut:
			s2 = 'O';
			break;
		default:
			s2 = '?';
		}


	uint16_t nn = buffer.b1.n + buffer.b2.n;
	double load = (double)nn*100.0/(double)BUFSIZE;

	int l = sprintf(b,"\r\ns: %c  fm %4d  i %4d  n %4d  to %4d    s: %c  fm %4d  i %4d  n %4d  to %4d    %5.1f %%     ",
			s1, buffer.b1.i_from,buffer.b1.i, buffer.b1.n, buffer.b1.i_to,
			s2, buffer.b2.i_from,buffer.b2.i, buffer.b2.n, buffer.b2.i_to,
			load);

	if (l>0) outString(b);
*/
	}


static void outNibble(uint8_t x) {
	const char hex[17] = "0123456789ABCDEF";
	outc(hex[x]);
	}

static void outByte(uint8_t x) {
	outNibble((x & 0xf0) >> 4);
	outNibble(x & 0x0f);
	}

static void outWord(uint16_t x) {
	outByte(x >> 8);
	outByte(x & 0xff);
	}


static void __TransferCompleteInterruptDisable() {
	buffer.hdma_usart_tx->Instance->CR &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
//	buffer.hdma_usart_tx->Instance->CR &= ~DMA_IT_TC;
	}

static void __TransferCompleteInterruptEnable() {
	buffer.hdma_usart_tx->Instance->CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
//	buffer.hdma_usart_tx->Instance->CR |= DMA_IT_TC;
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



void STDIOC_idle() {

	struct TBufferStatus *ib;
	struct TBufferStatus *ob;

// try to get a lock
/*
	if (lock(1000)!=stSuccess) {
		errno = EBUSY;
		return -1;
		}
*/

// the parameters of the input and output buffers

// any pending txDone event?
	__TransferCompleteInterruptDisable();
	if (buffer.nTXDone != 0) {
		outc('q');
		ib = currentInBuffer();
		ob = currentOutBuffer();

		outBufferStatus();
		// perform txDone process
		if (ib->n == 0) {
			reset();
			}
		else {
			outc('s');
			ob->n = 0;
			ob->i_from = ob->i_to;
			ob->status = stIdle;
			ib->i_to = ib->i_from;		// the whole buffer is for input
			}
		buffer.nTXDone = 0;

		outBufferStatus();

		if (ib->i_flush > ib->i_from) {

			ob->i_from = ib->i_flush;
			ob->i_flush = ob->i_from;
			ob->i_to = ib->i_from;
			ob->i = ib->i;
			ob->n = ib->n - (ib->i_flush - ib->i_from);
			ob->status = stIn;

			ib->i_to = ib->i_flush;
			ib->i_flush = ib->i_from;
			ib->i = ib->i_from;
			ib->n = ib->i_to - ib->i_from;
			ib->status = stOut;
			}
		else if (ib->i_flush < ib->i_from) {
			ob->i_from = 0;
			ob->i_flush = ib->i_flush;
			ob->i = ib->i;
			ob->n = ib->i;
			ob->i_to = ib->i_from;
			ob->status = stIn;

			ib->i_to = 0;
			ib->i_flush = ib->i_from;
			ib->i = ib->i_from;
			ib->n = BUFSIZE - ib->i_from;
			ib->status = stOut;
			}
		else {
			outc('o');
			__TransferCompleteInterruptEnable();
			return;
			}

		ob = ib;

		if (ob->n > 0) {
			if (HAL_UART_Transmit_DMA(buffer.huart, &buffer.buf[ob->i_from], ob->n)!=HAL_OK) {
				errno = EIO;
				outc('m');
		    	}
			else {
				outc('n');
		    	}
			}
		else
			ob->status = stIdle;
		}
	__TransferCompleteInterruptEnable();

	}


int writeInt(int file, char *data, int len) {

	struct TBufferStatus *ib;
	struct TBufferStatus *ob;

	outBufferStatus();

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
			ob->n = 0;
			ob->i_from = ob->i_to;
			ob->status = stIdle;
			ib->i_to = ib->i_from;		// the whole buffer is for input
			}
		buffer.nTXDone = 0;

		outBufferStatus();

		}
	__TransferCompleteInterruptEnable();



// copy buffer content
	outc('F');
	uint16_t cntg = 0;
	for (uint16_t i = 0; i<len; i++) {
		if ((ib->i == ib->i_to) && (ib->n > 0)) {
			errno = ENOMEM;
			unlock();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

			outString("\x1b[31mG:");
			outWord(cntg);
			outc('-');
			outc('H');
			outString("\x1b[37m");
			return -1;
			}
//		outc('G');
		cntg++;
		buffer.buf[ib->i] = data[i];
		ib->i++;
		ib->n++;
		if (ib->i >= BUFSIZE) ib->i = 0;
		}
	ib->i_flush = ib->i;

	outc('G');
	outc(':');
	outWord(cntg);
	outc('-');

	outc('J');

	outBufferStatus();

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


	if (ib->i > ib->i_from) {
		// whole buffer in linear address space
		ib->i_to = ib->i;
		ib->i = ib->i_from;

		ob->i_from = ib->i_to;
		ob->i = ob->i_from;
		ob->n = 0;
		ob->i_to = ib->i_from;
		ob->i_flush = ob->i_from;
		}
	else {
		// buffer split
		outc('P');
		ob->i = ib->i;
		ob->n = ob->i;
		ob->i_from = 0;
		ob->i_to = ib->i_from;
		ob->i_flush = ib->i_flush;

		ib->i_to = 0;
		ib->n = BUFSIZE - ib->i_from;
		ib->i = ib->i_from;
		}

	ib->status = stOut;
	ob->status = stIn;

	__TransferCompleteInterruptEnable();
	ob = ib;
	unlock();

	outBufferStatus();


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

int _write(int file, char *data, int len) {
	return writeInt(file, data, len);
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


