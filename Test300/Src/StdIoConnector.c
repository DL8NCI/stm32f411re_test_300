/*
 * StdIoConnector.c
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#include "StdIoConnector.h"
#include <sys/unistd.h>
#include <errno.h>


// --> http://www.openstm32.org/Importing+a+STCubeMX+generated+project
// --> http://wiki.eclipse.org/EGit/User_Guide#Creating_a_Repository

struct TBuffer buffer;


static void outString(char *c);
static void checkTransferComplete();
static void copyToBuffer(char *data, int len);
void checkSendBuffer();


void STDIOC_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_tx) {

	buffer.i_from = 0;
	buffer.i = 0;
	buffer.n = 0;
	buffer.i_flush = 0;
	buffer.n_flush = 0;
	buffer.i_to = 0;

	buffer.hdma_usart_tx = hdma_usart_tx;
	buffer.huart = huart;
	buffer.n_tx_done = 0;
	}


void STDIOC_init_alt(UART_HandleTypeDef *_huart_alt) {
	buffer.huart_alt = _huart_alt;
	outString("\x1b[2J\x1b[H");
	}


void STDIOC_idle() {
	checkTransferComplete();
	checkSendBuffer();
	}


int _write(int file, char *data, int len) {

	checkTransferComplete();
	checkSendBuffer();

	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

	copyToBuffer(data,len);
	checkSendBuffer();

	return len;
    }


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance != buffer.huart->Instance) return;
// DMA transmission for TX is done
	if (buffer.n_tx_done==1) buffer.n_tx_done = 2;
//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	}


static void outString(char *c) {
//	uint16_t i = 0;
//	while (c[i]!=0) outc(c[i++]);
	}


static void TransferCompleteInterruptDisable() {
	buffer.hdma_usart_tx->Instance->CR &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
//	buffer.hdma_usart_tx->Instance->CR &= ~DMA_IT_TC;
	}


static void TransferCompleteInterruptEnable() {
	buffer.hdma_usart_tx->Instance->CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
//	buffer.hdma_usart_tx->Instance->CR |= DMA_IT_TC;
	}


void checkSendBuffer() {

	uint16_t
		tx_from,
		tx_n,
		i_flush,
		n_tx_done;

	TransferCompleteInterruptDisable();
	n_tx_done = buffer.n_tx_done;
	TransferCompleteInterruptEnable();

	if (n_tx_done!=0) return;			// tx pending
	if (buffer.n_flush == 0) return;	// nothing to send

	buffer.n_tx_done = 1;
	i_flush = buffer.i_flush;
	if (i_flush==0) i_flush = BUFSIZE;

	tx_from = buffer.i_from;

	// with or without wrap around
	if (i_flush > buffer.i_from) {
		tx_n = buffer.n_flush;
		buffer.i_from = buffer.i_flush;
		}
	else {
		tx_n = BUFSIZE - buffer.i_from;
		buffer.i_from = 0;
		}

	buffer.n -= tx_n;
	buffer.n_flush -= tx_n;

	if (HAL_UART_Transmit_DMA(buffer.huart, &buffer.buf[tx_from], tx_n)!=HAL_OK) {
		errno = EIO;
		return;
    	}
	}


static void checkTransferComplete() {
	uint16_t n_tx_done;

	TransferCompleteInterruptDisable();
	n_tx_done = buffer.n_tx_done;
	TransferCompleteInterruptEnable();

	if (n_tx_done != 2) return;

	buffer.i_to = buffer.i_from;
	buffer.n_tx_done = 0;
	}


static void copyToBuffer(char *data, int len) {

	uint16_t cntg = 0;

	for (uint16_t i = 0; i<len; i++) {
		if (buffer.n == BUFSIZE) {
			errno = ENOMEM;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

			char b[200];
			sprintf(b,"\x1b[31mG:%d-P\x1b[37m",cntg);
			outString(b);
//			outString("\x1b[31mG:");
//			outWord(cntg);
//			outc('-');
//			outc('P');
//			outString("\x1b[37m");
//			for(;;) {}
			return; // -1;
			}
		cntg++;
		buffer.buf[buffer.i] = data[i];
		buffer.i++;
		buffer.n++;
		buffer.n_flush++;
		if (buffer.i >= BUFSIZE) buffer.i = 0;
		}
	buffer.i_flush = buffer.i;
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

