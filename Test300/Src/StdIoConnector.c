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
	txBuffer.fillingBuffer = &(txBuffer.b1);
	txBuffer.sendingBuffer = 0;
	txBuffer.huart = huart;
	txBuffer.hdma_usart_tx = hdma_usart_tx;

	txBuffer.b1.status = STDIO_CONNECTOR_STATUS_IDLE;
	txBuffer.b1.i_from = 0;
	txBuffer.b1.i_to = 0;

	txBuffer.b2.status = STDIO_CONNECTOR_STATUS_IDLE;
	txBuffer.b2.i_from = 0;
	txBuffer.b2.i_to = 0;
	}

// taken from <http://www.openstm32.org/forumthread1055>
int _write(int file, char *data, int len) {

	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

//	if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, len)!=HAL_OK) {
//    if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)data, len)!=HAL_OK) {
    if (HAL_UART_Transmit(txBuffer.huart, (uint8_t*)data, len, 1000)!=HAL_OK) {
    	errno = EIO;
    	return -1;
    	}
    else return len;
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

	}


