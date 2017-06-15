/*
 * StdIoConnector.c
 *
 *  Created on: 10.06.2017
 *      Author: DL8NCI
 */

#include "StdIoConnector.h"
#include  <sys/unistd.h>
#include <errno.h>



void init(UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_hdma_usart_tx ) {
	huart = _huart;
	hdma_usart_tx = _hdma_usart_tx;
	status = STDIO_CONNECTOR_STATUS_IDLE;
}

// taken from <http://www.openstm32.org/forumthread1055>
int _write(int file, char *data, int len) {

	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

//	if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, len)!=HAL_OK) {
//    if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)data, len)!=HAL_OK) {
    if (HAL_UART_Transmit(huart, (uint8_t*)data, len, 1000)!=HAL_OK) {
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


