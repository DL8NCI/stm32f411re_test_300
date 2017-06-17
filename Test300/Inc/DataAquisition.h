/*
 * Aquisition.h
 *
 *  Created on: 17.06.2017
 *      Author: DL8NCI
 */

#ifndef DATAAQUISITION_H_
#define DATAAQUISITION_H_

#include "stm32f4xx_hal.h"
#include  <sys/unistd.h>
#include "Statistics.h"


HAL_StatusTypeDef DAQU_startADC(ADC_HandleTypeDef* hadc, uint16_t n_iter, uint8_t n_chan, TStat *st);
void DAQU_printResult();
void DAQU_printErrorInfo(HAL_StatusTypeDef rc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

#endif /* DATAAQUISITION_H_ */
