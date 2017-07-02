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



#define HIH8000_ST_NORMAL_OPERATION 0;
#define HIH8000_ST_STALE_DATA 1;
#define HIH8000_ST_COMMAND_MODE 2;


struct DAQU_HIH8000_result {
	uint8_t status;
	uint16_t humidity;
	uint16_t temperature;
	};




HAL_StatusTypeDef DAQU_startADC(ADC_HandleTypeDef* hadc, uint16_t n_iter, uint8_t n_chan, TStat *st);
void DAQU_printResult();
void DAQU_printErrorInfo(HAL_StatusTypeDef rc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

HAL_StatusTypeDef DAQU_start_HIH8000(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef DAQU_read_HIH8000(I2C_HandleTypeDef *hi2c, struct DAQU_HIH8000_result *r);
double DAQU_HIH8000_get_RelativeHumidity(struct DAQU_HIH8000_result *r);
double DAQU_HIH8000_get_Temperature(struct DAQU_HIH8000_result *r);
double DAQU_HIH8000_getDewPointTemperature(struct DAQU_HIH8000_result *r);


#endif /* DATAAQUISITION_H_ */
