/*
 * TAquisition.c
 *
 *  Created on: 17.06.2017
 *      Author: DL8NCI
 */


#include <DataAquisition.h>


static uint16_t adc[10];
static volatile int adc_dma_results_available;
static volatile int adc_dma_error_occured;


HAL_StatusTypeDef DAQU_startADC(ADC_HandleTypeDef* hadc, uint16_t n_iter, uint8_t n_chan, TStat *st) {

	uint8_t i_chan;
	uint16_t i_iter;
	uint32_t t, dt;
	HAL_StatusTypeDef rc;

	for (i_chan=0; i_chan<n_chan; i_chan++) STAT_init(&st[i_chan]);

	for (i_iter=0; i_iter<n_iter; i_iter++) {
		for (i_chan=0; i_chan<n_chan; i_chan++) adc[i_chan]=0;

		adc_dma_results_available = 0;
		adc_dma_error_occured = 0;

		rc = HAL_ADC_Start_DMA(hadc, (uint32_t*)adc, 10);
		if (rc!=0) return rc;

		t = HAL_GetTick();
		dt = 0;

		while ((adc_dma_results_available==0)&&(dt<1000)) {
			dt = HAL_GetTick()-t;
			if (dt>1000) return HAL_TIMEOUT;
			}

		if (adc_dma_error_occured!=0) return HAL_ERROR;

		for (i_chan=0; i_chan<n_chan; i_chan++) STAT_add(&st[i_chan],adc[i_chan]);
		}
	return HAL_OK;
	}


void DAQU_printErrorInfo(HAL_StatusTypeDef rc) {
	  switch (rc) {
	  	  case HAL_ERROR:
			  printf("\r\nHAL_ERROR - DAQU_startADC\r\n");
	  		  break;
	  	  case HAL_TIMEOUT:
			  printf("\r\nHAL_TIMEOUT - DAQU_startADC\r\n");
	  		  break;
	  	  case HAL_BUSY:
			  printf("\r\nHAL_BUSY - DAQU_startADC\r\n");
	  		  break;
	  	  case HAL_OK:
	  		  break;
	  	  default:
			  printf("\r\nHAL_UNKNOWN_ERROR - DAQU_startADC: rc = %d\r\n",rc);
	  		  break;
	  	  }
	}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	adc_dma_results_available = 1;
	}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	adc_dma_error_occured = 1;
	}

