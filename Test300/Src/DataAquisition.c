/*
 * TAquisition.c
 *
 *  Created on: 17.06.2017
 *      Author: DL8NCI
 */


#include <DataAquisition.h>
#include "ArduinoPins.h"
#include <math.h>


static uint16_t adc[10];
static volatile int adc_dma_results_available;
static volatile int adc_dma_error_occured;
static uint32_t t;



HAL_StatusTypeDef DAQU_startADC(ADC_HandleTypeDef* hadc, uint16_t n_iter, uint8_t n_chan, TStat *st) {

	uint8_t i_chan;
	uint16_t i_iter;
	uint32_t dt;
	HAL_StatusTypeDef rc;

	for (i_chan=0; i_chan<n_chan; i_chan++) STAT_init(&st[i_chan]);

	for (i_iter=0; i_iter<n_iter; i_iter++) {
		for (i_chan=0; i_chan<n_chan; i_chan++) adc[i_chan]=0;

		adc_dma_results_available = 0;
		adc_dma_error_occured = 0;

		// Pin D2 geht auf 1 bei Beginn der Wandlungen und geht am Ende auf 0 in ISR
		HAL_GPIO_WritePin(D2_PORT,D2_PIN,GPIO_PIN_SET);
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
	HAL_GPIO_WritePin(D2_PORT,D2_PIN,GPIO_PIN_RESET);
	adc_dma_results_available = 1;
	}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	adc_dma_error_occured = 1;
	}

HAL_StatusTypeDef DAQU_start_HIH8000(I2C_HandleTypeDef *hi2c) {
	uint8_t tx = 0x00;
	return HAL_I2C_Master_Transmit(hi2c, 0x4e, &tx, 1, 1000);
	}


HAL_StatusTypeDef DAQU_read_HIH8000(I2C_HandleTypeDef *hi2c, struct TDAQU_HIH8000_result *r) {
	uint8_t b[4];
	HAL_StatusTypeDef rc = HAL_I2C_Master_Receive(hi2c, 0x4f, &b[0], 4, 1000);
	if (rc!=HAL_OK) return rc;
	r->status = b[0]>>6;
	r->humidity = (((uint16_t)b[0] & 0x003f) << 8) | (uint16_t)b[1];
	r->temperature = ((uint16_t)b[2]<<6) | ((uint16_t)b[3]>>2);
	return HIH8000_ST_NORMAL_OPERATION;
	return rc;
	}

double DAQU_HIH8000_get_RelativeHumidity(struct TDAQU_HIH8000_result *r) {
	return (double)r->humidity/163.820;
	}

double DAQU_HIH8000_get_Temperature(struct TDAQU_HIH8000_result *r) {
	return (double)r->temperature*165.0/16382.0 - 40.0;
	}

double DAQU_HIH8000_getDewPointTemperature(struct TDAQU_HIH8000_result *r) {
	double rh = (double)r->humidity/163.820;;
	double t = (double)r->temperature*165.0/16382.0 - 40.0;
	return pow(rh/100.0,1.0/8.02)*(109.8+t)-109.8;
	}

void DAQU_TIM_dumpRegisters(TIM_HandleTypeDef *htim) {
	printf("TIM2_CR1   = %08lx\r\n",htim->Instance->CR1);
	printf("TIM2_CR2   = %08lx\r\n",htim->Instance->CR2);
	printf("TIM2_SMCR  = %08lx\r\n",htim->Instance->SMCR);
	printf("TIM2_DIER  = %08lx\r\n",htim->Instance->DIER);
	printf("TIM2_SR    = %08lx\r\n",htim->Instance->SR);
	printf("TIM2_EGR   = %08lx\r\n",htim->Instance->EGR);
	printf("TIM2_CCMR1 = %08lx\r\n",htim->Instance->CCMR1);
	printf("TIM2_CCMR2 = %08lx\r\n",htim->Instance->CCMR2);
	printf("TIM2_CCER  = %08lx\r\n",htim->Instance->CCER);
	printf("TIM2_CNT   = %08lx\r\n",htim->Instance->CNT);
	printf("TIM2_PSC   = %08lx\r\n",htim->Instance->PSC);
	printf("TIM2_ARR   = %08lx\r\n",htim->Instance->ARR);
	printf("TIM2_CCR1  = %08lx\r\n",htim->Instance->CCR1);
	printf("TIM2_CCR2  = %08lx\r\n",htim->Instance->CCR2);
	printf("TIM2_CCR3  = %08lx\r\n",htim->Instance->CCR3);
	printf("TIM2_CCR4  = %08lx\r\n",htim->Instance->CCR4);
	printf("TIM2_DCR   = %08lx\r\n",htim->Instance->DCR);
	printf("TIM2_DMAR  = %08lx\r\n",htim->Instance->DMAR);
	printf("TIM2_OR    = %08lx\r\n\r\n",htim->Instance->OR);
	}


