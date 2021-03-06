/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include  <sys/unistd.h>
#include <errno.h>
#include "StdIoConnector.h"
#include "Statistics.h"
#include "ArduinoPins.h"
#include "DataAquisition.h"
#include "VT100.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t spi_out[12] = { 0x06, 0x00, 0x00,  0x06, 0x40, 0x00,  0x06, 0x80, 0x00,  0x06, 0xc0, 0x00}; // ch0-ch3 in single ended mode
uint8_t spi_in[12];

double tAmb = 25.0;		// Umgebungstemperatur
double uSupp5 = 5.0;	// Versorgungsspannung 5.0 V

struct TDAQU_HIH8000_result hih8000_result;
HAL_StatusTypeDef hih8000_status;

volatile uint32_t counter_2_1 = 0; // counter 2, channel 1
volatile uint8_t  trigger_detected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */


  GPIO_InitTypeDef itd;

  // some common setting for output pins:
  itd.Mode = GPIO_MODE_OUTPUT_PP;
  itd.Pull = GPIO_NOPULL;
  itd.Speed = GPIO_SPEED_HIGH;

  // LED
  itd.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT,&itd);

  // PB2: !CS for MCP3204 (12 bit ADC)
  itd.Pin = CS_3204_PIN;
  HAL_GPIO_Init(CS_3204_PORT,&itd);

  // deselect MCP3204
  HAL_GPIO_WritePin(CS_3204_PORT,CS_3204_PIN,GPIO_PIN_SET);

  // some common settings for input pins
  itd.Mode = GPIO_MODE_INPUT;
  itd.Pull = GPIO_NOPULL;
  itd.Speed = GPIO_SPEED_HIGH;

  // User-Key
  itd.Pin = USER_KEY_PIN;
  HAL_GPIO_Init(USER_KEY_PORT,&itd);


  STDIOC_init(&huart2,&hdma_usart2_tx);
  STDIOC_initDiag(&huart1);

  VT100EraseScreen();
  VT100CursorHome();

  printf("The quick brown fox jumps over the lazy dog back\r\n");

  HAL_Delay(3000);
  VT100EraseScreen();
  VT100CursorHome();


/* Pinbelegung
 *
 * Funktion   Arduino           morpho
 * IN0:	PA0 - CN8-1 (A0)		CN7-28 (PA0)
 * IN1:	PA1 - CN8-2 (A1)		CN7-30 (PA1)
 * IN4:	PA4 - CN8-3 (A2)		CN7-32 (PA4)
 * IN5: PA5 - CN5-6 (D13)		CN10-11 (PA5)
 * IN6: PA6 - CN5-5 (D12)		CN10-13 (PA6)
 * IN7:	PA7 - CN5-4 (D11)		CN10-15 (PA7)
 * IN8:	PB0 - CN8-4 (A3)		CN7-34 (PB0)
 * IN9: PB1						CN10-24 (PB1)
 * IN10:PC0 - CN8-6 (A5)		CN7-38 (PC0/PB8)
 * IN11:PC1 - CN8-5 (A4)		CN7-36 (PC1/PB9)
 * IN12:PC2						CN7-35 (PC2)
 * IN13:PC3						CN7-37 (PC3)
 * IN14:PC4						CN10-34 (PC4)
 * IN15:PC5						CN10-6 (PC5)
 *
 * Project Setup: http://www.openstm32.org/Importing+a+STCubeMX+generated+project
 *
 */

  TStat st[N_CHANNELS];
  TStat st3204[4];
  uint8_t row;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int i;

	  GPIO_PinState x = HAL_GPIO_ReadPin(USER_KEY_PORT,USER_KEY_PIN);
//	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,x);
//	  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

	  hih8000_status = DAQU_start_HIH8000(&hi2c1);

	  HAL_StatusTypeDef rc = DAQU_startADC(&hadc1, N_ITERATIONS, N_CHANNELS, st);
	  if (rc!=0) DAQU_printErrorInfo(rc);

	  for(i=0; i<4; i++) STAT_init(&st3204[i]);
	  // select MCP3204, external ADC
	  for (uint16_t i_iter=0; i_iter<N_ITERATIONS; i_iter++) {
		  for (i=0; i<4; i++) {
			  HAL_GPIO_WritePin(CS_3204_PORT,CS_3204_PIN,GPIO_PIN_RESET);
			  HAL_StatusTypeDef rc1 = HAL_SPI_TransmitReceive(&hspi2, &spi_out[3*i], &spi_in[3*i], 3, 100);
			  HAL_GPIO_WritePin(CS_3204_PORT,CS_3204_PIN,GPIO_PIN_SET);
			  if (rc==HAL_OK) {
				  uint16_t y = ((((uint16_t)spi_in[1+3*i])<<8)|spi_in[2+3*i]) & 0x0fff;
				  STAT_add(&st3204[i],y);
			  	  }
			  else DAQU_printErrorInfo(rc1);
		  	  }
	  	  }


	  if (hih8000_status==HAL_OK) {
		  hih8000_status = DAQU_read_HIH8000(&hi2c1,&hih8000_result);
	  	  }

	  row = 1;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch0 - 1.22 V:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[0]);						// STM32F411re - ch0 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[0],3.3,4096);			// STM32F411re - ch0 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("3204   - ch0 - 1.22 V:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st3204[0]);					// MCP3204     - ch0 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st3204[0],3.0,4096);		// MCP3204     - ch0 - Volt


	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch1 - 3.0 V:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[1]);						// STM32F411re - ch1 - cnts (gemessene Referenz vom MCP3204)
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[1],3.3,4096);			// STM32F411re - ch1 - Volt (gemessene Referenz vom MCP3204)

	  row++;
	  VT100CursorGoto(row,1);
	  printf("3204   - ch2 - 5 V:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st3204[2]);					// MCP3204     - ch2 - cnts - 5 V
	  VT100CursorGoto(row,50);
	  uSupp5 = STAT_printVolt(&st3204[2],3.0*8.0/4.7,4096);		// MCP3204     - ch2 - Volt

	  row += 2;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch4:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[2]);						// STM32F411re - ch4 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[2],3.3,4096);			// STM32F411re - ch4 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch4 - RH:");
	  VT100CursorGoto(row,50);
	  STAT_printRH(&st[2], 3.3, uSupp5, tAmb);		// STM32F411re - ch4 - RH (via HIH4000)

	  row++;
	  VT100CursorGoto(row,1);
	  printf("3204   - ch3:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st3204[3]);					// MCP3204     - ch3 - cnts - 5 V
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st3204[3],3.0,4096);		// MCP3204     - ch3 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("3204   - ch3 - RH:");
	  VT100CursorGoto(row,50);
	  STAT_printRH(&st3204[3], 3.0, uSupp5, tAmb);		// MCP3204     - ch3 - RH (via HIH4000)

/* irgendwie tut der nicht
	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch5 - 3.3V:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[3]);						// STM32F411re - ch5 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[3],3.3,4096);			// STM32F411re - ch5 - Volt
*/

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch6:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[4]);						// STM32F411re - ch6 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[4],3.3,4096);			// STM32F411re - ch6 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch6 - LDR:");
	  VT100CursorGoto(row,50);
	  STAT_printLux(&st[4],uSupp5);				// STM32F411re - ch6 - LDR

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch7 - 3.3 V");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[5]);						// STM32F411re - ch7 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[5],3.3,4096);			// STM32F411re - ch7 - Volt
/*
	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch8:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[6]);						// STM32F411re - ch8 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[6],3.3,4096);			// STM32F411re - ch8 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch10:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[7]);						// STM32F411re - ch10 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[7],3.3,4096);			// STM32F411re - ch10 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch11:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[8]);						// STM32F411re - ch11 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[8],3.3,4096);			// STM32F411re - ch11 - Volt
*/

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch9:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st[9]);						// STM32F411re - ch9 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st[9],3.3,4096);			// STM32F411re - ch9 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("32F411 - ch9 - CPU-T:");
	  VT100CursorGoto(row,50);
	  STAT_printCpuTemperature(&st[9]);			// STM32F411re - Temperatur - deg C

	  row++;
	  VT100CursorGoto(row,1);
	  printf("3204   - ch1:");
	  VT100CursorGoto(row,25);
	  STAT_print(&st3204[1]);					// MCP3204     - ch1 - cnts
	  VT100CursorGoto(row,50);
	  STAT_printVolt(&st3204[1],3.0,4096);		// MCP3204     - ch1 - Volt

	  row++;
	  VT100CursorGoto(row,1);
	  printf("3204   - ch1 - T:");
	  VT100CursorGoto(row,50);
	  STAT_printPT1000Temperature(&st3204[1], uSupp5, 3.0);		// MCP3204     - ch1 - PT1000


	  if (hih8000_status==HAL_OK) {
		  row += 2;
		  VT100CursorGoto(row,1);
		  printf("HIH8000      - RH:");
		  VT100CursorGoto(row,50);
		  printf("%7.1f %%",DAQU_HIH8000_get_RelativeHumidity(&hih8000_result));

		  row++;
		  VT100CursorGoto(row,1);
		  printf("HIH8000      - T:");
		  VT100CursorGoto(row,50);
		  tAmb = DAQU_HIH8000_get_Temperature(&hih8000_result);
		  printf("%7.1f deg C",tAmb);

		  row++;
		  VT100CursorGoto(row,1);
		  printf("HIH8000      - DP:");
		  VT100CursorGoto(row,50);
		  printf("%7.1f deg C",DAQU_HIH8000_getDewPointTemperature(&hih8000_result));

	  	  }
	  else {
		  row += 3;
	  }
	  printf("\r\n");


	  printf("\r\n");

	  counter_2_1 = 0;
	  trigger_detected = 0;

//	  htim2.Instance->SR = 0;

	  rc = HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

	  if (rc==0) {

		  uint32_t t = HAL_GetTick();
		  uint32_t dt;

		  do {
			  if (trigger_detected==2) {
//				  printf("trigger detected\r\n");
				  trigger_detected = 3;
			  	  }
		  	  dt = HAL_GetTick() - t;
		  	  } while (trigger_detected<11 && (dt<100));

		  uint16_t ov = STDIOC_getMaxOverflow();
		  if (dt<100) {
			  printf("cnt = %ld     ovr = %d\r\n",counter_2_1, ov);
			  double m = 12.90322580645162; // uT/us
			  double t = -151.9354838709678; // uT
			  double tcorr = 1.0/(0.015 + (10*96.0)/(double)counter_2_1);
			  double b = m * tcorr + t;

			  printf("T   = %6.3f us     Tcorr = %6.3f us     B = %7.3f uT\r\n",(double)counter_2_1/(10*96.0), tcorr, b );
		  	  }
		  else {
			  printf("Timeout - cnt = %ld     ovr = %d\r\n",htim2.Instance->CNT, ov);
		  	  }
	  	  }
	  else {
		  printf("Start error (rc = %d)\r\n", rc);
	  	  }


	  rc = HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	  if (rc!=0) printf("Stop error (rc = %d)\r\n",rc);



	  if (x==GPIO_PIN_RESET) HAL_Delay(1000); else HAL_Delay(500);

	  uint32_t t, dt;

	  t = HAL_GetTick();
	  dt = 0;

	  while (dt<1000) {
		  STDIOC_idle();
		  dt = HAL_GetTick()-t;
	  	  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 69000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 3;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  {
//	HAL_GPIO_TogglePin(D3_PORT,D3_PIN);
//	}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance!=TIM2) return;
	if ((trigger_detected>=1) && (trigger_detected<=10))
		counter_2_1 += HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	trigger_detected++;
	}
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim) {
//	trigger_detected++;
//	htim->Instance->SR &= ~0x04UL;
	}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
