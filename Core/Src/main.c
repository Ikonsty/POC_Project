/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "pdm2pcm.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRINT(X)			CDC_Transmit_FS(X, strlen(X))
//#define ARM_MATH_CM4
#define LENGTH				128

const uint16_t FFT_SIZE = 512;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t txBuf[128];
uint16_t pdmRxBuf[128];
uint16_t MidBuffer[16];
uint8_t txstate = 0;
uint8_t rxstate = 0;


uint16_t fifobuf[256];
uint8_t fifo_w_ptr = 0;
uint8_t fifo_r_ptr = 0;
uint8_t fifo_read_enabled = 0;
float fft_out_buf[2048];

arm_rfft_fast_instance_f32 fft_handler;

int i;
float String_Freq[] = {82.41, 110, 146.83, 196, 246.94, 329.63}; // From S1(thickest) to S6(thinnest)

void FifoWrite(uint16_t data) {
	fifobuf[fifo_w_ptr] = data;
	fifo_w_ptr++;
}

uint16_t FifoRead() {
	uint16_t val = fifobuf[fifo_r_ptr];
	fifo_r_ptr++;
	return val;
}

void ReadSample() { //txBuf contain sample
	  if(rxstate == 1) {
		  PDM_Filter(&pdmRxBuf[0], &MidBuffer[0], &PDM1_filter_handler); //MidBuffer is the pcm data
		  for(int i = 0; i < 16; i++) {FifoWrite(MidBuffer[i]);}
		  if(fifo_w_ptr - fifo_r_ptr > 128) fifo_read_enabled = 1;
		  rxstate = 0;
	  }

	  if(rxstate == 2) {
		  PDM_Filter(&pdmRxBuf[64], &MidBuffer[0], &PDM1_filter_handler);
		  for(int i = 0; i < 16; i++) {FifoWrite(MidBuffer[i]); }
		  rxstate = 0;
	  }

	  if (txstate == 1) {
		if (fifo_read_enabled == 1) {
			for (int i = 0; i < 64;i = i + 4) {
				uint16_t data = FifoRead();
				txBuf[i] = data;
				txBuf[i+2] = data;
			}
		}
		txstate=0;
	  }

	  if(txstate == 2) {
		  if(fifo_read_enabled == 1) {
			  for(int i = 64; i < 128; i = i + 4) {
				  uint16_t data = FifoRead();
				  txBuf[i] = data;
				  txBuf[i+2] = data;
			  }
		  }
		  txstate = 0;
	  }
}

float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}

float* DoFFT(float* fft_in_buf) {
	//Do FFT
	arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);

	float freqs[1024];
	int freqpoint = 0;
	int offset = 150; //variable noisefloor offset

	//calculate abs values and linear-to-dB
	for (int i=0; i<2048; i=i+2) {
		freqs[freqpoint] = (float)(20*log10f(complexABS(fft_out_buf[i], fft_out_buf[i+1])))-offset;
		if (freqs[freqpoint]<0) freqs[freqpoint]=0;
		freqpoint++;
	}
	return freqs;
}

int getClosest(float val1, float val2, float freq, int index) {
	if (freq - val1 >= val2 - freq)
		return index+1;
	else
		return index;
}

char* concat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1) + strlen(s2) + 1); // +1 for the \0
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}

char* detectString(float freq) {
	int String = 0;
	if (freq <= String_Freq[0]) String = 1;
	if (freq >= String_Freq[5]) String = 6;

	int m = 0, j = 6, mid = 0;
	while (m < j) {
		mid = (m+j)/2;
		if (freq == String_Freq[mid]) {
			String = mid + 1;
		}

		if (freq < String_Freq[mid]) {
			if (mid > 0 && freq > String_Freq[mid-1]){
				String = getClosest(String_Freq[mid-1], String_Freq[mid], freq, mid-1) + 1;
			}
			j = mid;
		}

		else {
			if (mid < 5 && freq < String_Freq[mid+1]) {
				String = getClosest(String_Freq[mid], String_Freq[mid+1], freq, mid) + 1;
			}
			m = mid + 1;
		}
	}
	char str_res[10];
	sprintf(str_res, "String %i", String);

	if (!String) String = String_Freq[mid];

	if (freq - String_Freq[String - 1] < -1) { //too high, make lower
		char feedback[] = "\tToo high, make lower\n";
		return concat(str_res, feedback);
	}
	if (freq - String_Freq[String - 1] > 1) { //too low, make higher
		char feedback[] = "\tToo low, make it higher\n";
		return concat(str_res, feedback);
	}
	char feedback[] = "\tTuned!\n";
	return concat(str_res, feedback);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2S_Transmit_DMA(&hi2s3, &txBuf[0], 64);
  HAL_I2S_Receive_DMA(&hi2s2, &pdmRxBuf[0],64); //pdm receive

  arm_rfft_fast_init_f32(&fft_handler, 2048);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  sample = ReadSample();
//	  DoFFT(sample);
	  PRINT(detectString(78.3));
	  HAL_Delay(1000);
	  PRINT(detectString(82));
	  HAL_Delay(1000);
	  PRINT(detectString(88.3));
	  HAL_Delay(1000);
	  PRINT(detectString(250.0));
	  HAL_Delay(1000);
	  PRINT(detectString(350.3));
	  HAL_Delay(1000);
	  PRINT(detectString(328));
	  HAL_Delay(1000);

//    size_t ln = sizeof(txBuf)/sizeof(txBuf[0]);
//    for (size_t i = 0; i < ln; i++){
//    	char st[16];
//    	itoa(txBuf[i], st, 16);
//    	PRINT(st);
//    }
//    HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    txstate = 1;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    txstate = 2;
}

void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
	rxstate = 1;
}

void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef *hi2s) {
	rxstate = 2;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
