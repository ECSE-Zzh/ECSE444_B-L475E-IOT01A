/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include <limits.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t sineValueC7[20]=
		{100, 131, 159, 181, 195, 200, 195, 181,
				159, 131, 100, 69, 41, 19, 5, 0,
				5, 19, 41, 69};


uint32_t sineValue1400[29]=
		{100, 121, 142, 161, 176, 188, 196, 200,
				199, 193, 183, 169, 152, 132, 111, 89,
				68, 48, 31, 17, 7, 1, 0, 4,
				12, 24, 39, 58, 79};

uint32_t sineValueC6[40]=
		{100, 116, 131, 145, 159, 171, 181, 189,
				195, 199, 200, 199, 195, 189, 181, 171,
				159, 145, 131, 116, 100, 84, 69, 55,
				41, 29, 19, 11, 5, 1, 0, 1,
				5, 11, 19, 29, 41, 55, 69, 84};

uint32_t sineValue1600[25]=
		{100, 125, 148, 168, 184, 195, 200, 198,
				190, 177, 159, 137, 113, 87, 63, 41,
				23, 10, 2, 0, 5, 16, 32, 52,
				75};

uint32_t sineValue1200[33]=
		{100, 119, 137, 154, 169, 181, 191, 197,
				200, 199, 195, 187, 176, 162, 146, 128,
				110, 90, 72, 54, 38, 24, 13, 5,
				1, 0, 3, 9, 19, 31, 46, 63,
				81};

uint32_t sineValue1800[23]=
		{100, 127, 152, 173, 189, 198, 200, 194,
				182, 163, 140, 114, 86, 60, 37, 18,
				6, 0, 2, 11, 27, 48, 73};

	int counter = 0;

	int flag = 0;
	int loopDMA = 0;
	int toneNum = 0;
	int bufferSize = 20000;
	int32_t micBuffer[20000];

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  		HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 50;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
//  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blueBtn_Pin */
  GPIO_InitStruct.Pin = blueBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blueBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*-------------------------------------------PART_1--------------------------------------------------*/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sineValue[counter]);
//	if(counter == 19){
//		counter = 0;
//	}else{
//		counter++;
//	}
//}
/*-------------------------------------------PART_1--------------------------------------------------*/


/*---------------------------------------------PART_2--------------------------------------------------*/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == blueBtn_Pin){
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//		if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin)== 0){
//			HAL_TIM_Base_Stop_IT(&htim2);
//		} else {
//			HAL_TIM_Base_Start_IT(&htim2);
//			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValueC7, 20, DAC_ALIGN_8B_R);
//		}
//	}

//	if (GPIO_Pin == blueBtn_Pin){
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	}
//
//	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValueC7, 20, DAC_ALIGN_8B_R);
//}
/*----------------------------------------------PART_2--------------------------------------------------*/


/*-------------------------------------PART_3--------------------------------------------------*/
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//blink
	if (GPIO_Pin == blueBtn_Pin){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	}

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, micBuffer, bufferSize);
}

void HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter){
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	//find the max and the min value in micBuffer
	for(int i = 0; i < bufferSize; i++){
		micBuffer[i] >> 8;//discard the useless 8 bits
	}

	int max = INT_MIN;
	int min = INT_MAX;
	for(int i = 0; i < bufferSize; i++){
		if(micBuffer[i] < min){
			min = micBuffer[i];
		}
		if(micBuffer[i] > max){
			max = micBuffer[i];
		}
	}
	float diff = max - min;

	for(int i = 0; i < bufferSize; i++){
		micBuffer[i] = (uint32_t)(((micBuffer[i]-min)/(diff))*4000);
	}

	HAL_TIM_Base_Stop_IT(&htim3);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, micBuffer, bufferSize, DAC_ALIGN_12B_L);
	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3){
		counter++;
		if(counter == 100){
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			counter = 0;
		}
	}
}
*/
/*-----------------------------------PART_3--------------------------------------------------------*/

/*---------------------------------------------PART_4--------------------------------------------------------*/

/*HAL_GPIO_EXTI_Callback: Detect button press. If pressed, go into this function*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == blueBtn_Pin){
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		flag++;
	}

	// Start rec.
	if(flag % 2 == 1){
		HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, micBuffer, bufferSize);//rec
		//play back
	}else if(flag % 2 == 0){
			if(toneNum > 6){
				toneNum = 0;
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 0; loopDMA++){
				HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValueC7, 20, DAC_ALIGN_12B_R);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
				}
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 1; loopDMA++){
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValue1400, 29, DAC_ALIGN_12B_R);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
				}
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 2; loopDMA++){
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValueC6, 40, DAC_ALIGN_12B_R);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
				}
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 3; loopDMA++){
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValue1600, 25, DAC_ALIGN_12B_R);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
				}
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 4; loopDMA++){
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValue1200, 33, DAC_ALIGN_12B_R);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
				}
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 5; loopDMA++){
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineValue1800, 23, DAC_ALIGN_12B_R);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
				}
			}

			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			for(loopDMA = 0; loopDMA <=900000 && toneNum == 6; loopDMA++){
				HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, micBuffer, bufferSize, DAC_ALIGN_12B_R);
				HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
				if(loopDMA == 900000){
					loopDMA = 0;
					toneNum++;
					break;
				}
			}
	}

}

void HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter){
		//find the max and the min value in micBuffer
		for(int i = 0; i < bufferSize; i++){
			micBuffer[i] >> 8;//discard the useless 8 bits
		}
		int max = INT_MIN;
		int min = INT_MAX;
		for(int i = 0; i < bufferSize; i++){
			if(micBuffer[i] < min){
				min = micBuffer[i];
			}
			if(micBuffer[i] > max){
				max = micBuffer[i];
			}
		}
		float diff = max - min;
		for(int i = 0; i < bufferSize; i++){
			micBuffer[i] = (uint32_t)(((micBuffer[i]-min)/(diff))*4000);
		}
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

/*---------------------------------------------PART_4--------------------------------------------------------*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
