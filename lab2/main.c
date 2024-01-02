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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */
	//private variable
	int triangle;
	int flag_tri = 1;
	int saw;
	int flag_saw = 1;
	int sine;
	float sample = 0;
	float pi = 3.1415926535;
	float temperatureC;
	float temp;
	int pressCount = 0;
	int flag_LED = 0;

	float playTemperature();
	void playSine();
	void playSaw();
	void playSaw_for();
	void playTriangle();
	void playSaw_Temperature(float temp);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  //float temperatureC;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  part 1
// 	  default on
//	  if(HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin)== 0){
//		  HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
//	  } else {
//		  HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
//	  }

//////////////////////////////////////////////////////////////////////////////////////////////
	  //part 2:
//	  int flag_tri = 1;
//	  int flag_saw = 1;
//	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
//	  while(1){
//		  //Triangle
//		  if(flag_tri == 1){
//			  triangle += 200;
//			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, triangle);
//			  if(triangle >= 2000){
//				  flag_tri = 0;
//			  }
//		  } else if(flag_tri == 0){
//			  triangle -= 200;
//			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, triangle);
//			  if(triangle <= 0){
//				  flag_tri = 1;
//			  }
//		  }
//
//		  //Saw
//		  if(flag_saw == 1){
//		  	  saw += 100;
//		  	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, saw);
//		  	  if(saw >= 2000){
//		  		  flag_saw = 0;
//		  	  }
//		  } else if(flag_saw == 0){
//			  saw = 0;
//		  	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, saw);
//		  	  flag_saw = 1;
//		  }
//		  //Dummy Delay
//		  ITM_Port32(31) = 1;
//		  for(int i = 0; i <= 400; i++){
//
//		  }
//		  ITM_Port32(31) = 2;

		  //sine
//		  float sample = 0;
//		  for(int j = 0; j < 20; j++){
//			  sine = 2000*(1+arm_sin_f32(2*pi*sample));
//		  	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, sine);
//		  	  sample += 0.05;
//		  	  //HAL_Delay(1);
//			  //Dummy Delay
//			  for(int i = 0; i <= 400; i++){
//
//			  }
//		  }
//	  }
//////////////////////////////////////////////////////////////////////////////////////////////
	  //Part 3
//	  for(int i = 0; i<=10000; i++){
//
//	  }
//	  float VREF;
//	  int TS_data;
//	  int VREFINT = *VREFINT_CAL_ADDR ;
//	  float VMeasured;
//	  int TS_Measured;
//	  int TS_CAL1 = *TEMPSENSOR_CAL1_ADDR;
//	  int TS_CAL2 = *TEMPSENSOR_CAL2_ADDR;
//
//	  // get VREF
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 200);
//	  VMeasured = HAL_ADC_GetValue(&hadc1);
//	  VREF = 3000.0 *(float)VREFINT / VMeasured;
//
//	  //get TS_DATA
//	  HAL_ADC_Start(&hadc3);
//	  HAL_ADC_PollForConversion(&hadc3, 200);
//	  TS_Measured = HAL_ADC_GetValue(&hadc3);
//	  TS_data = (TS_Measured * VREF)/3000;
//
//	  float diff = TS_data - TS_CAL1;
//	  float nume = 110 - 30;
//	  float denom = TS_CAL2 - TS_CAL1;
//
//	  temperatureC = (nume/denom)*diff + 30.0;
//
//	  HAL_ADC_Stop(&hadc1);
//	  HAL_ADC_Stop(&hadc3);
//////////////////////////////////////////////////////////////////////////////////////////////
//Part 4
//Default: Play a fixed sound: saw
	  /*First while: Default, play saw*/
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  pressCount == 0;
	  while(pressCount == 0){
		  playSaw();
		  if(HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) == 1){
			pressCount++;
			break;//break the first while loop, go to the second, in which the light is on.
		  }
	  }
	  /*First while end*/

	  /*Second while: LED2 is on, do temperature*/
	  while(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
		  temp = playTemperature();
		  if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 0){
			  break;
		  }
		  playSaw_Temperature(temp);
		  if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 0){
			  break;
		  }

	  }
	  /*Second while end*/

	  /*Third while: LED2 is off, play three sounds*/
	  while(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 0){
		  /*First sound loop begin: play triangle*/
		  playTriangle();
		  /*First sound loop end*/

		  if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
			  break; //break the third while loop, should go back to the second
		  }
		  /*Third while: could be ending here*/

		  /*Second sound loop begin: play sawtooth*/
		  playSaw_for();
		  /*Second sound loop end*/

		  if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
			  break; //break the third while loop, should go back to the second
		  }
		  /*Third while: could be ending here*/

		  /*Third sound loop begin: play sine*/
		  playSine();
		  /*Third sound loop end*/
		  if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
			  break; //break the third while loop, should go back to the second
		  }
		  /*Third while: could be ending here*/
	  }
	  /*Third while end*/
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	float playTemperature(){
		/*Detect begin*/
		while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
			if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				break;
			}
		}
		if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 0){
		    HAL_ADC_Stop(&hadc1);
		    HAL_ADC_Stop(&hadc3);
		    return 0.0;
		}
		/*Detect end*/

//		if(HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0&& HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
//		    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
//		    HAL_ADC_Stop(&hadc1);
//		    HAL_ADC_Stop(&hadc3);
//		    return 0.0;
//		}
		float VREF;
		int TS_data;
		int VREFINT = *VREFINT_CAL_ADDR ;
		float VMeasured;
		int TS_Measured;
		int TS_CAL1 = *TEMPSENSOR_CAL1_ADDR;
		int TS_CAL2 = *TEMPSENSOR_CAL2_ADDR;

		// get VREF
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 200);
		VMeasured = HAL_ADC_GetValue(&hadc1);
		VREF = 3000.0 *(float)VREFINT / VMeasured;

		//get TS_DATA
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, 200);
		TS_Measured = HAL_ADC_GetValue(&hadc3);
		TS_data = (TS_Measured * VREF)/3000;

		float diff = TS_data - TS_CAL1;
		float nume = 110 - 30;
		float denom = TS_CAL2 - TS_CAL1;

		temperatureC = (nume/denom)*diff + 30.0;

		return temperatureC;
	}

	void playTriangle(){
		flag_LED = 0;
		for(int i =0; i <= 10000; i++){
			/*Detect begin*/
			while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
					break;
				}
			}
			if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
				break; //break sound loop
			}
			/*Detect end*/
			if(flag_tri == 1){
				triangle += 200;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, triangle);
				if(triangle >= 2000){
					flag_tri = 0;
				}
			} else if(flag_tri == 0){
				triangle -= 200;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, triangle);
				if(triangle <= 0){
					flag_tri = 1;
				}
			}
			//Dummy Delay
			for(int i = 0; i <= 400; i++){
//				if(HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0){
//					flag_LED = 1;
//					break; //break sound loop
//				}
			}
		}

	}

	void playSaw(){
		flag_LED = 0;
		while(1){
			/*Detect begin*/
			while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
					break;
				}
			}
			if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin)== 1){
				break; //break sound loop
			}
			/*Detect end*/
			if(flag_saw == 1){
				saw += 100;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw);
				if(saw >= 2000){
					flag_saw = 0;
				}
			} else if(flag_saw == 0){
				saw = 0;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw);
				flag_saw = 1;
			}
			//Dummy Delay
			for(int i = 0; i <= 400; i++){
//				if(HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0){
//					flag_LED = 1;
//					break; //break sound loop
//				}
			}
		}

	}

	void playSaw_for(){
		flag_LED = 0;
		for(int i = 0; i <= 20000; i++){
			/*Detect begin*/
			while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
					break;
				}
			}
			if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) == 1){
				break; //break sound loop
			}
			/*Detect end*/
			if(flag_saw == 1){
				saw += 100;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw);
				if(saw >= 2000){
					flag_saw = 0;
				}
			} else if(flag_saw == 0){
				saw = 0;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw);
				flag_saw = 1;
			}
			//Dummy Delay
			for(int i = 0; i <= 400; i++){
//				if(HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0){
//					flag_LED = 1;
//					break; //break sound loop
//				}
			}
		}
	}

	void playSaw_Temperature(float temp){
		flag_LED = 0;
		for(int i = 0; i <= 20000; i++){
			/*Detect begin*/
			while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
					break;
				}
			}
			if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin == 1)){
				break; //break sound loop
			}
			/*Detect end*/
			if(flag_saw == 1){
				saw += 100;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw);
				if(saw >= 2000){
					flag_saw = 0;
				}
			} else if(flag_saw == 0){
				saw = 0;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw);
				flag_saw = 1;
			}
			//Dummy Delay
			for(int i = 0; i <= 12*temp; i++){
//				if(HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0){
//					flag_LED = 1;
//					break; //break sound loop
//				}
			}
		}
	}

	void playSine(){
		for(int i = 0; i <= 900; i++){
			/*Detect begin*/
			while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
					break;
				}
			}
			if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin == 1)){
				break; //break sound loop
			}
			/*Detect end*/
			for(int j = 0; j < 20; j++){
				/*Detect begin*/
				while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0)){
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
					if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1 )){
						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
						break;
					}
				}
				if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin == 1)){
					break; //break sound loop
				}
				/*Detect end*/
				sine = 2000*(1+arm_sin_f32(2*pi*sample));
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sine);
				sample += 0.05;
				//Dummy Delay
				for(int i = 0; i <= 320; i++){
//					if(flag_LED == 1){
//						break;
//					}
				}
			}
		}

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
