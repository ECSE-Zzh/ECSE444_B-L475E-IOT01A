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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_qspi.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WRITE_READ_ADDR 0x00
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart1;

osThreadId chageModeTaskHandle;
osThreadId transmitDataTasHandle;
osThreadId readSensorTaskHandle;
/* USER CODE BEGIN PV */
	float temperature;
	float pressure;
	char buf[1000];
	int counter = 0;

	uint8_t flag = 0;
	int sensorMode = -1;

	int eraseState = 0;
	int eraseArrFlashState = 0;
	int isRead = 0;
	uint32_t Addr_Start = WRITE_READ_ADDR;
	int validNum = 0;

	int temperatureNum = 0;
	uint8_t temperatureArr[1];
	uint8_t temperatureArrFlash[50] = {0};
	float temperatureSum = 0;
	float temperatureMean = 0;
	float temperatureVarSum = 0;
	float temperatureVar = 0;

	int pressureNum = 0;
	uint16_t pressureArr[1];
	uint16_t pressureArrFlash[10] = {0};
	float pressureSum = 0;
	float pressureMean = 0;
	float pressureVarSum = 0;
	float pressureVar = 0;

	int accNum = 0;
	int16_t acceleration[3];
	int accelerationXYZ[3];
	int accelerationXYZFlash[15];
	//////////////acc_X///////////////
	int accelerationX[5];
	int x = 0;
	int x_i = 0;
	int accelerationX_Sum;
	int accelerationX_Mean;
	int accelerationX_VarSum;
	int accelerationX_Variance;
	//////////////acc_Y///////////////
	int accelerationY[5];
	int y = 0;
	int y_i = 1;
	int accelerationY_Sum;
	int accelerationY_Mean;
	int accelerationY_VarSum;
	int accelerationY_Variance;
	//////////////acc_Z///////////////
	int accelerationZ[5];
	int z = 0;
	int z_i = 2;
	int accelerationZ_Sum;
	int accelerationZ_Mean;
	int accelerationZ_VarSum;
	int accelerationZ_Variance;

	int magNum = 0;
	int16_t magnetic[3];
	int magneticXYZ[3];
	int magneticXYZFlash[15];
	//////////////acc_X///////////////
	int magneticX[5];
	int magneticX_Sum;
	int magneticX_Mean;
	int magneticX_VarSum;
	int magneticX_Variance;
	//////////////acc_Y///////////////
	int magneticY[5];
	int magneticY_Sum;
	int magneticY_Mean;
	int magneticY_VarSum;
	int magneticY_Variance;
	//////////////acc_Z///////////////
	int magneticZ[5];
	int magneticZ_Sum;
	int magneticZ_Mean;
	int magneticZ_VarSum;
	int magneticZ_Variance;
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_QUADSPI_Init(void);
void StartChangeModeTask(void const * argument);
void StartTransmitDataTask(void const * argument);
void StartReadSensorTask(void const * argument);

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
//
//---------------------------Part3_Test_Code-----------------------------------//
//	void *Addr_Start = WRITE_READ_ADDR;
//	void *Addr_Read = WRITE_READ_ADDR;
//
//	uint8_t example_arr[5] = {0,1,2,3,4};
//	uint8_t example_copy[5] = {0,0,0,0,0};
//
//	uint8_t example_arr2[3] = {5,6,7};
//	uint8_t example_copy2[8];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_PSENSOR_Init ();
  BSP_HSENSOR_Init ();
  BSP_TSENSOR_Init ();
  BSP_GYRO_Init ();
  BSP_MAGNETO_Init();
  BSP_ACCELERO_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
  BSP_QSPI_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of chageModeTask */
  osThreadDef(chageModeTask, StartChangeModeTask, osPriorityNormal, 0, 128);
  chageModeTaskHandle = osThreadCreate(osThread(chageModeTask), NULL);

  /* definition and creation of transmitDataTas */
  osThreadDef(transmitDataTas, StartTransmitDataTask, osPriorityNormal, 0, 256);
  transmitDataTasHandle = osThreadCreate(osThread(transmitDataTas), NULL);

  /* definition and creation of readSensorTask */
  osThreadDef(readSensorTask, StartReadSensorTask, osPriorityNormal, 0, 128);
  readSensorTaskHandle = osThreadCreate(osThread(readSensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

// ----------------------------------------------PART1_v1------------------------------------------------------- //
	  /*
	  BSP_GYRO_GetXYZ(angVelocity);
	  humidity = BSP_HSENSOR_ReadHumidity();
	  temperature = BSP_TSENSOR_ReadTemp();
	  pressure = BSP_PSENSOR_ReadPressure();
	  sprintf(buf,
			  "1.Angular Velocity: X:%.2f, Y:%.2f, Z:%.2f\r\n"
			  "2.Humidity:%.2f\r\n"
			  "3.Temperature: %.2f\r\n"
			  "4.Pressure:%.2f\r\n"
			  " \r\n",
			  angVelocity[0], angVelocity[1], angVelocity[2], humidity, temperature, pressure);
	  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
	  //read each sensor value (four of them) at 10 Hz.
	  HAL_Delay(100);
	   */
//
//	  counter = 0;
//	  while(1){
//		  BSP_GYRO_GetXYZ(angVelocity);
//		  humidity = BSP_HSENSOR_ReadHumidity();
//		  temperature = BSP_TSENSOR_ReadTemp();
//		  pressure = BSP_PSENSOR_ReadPressure();
//
//		  while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0) && counter%4 == 0){
//			  if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1)&& counter%4 == 0){
//				  sprintf(buf,
//						  "1.Angular Velocity: X:%.2f, Y:%.2f, Z:%.2f\r\n"
//						  " \r\n",
//						  angVelocity[0], angVelocity[1], angVelocity[2]);
//				  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
//				  counter++;
//				  break;
//			  }
//		  }
//
//		  while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0) && counter%4 == 1){
//			  if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1)&& counter%4 == 1){
//				  sprintf(buf,
//						  "2.Humidity:%.2f\r\n"
//						  " \r\n",
//						  humidity);
//				  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
//				  counter++;
//				  break;
//			  }
//		  }
//
//		  while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0) && counter%4 == 2){
//			  if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1)&& counter%4 == 2){
//				  sprintf(buf,
//						  "3.Temperature: %.2f\r\n"
//						  " \r\n",
//						  temperature);
//				  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
//				  counter++;
//				  break;
//			  }
//		  }
//
//		  while((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 0) && counter%4 == 3){
//			  if((HAL_GPIO_ReadPin (blueBtn_GPIO_Port, blueBtn_Pin) == 1)&& counter%4 == 3){
//				  sprintf(buf,
//						  "4.Pressure:%.2f\r\n"
//						  " \r\n",
//						  pressure);
//				  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
//				  counter++;
//				  break;
//			  }
//		  }
//		  //clear buf: make sure vlaues for angular velocity won't be printed when only other three values are expected
//		  memset(buf, '\0', sizeof(buf));
//	  }

// ----------------------------------------------PART1_v1------------------------------------------------------- //

// -----------------------------------------PART3-------------------------------------------------- //
	  /*
	  HAL_GPIO_WritePin(LEDError_GPIO_Port, LEDError_Pin, GPIO_PIN_SET);
	  if(BSP_QSPI_Erase_Block((uint32_t) WRITE_READ_ADDR) != QSPI_OK)
		  Error_Handler();
	  	  if(BSP_QSPI_Write(example_arr, (uint32_t) WRITE_READ_ADDR, sizeof(example_arr)) != QSPI_OK)
	  		  Error_Handler();
	  	  if(BSP_QSPI_Read(example_copy, (uint32_t) WRITE_READ_ADDR, sizeof(example_copy)) != QSPI_OK)
	  		  Error_Handler();

	  	  if(BSP_QSPI_Write(example_arr2, (uint32_t) WRITE_READ_ADDR + sizeof(example_arr), sizeof(example_arr2)) != QSPI_OK)
	  		  Error_Handler();
	  	  if(BSP_QSPI_Read(example_copy2, (uint32_t) WRITE_READ_ADDR, sizeof(example_copy2)) != QSPI_OK)
	  		  Error_Handler();
	   */
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDError_GPIO_Port, LEDError_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDError_Pin */
  GPIO_InitStruct.Pin = LEDError_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDError_GPIO_Port, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ----------------------------------------------PART1------------------------------------------------------- //
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	BSP_GYRO_GetXYZ(angVelocity);
	humidity = BSP_HSENSOR_ReadHumidity();
	temperature = BSP_TSENSOR_ReadTemp();
	pressure = BSP_PSENSOR_ReadPressure();

	if (GPIO_Pin == blueBtn_Pin){
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		counter++;
	}

	if(counter > 3){
		counter = 0;
	}

	if(counter%4 == 0){
		sprintf(buf,
				"1.Angular Velocity: X:%.2f, Y:%.2f, Z:%.2f\r\n"
				" \r\n",
				angVelocity[0], angVelocity[1], angVelocity[2]);
		HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
	}

	if(counter%4 == 1){
		sprintf(buf,
				"2.Humidity:%.2f\r\n"
				" \r\n",
				humidity);
		HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
	}

	if(counter%4 == 2){
		sprintf(buf,
				"3.Temperature: %.2f\r\n"
				" \r\n",
				temperature);
		HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
	}

	if(counter%4 == 3){
		sprintf(buf,
				"4.Pressure:%.2f\r\n"
				" \r\n",
				pressure);
		HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
	}

	//clear buf: make sure vlaues for angular velocity won't be printed when only other three values are expected
	memset(buf, '\0', sizeof(buf));
}
*/
// ----------------------------------------------PART1------------------------------------------------------- //

// ---------------------------------------------------PART2------------------------------------------------------- //
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == blueBtn_Pin){
		HAL_GPIO_WritePin(LEDError_GPIO_Port, LEDError_Pin, 1);
		flag = 1;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartChangeModeTask */
///**
//  * @brief  Function implementing the chageModeTask thread.
//  * @param  argument: Not used
//  * @retval None
//  */
/* USER CODE END Header_StartChangeModeTask */
void StartChangeModeTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
//	/* Infinite loop */
	for(;;)
	{
		osDelay(5000);
		if(flag == 1){
			flag = 0;
			sensorMode = counter % 5;
			eraseState = 0;
			eraseArrFlashState = 0;
			Addr_Start = 0x00;
			validNum = 0;
			isRead = 0;
			counter++;

			x_i = 0;
			x = 0;
			y_i = 1;
			y = 0;
			z_i = 2;
			z = 0;
			if(counter > 4){
				counter = 0;
			}
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTransmitDataTask */
///**
//* @brief Function implementing the transmitDataTas thread.
//* @param argument: Not used
//* @retval None
//*/
/* USER CODE END Header_StartTransmitDataTask */
void StartTransmitDataTask(void const * argument)
{
  /* USER CODE BEGIN StartTransmitDataTask */
	// UART status
	HAL_StatusTypeDef UART_status;
	HAL_UART_Init(&huart1);

	/* Infinite loop */
	for(;;)
	{
		osDelay(5000);

		/////////---------------Temperature------------------///////////
		if(sensorMode == 0){
			if(eraseState == 0){
				eraseState = 1;
				//Erase block
				if(BSP_QSPI_Erase_Block((uint32_t) WRITE_READ_ADDR) != QSPI_OK)
					Error_Handler();
			}

			if(eraseArrFlashState == 0){
				eraseArrFlashState = 1;
				for(int i = 0; i < sizeof(temperatureArrFlash); i++){
					temperatureArrFlash[i] = 0;
				}
			}

			temperatureArr[0] = temperature;

			//Write data
			if(BSP_QSPI_Write(temperatureArr, Addr_Start, sizeof(temperatureArr)) != QSPI_OK)
				Error_Handler();
			Addr_Start += sizeof(temperatureArr);

			//Read data (test: pass for now)
			if(BSP_QSPI_Read(temperatureArrFlash, (uint32_t) WRITE_READ_ADDR, sizeof(temperatureArrFlash)) != QSPI_OK){
				Error_Handler();
			} else {
				isRead++;
				if(isRead > 50){
					isRead = 50;
				}
			}

			//#Sample
			temperatureNum = isRead;

			//Calculate mean value: temperature
			temperatureSum = 0;
			for(int i = 0; i < isRead; i++){
				temperatureSum += temperatureArrFlash[i];
			}
			temperatureMean = temperatureSum / (float)isRead;

			//Calculate variance value: pressure
			temperatureVarSum = 0;
			for(int i = 0; i < isRead; i++){
				temperatureVarSum = (float)temperatureVarSum + (float)((temperatureArrFlash[i]-temperatureMean) * (temperatureArrFlash[i]-temperatureMean));
			}
			temperatureVar = (float)(temperatureVarSum / (float)isRead);
			sprintf(buf,
					"1.Temperature:%.2d\r\n"
					" \r\n",
					(int)temperature);
			HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
		}

		/////////---------------Pressure------------------///////////
		if(sensorMode == 1){
			if(eraseState == 0){
				eraseState = 1;
				//Erase block
				if(BSP_QSPI_Erase_Block((uint32_t) WRITE_READ_ADDR) != QSPI_OK)
					Error_Handler();
			}

			if(eraseArrFlashState == 0){
				eraseArrFlashState = 1;
				for(int i = 0; i < sizeof(pressureArrFlash); i++){
					pressureArrFlash[i] = 0;
				}
			}

			pressureArr[0] = pressure;

			//Write data
			if(BSP_QSPI_Write(pressureArr, Addr_Start, sizeof(pressureArr)) != QSPI_OK)
				Error_Handler();
			Addr_Start += sizeof(pressureArr);

			//Read data (test: pass for now)
			if(BSP_QSPI_Read(pressureArrFlash, (uint32_t) WRITE_READ_ADDR, sizeof(pressureArrFlash)) != QSPI_OK){
				Error_Handler();
			} else {
				isRead++;
				if(isRead > 10){
					isRead = 10;
				}
			}

			//#Sample
			pressureNum = isRead;

			//Calculate mean value: pressure
			pressureSum = 0;
			for(int i = 0; i < isRead; i++){
				pressureSum += pressureArrFlash[i];
			}
			pressureMean = pressureSum / (float)isRead;

			//Calculate variance value: pressure
			pressureVarSum = 0;
			for(int i = 0; i < isRead; i++){
				pressureVarSum = (float)pressureVarSum + (float)((pressureArrFlash[i]-pressureMean) * (pressureArrFlash[i]-pressureMean));
			}
			pressureVar = 100*(float)(pressureVarSum / (float)isRead);
			sprintf(buf,
					"2.Pressure:%.2d\r\n"
					" \r\n",
					(int)pressure);
			HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
		}

		/////////---------------Acceleration------------------///////////
		if(sensorMode == 2){
			if(eraseState == 0){
				eraseState = 1;
				//Erase block
				if(BSP_QSPI_Erase_Block((uint32_t) WRITE_READ_ADDR) != QSPI_OK)
					Error_Handler();
			}

			if(validNum > 5){

				for(int i = 0; i < 3; i++){
					accelerationXYZ[i] = acceleration[i];
				}

				//Write data
				if(BSP_QSPI_Write(accelerationXYZ, Addr_Start, sizeof(accelerationXYZ)) != QSPI_OK)
					Error_Handler();
				Addr_Start += sizeof(accelerationXYZ);

				if(eraseArrFlashState == 0){
					eraseArrFlashState = 1;
					for(int i = 0; i < sizeof(accelerationXYZFlash); i++){
						accelerationXYZFlash[i] = 0;
					}
				}

				//Read data (test: pass for now)
				if(BSP_QSPI_Read(accelerationXYZFlash, (uint32_t) WRITE_READ_ADDR, sizeof(accelerationXYZFlash)) != QSPI_OK){
					Error_Handler();
				} else {
					isRead += 3;
					if(isRead > 15){
						isRead = 15;
					}
				}

				//Split X, Y, Z:
				while(x_i < isRead){
					accelerationX[x] = accelerationXYZFlash[x_i];
					x++;
					x_i+=3;
				}
				while(y_i < isRead){
					accelerationY[y] = accelerationXYZFlash[y_i];
					y++;
					y_i+=3;
				}
				while(z_i < isRead){
					accelerationZ[z] = accelerationXYZFlash[z_i];
					z++;
					z_i+=3;
				}

				//#Sample
				accNum = z;

				//Mean X:
				accelerationX_Sum = 0;
				for(int i = 0; i < x; i++){
					accelerationX_Sum += accelerationX[i];
				}
				accelerationX_Mean = (float)accelerationX_Sum/(float)x;
				//Mean Y:
				accelerationY_Sum = 0;
				for(int i = 0; i < y; i++){
					accelerationY_Sum += accelerationY[i];
				}
				accelerationY_Mean = (float)accelerationY_Sum/(float)y;
				//Mean Z:
				accelerationZ_Sum = 0;
				for(int i = 0; i < z; i++){
					accelerationZ_Sum += accelerationZ[i];
				}
				accelerationZ_Mean = (float)accelerationZ_Sum/(float)z;

				//Variance X:
				accelerationX_VarSum = 0;
				for(int i = 0; i < x; i++){
					accelerationX_VarSum = (float)accelerationX_VarSum + (float)((accelerationX[i]-accelerationX_Mean) * (accelerationX[i]-accelerationX_Mean));
				}
				accelerationX_Variance = (float)(accelerationX_VarSum / (float)x);
				//Variance Y:
				accelerationY_VarSum = 0;
				for(int i = 0; i < y; i++){
					accelerationY_VarSum = (float)accelerationY_VarSum + (float)((accelerationY[i]-accelerationY_Mean) * (accelerationY[i]-accelerationY_Mean));
				}
				accelerationY_Variance = (float)(accelerationY_VarSum / (float)y);
				//Variance Z:
				accelerationZ_VarSum = 0;
				for(int i = 0; i < z; i++){
					accelerationZ_VarSum = (float)accelerationZ_VarSum + (float)((accelerationZ[i]-accelerationZ_Mean) * (accelerationZ[i]-accelerationZ_Mean));
				}
				accelerationZ_Variance = (float)(accelerationZ_VarSum / (float)z);

				sprintf(buf,
						"3.Acceleration: X:%.2d, Y:%.2d, Z:%.2d\r\n"
						" \r\n",
						(int)acceleration[0],(int)acceleration[1], (int)acceleration[2]);
				HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
			}

		}

		/////////---------------Magnetic------------------///////////
		if(sensorMode == 3){
			if(eraseState == 0){
				eraseState = 1;
				//Erase block
				if(BSP_QSPI_Erase_Block((uint32_t) WRITE_READ_ADDR) != QSPI_OK)
					Error_Handler();
			}

			for(int i = 0; i < 3; i++){
				magneticXYZ[i] = magnetic[i];
			}

			//Write data
			if(BSP_QSPI_Write(magneticXYZ, Addr_Start, sizeof(magneticXYZ)) != QSPI_OK)
				Error_Handler();
			Addr_Start += sizeof(magneticXYZ);

			if(eraseArrFlashState == 0){
				eraseArrFlashState = 1;
				for(int i = 0; i < sizeof(magneticXYZFlash); i++){
					magneticXYZFlash[i] = 0;
				}
			}

			//Read data (test: pass for now)
			if(BSP_QSPI_Read(magneticXYZFlash, (uint32_t) WRITE_READ_ADDR, sizeof(magneticXYZFlash)) != QSPI_OK){
				Error_Handler();
			} else {
				isRead += 3;
				if(isRead > 15){
					isRead = 15;
				}
			}

			//Split X, Y, Z:
			while(x_i < isRead){
				magneticX[x] = magneticXYZFlash[x_i];
				x++;
				x_i+=3;
			}
			while(y_i < isRead){
				magneticY[y] = magneticXYZFlash[y_i];
				y++;
				y_i+=3;
			}
			while(z_i < isRead){
				magneticZ[z] = magneticXYZFlash[z_i];
				z++;
				z_i+=3;
			}

			//#Sample
			magNum = z;

			//Mean X:
			magneticX_Sum = 0;
			for(int i = 0; i < x; i++){
				magneticX_Sum += magneticX[i];
			}
			magneticX_Mean = (float)magneticX_Sum/(float)x;
			//Mean Y:
			magneticY_Sum = 0;
			for(int i = 0; i < y; i++){
				magneticY_Sum += magneticY[i];
			}
			magneticY_Mean = (float)magneticY_Sum/(float)y;
			//Mean Z:
			magneticZ_Sum = 0;
			for(int i = 0; i < z; i++){
				magneticZ_Sum += magneticZ[i];
			}
			magneticZ_Mean = (float)magneticZ_Sum/(float)z;

			//Variance X:
			magneticX_VarSum = 0;
			for(int i = 0; i < x; i++){
				magneticX_VarSum = (float)magneticX_VarSum + (float)((magneticX[i]-magneticX_Mean) * (magneticX[i]-magneticX_Mean));
			}
			magneticX_Variance = (float)(magneticX_VarSum / (float)x);
			//Variance Y:
			magneticY_VarSum = 0;
			for(int i = 0; i < y; i++){
				magneticY_VarSum = (float)magneticY_VarSum + (float)((magneticY[i]-magneticY_Mean) * (magneticY[i]-magneticY_Mean));
			}
			magneticY_Variance = (float)(magneticY_VarSum / (float)y);
			//Variance Z:
			magneticZ_VarSum = 0;
			for(int i = 0; i < z; i++){
				magneticZ_VarSum = (float)magneticZ_VarSum + (float)((magneticZ[i]-magneticZ_Mean) * (magneticZ[i]-magneticZ_Mean));
			}
			magneticZ_Variance = (float)(magneticZ_VarSum / (float)z);
			sprintf(buf,
					"4.Magnetic: X: %.2d, Y: %.2d, Z: %.2d\r\n"
					" \r\n",
					(int)magnetic[0], (int)magnetic[1], (int)magnetic[2]);
			HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);

		}

		/////////---------------Report------------------///////////
		if(sensorMode == 4){
			sprintf(buf,
					"5.1 Temperature Report: Mean: %.2d, Variance: %.2d, Sample Num: %.2d\r\n"
					"5.2 Pressure Report: Mean: %.2d, Variance(enlarger 100 times): %.2d, Sample Num: %.2d\r\n"
					"5.3.0 Acc Sample Num: %.2d\r\n"
					"5.3.1 Acceleration Report_Mean: X: %.2d, Y: %.2d, Z: %.2d\r\n"
					"5.3.2 Acceleration Report_Variance: X: %.2d, Y: %.2d, Z: %.2d\r\n"
					"5.4.0 Mag Sample Num: %.2d\r\n"
					"5.4.1 Magnetic Report_Mean: X: %.2d, Y: %.2d, Z: %.2d\r\n"
					"5.4.2 Magnetic Report_Variance: X: %.2d, Y: %.2d, Z: %.2d\r\n"
					" \r\n",
					(int)temperatureMean, (int)temperatureVar,(int)temperatureNum,
					(int)pressureMean, (int)pressureVar,(int)pressureNum,
					(int)accNum,
					(int)accelerationX_Mean, (int)accelerationY_Mean, (int)accelerationZ_Mean,
					(int)accelerationX_Variance, (int)accelerationY_Variance, (int)accelerationZ_Variance,
					(int)magNum,
					(int)magneticX_Mean, (int)magneticY_Mean, (int)magneticZ_Mean,
					(int)magneticX_Variance, (int)magneticY_Variance, (int)magneticZ_Variance);
			HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
		}

		//clear buf: make sure vlaues for angular velocity won't be printed when only other three values are expected
		memset(buf, '\0', sizeof(buf));

	}
  /* USER CODE END StartTransmitDataTask */
}

/* USER CODE BEGIN Header_StartReadSensorTask */
///**
//* @brief Function implementing the readSensorTask thread.
//* @param argument: Not used
//* @retval None
//*/
/* USER CODE END Header_StartReadSensorTask */
void StartReadSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartReadSensorTask */
//  /* Infinite loop */
  for(;;)
  {
	  osDelay(5000);
	  //Collect data from sensors
	  BSP_ACCELERO_AccGetXYZ(acceleration);
	  BSP_MAGNETO_GetXYZ(magnetic);
	  temperature = BSP_TSENSOR_ReadTemp();
	  pressure = BSP_PSENSOR_ReadPressure();

	  validNum++;

  }
  /* USER CODE END StartReadSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(LEDError_GPIO_Port, LEDError_Pin, GPIO_PIN_RESET);
	__BKPT();

  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
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
