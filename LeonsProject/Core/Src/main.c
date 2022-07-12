/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  typedef struct
  {
  int16_t x;
  int16_t y;
  int16_t z;
  } MA_RAWDATA;

  typedef struct
  {
  int16_t x;
  int16_t y;
  int16_t z;
  } DATA_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
  // FXAS21002 Addresses (Gyroscope)
  const uint16_t GYRO_DEVADDR_R = 0x41; // or 0x43 //Read Address of Sensor FXAS21002
  const uint16_t GYRO_DEVADDR_W = 0x40; // or 0x42 //Write Address of Sensor FXAS21002

  //Gyro Registers
  const uint8_t Gyro_addr_config = 0x13;
  uint16_t Gyro_MSB_X = 0x01;
  uint16_t Gyro_LSB_X = 0x02;
  uint16_t Gyro_MSB_Y = 0x03;
  uint16_t Gyro_LSB_Y = 0x04;
  uint16_t Gyro_MSB_Z = 0x05;
  uint16_t Gyro_LSB_Z = 0x06;

  // FXOS8700CQ I2C Address (Accelerometer)
  const uint16_t MAGACC_DEVADDR = 0x1E; //or 0x1D 0x1C 0x1F // with pins SA0=0, SA1=0

  //Accelerometer Registers
  const uint16_t ACC_MSB_X = 0x01;
  const uint16_t ACC_LSB_X = 0x02;
  const uint16_t ACC_MSB_Y = 0x03;
  const uint16_t ACC_LSB_Y = 0x04;
  const uint16_t ACC_MSB_Z = 0x05;
  const uint16_t ACC_LSB_Z = 0x06;

  //Magnetometer Registers
  const uint16_t MAG_MSB_X = 0x33;
  const uint16_t MAG_LSB_X = 0x34;
  const uint16_t MAG_MSB_Y = 0x35;
  const uint16_t MAG_LSB_Y = 0x36;
  const uint16_t MAG_MSB_Z = 0x37;
  const uint16_t MAG_LSB_Z = 0x38;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef read_Gyro_data(DATA_TypeDef *Data)
{
	uint8_t rawData[2];

	HAL_StatusTypeDef ret;
	ret=HAL_OK;
	//ret=HAL_I2C_Mem_Write(&hi2c1, (uint8_t)(Sensor_adress), 0x13, 1, &config, 1, 1000);
	//if (ret!= HAL_OK){
	//	return ret;}

	//ret=HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) GYRO_DEVADDR_R, 1000, 5000);
	//HAL_Delay(10);
	ret=HAL_I2C_Mem_Read(&hi2c1, GYRO_DEVADDR_R, Gyro_MSB_X, 1, rawData, 2, 5000);
	HAL_Delay(10);
	ret=HAL_OK;
	ret=HAL_I2C_Mem_Read(&hi2c1, MAGACC_DEVADDR, MAG_MSB_X, 1, rawData, 2, 5000);
	HAL_Delay(10);
	ret=HAL_OK;
	ret=HAL_I2C_Master_Transmit(&hi2c1, GYRO_DEVADDR_W, &Gyro_MSB_X, 1, 5000);
	HAL_Delay(10);
	ret=HAL_OK;
	ret=HAL_I2C_Master_Receive(&hi2c1, GYRO_DEVADDR_R, rawData, 2, 5000);
	//ret=HAL_I2C_Mem_Read(&hi2c1, Sensor_adress, Gyro_MSB_X, 1, rawData, 2, 500);
	HAL_Delay(10);
		if (ret == HAL_ERROR){
			return ret;}
	Data->x = ((uint16_t) rawData[0])<<8 | ((uint16_t) rawData[1]);
/*
	ret=HAL_I2C_Mem_Read(&hi2c1, Sensor_adress, Gyro_MSB_Y, 1, rawData, 2, 500);
		if (ret == HAL_ERROR){
			return ret;}
	*/
	Data->y = ((uint16_t) rawData[0])<<8 | ((uint16_t) rawData[1]);

	/*
	ret=HAL_I2C_Mem_Read(&hi2c1, Sensor_adress, Gyro_MSB_Z, 1, rawData, 2, 500);
	if (ret!= HAL_OK){
		return ret;}
	GYRO_DATA->z = rawData;
	*/

	return HAL_OK;
}

HAL_StatusTypeDef read_Magn_Accel_data(uint8_t Sensor_adress, uint8_t config, float Data)
{
	uint8_t rawData[2];
	HAL_StatusTypeDef ret;
	ret=HAL_I2C_Mem_Read(&hi2c1, Sensor_adress, ACC_MSB_X, 1, rawData, 2, 50);
	if (ret!= HAL_OK){
		return ret;}

	return HAL_OK;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //int perm_B1=1;
  DATA_TypeDef Gyro_Data;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 0);
  //HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);

  /*
  HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout);
  HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, Timeout);
  HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
  HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
  */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(500);
	  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  read_Gyro_data(&Gyro_Data);
	  //read_Gyro_data();

	  /*
	  uint8_t config= 0b00100000;
	  read_Gyro_data(GYRO_DEVADDR_R, config, Gyro_Data);
	  read_Magn_Accel_data(MAGACC_DEVADDR, config, MA_Data);
	  */

	  //HAL_TIM_Base_Start(&htim1)

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

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
