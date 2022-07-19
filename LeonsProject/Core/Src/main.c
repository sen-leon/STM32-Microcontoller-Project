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
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

  typedef struct
  {
  float x;
  float y;
  float z;
  } DATA_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
  // FXAS21002 Addresses (Gyroscope)
  const uint16_t GYRO_DEVADDR = 0x21<<1;

  //Gyro Registers
  const uint8_t Gyro_addr_config = 0x13;
  uint16_t Gyro_MSB_X = 0x01;
  uint16_t Gyro_LSB_X = 0x02;
  uint16_t Gyro_MSB_Y = 0x03;
  uint16_t Gyro_LSB_Y = 0x04;
  uint16_t Gyro_MSB_Z = 0x05;
  uint16_t Gyro_LSB_Z = 0x06;
  uint16_t Gyro_WHO_AM_I = 0x0C;

  float Gyro_conv_factor = 0.0078125; //7.8125 mdps/LSB
  int Gyro_FSR = 250;

  // FXOS8700CQ I2C Address (Accelerometer)
  uint16_t MAGACC_DEVADDR = 0x1F<<1; //0x1E or 0x1D 0x1C 0x1F // with pins SA0=0, SA1=0
  const uint16_t MAGACC_WHO_AM_I = 0x0D;

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

  float Mag_conv_factor = 0.1; //0.1 μT/LSB.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */



float nullangle;
float currentangle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void initialize_Sensors()
{
	int error_count = 0;
	HAL_StatusTypeDef ret;
	uint8_t buffer;

	//INITIALIZE GYROSCOPE
	ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_DEVADDR, Gyro_WHO_AM_I, 1, &buffer, 1, 5); //This should return "0xd7" for the Gyroscope in the buffer[0]
	//This leads to a nominal sensitivity of 7.8125 mdps/LSB.
	error_count += (ret!=HAL_OK);
	uint16_t CTRL_REG0 = 0x0D;
	uint8_t BW = 0b00; //Bandwidth BW=4
	uint8_t FSR = 0b11; //Full Scale Range FSR= +-250mdps/LSB
	uint8_t CTRL_REG0_input = BW<<6 | FSR;
	//uint8_t CTRL_REG0_input = 0b01000011;

	//ret = initialize_Sensor(GYRO_DEVADDR, Gyro_WHO_AM_I, CTRL_REG0, CTRL_REG0_input, output);
	ret = HAL_I2C_Mem_Write(&hi2c1, GYRO_DEVADDR, CTRL_REG0, 1, &CTRL_REG0_input, 1, 5);
	error_count += (ret!=HAL_OK);

	uint16_t CTRL_REG1 = 0x13;
	uint8_t ODR = 0b111; //Output Data Rate ODR=12.5Hz
	volatile uint8_t ACTIVE = 0b1; //active=1
	volatile uint8_t READY = 0b0; //ready=X (0 or 1)
	volatile uint8_t CTRL_REG1_input = ODR<<2 | ACTIVE<<1 | READY;
	//uint8_t CTRL_REG1_input = 0b00011110;

	//ret = initialize_Sensor(GYRO_DEVADDR, Gyro_WHO_AM_I, CTRL_REG1, CTRL_REG1_input, output);
	ret = HAL_I2C_Mem_Write(&hi2c1, GYRO_DEVADDR, CTRL_REG1, 1, &CTRL_REG1_input, 1, 5);
	error_count += (ret!=HAL_OK);

	//INITIALIZE MAGNETOMETER
	ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_DEVADDR, MAGACC_WHO_AM_I, 1, &buffer, 1, 5); //This should return "0xc7" for the Magnetometer/Accelerometer in the buffer[0]
	error_count += (ret!=HAL_OK);
	//Output Data Rate ODR=12.5Hz; low_noise=1; active=1;
	uint16_t MA_CTRL_REG1 = 0x2A;
	uint8_t M_ODR = 0b111;
	uint8_t M_ACTIVE = 0b1;
	uint8_t M_READY = 0b0;
	uint8_t MA_CTRL_REG1_input = 0b10101101;
	//ret = initialize_Sensor(MAGACC_DEVADDR, MAGACC_WHO_AM_I, MA_CTRL_REG1, MA_CTRL_REG1_input, output);
	ret = HAL_I2C_Mem_Write(&hi2c1, MAGACC_DEVADDR, MA_CTRL_REG1, 1, &MA_CTRL_REG1_input, 1, 5);
	error_count += (ret!=HAL_OK);
	//This sensor has a nominal sensitivity of 0.1 μT/LSB.
	//Auto-Calibration: disabled; Oversample Ratio OSR=7; Only Magnetometer is active
	uint16_t M_CTRL_REG1 = 0x5B;
	uint8_t M_CTRL_REG1_input = 0b00011101;
	//ret = initialize_Sensor(MAGACC_DEVADDR, MAGACC_WHO_AM_I, M_CTRL_REG1, M_CTRL_REG1_input, output);
	ret = HAL_I2C_Mem_Write(&hi2c1, MAGACC_DEVADDR, M_CTRL_REG1, 1, &M_CTRL_REG1_input, 1, 5);
	error_count += (ret!=HAL_OK);
}
/*
HAL_StatusTypeDef initialize_Sensor(uint16_t Sensor_addr, uint16_t WHO_AM_I, uint16_t CTRL_REG, uint8_t CTRL_REG_input, uint8_t *Out)
{
	uint8_t buffer;

	HAL_StatusTypeDef ret;
	 //This should return "0xd7" for the Gyroscope and "0xc7" for the Magnetometer/Accelerometer in the buffer[0]
	ret = HAL_I2C_Mem_Read(&hi2c1, Sensor_addr, WHO_AM_I, 1, &buffer, 1, 5);
	Out=&buffer;
	//Write the desired values into the control Register
	ret = HAL_I2C_Mem_Write(&hi2c1, Sensor_addr, CTRL_REG, 1, &CTRL_REG_input, 1, 5);
	HAL_Delay(50);
	return ret;
}
*/

HAL_StatusTypeDef read_Sensor_data(int16_t *Data, uint16_t Sensor_address, uint16_t Start_register)
{
	HAL_StatusTypeDef ret;
	uint8_t buffer[6];
	ret=HAL_I2C_Mem_Read(&hi2c1, Sensor_address, Start_register, 1, buffer, 6, 5);
	//HAL_Delay(10);
		if (ret == HAL_ERROR){
			return ret;}
/*	Data[0] = ((uint16_t) buffer[1])<<8 | ((uint16_t) buffer[0]);
	Data[1] = ((uint16_t) buffer[3])<<8 | ((uint16_t) buffer[2]);
	Data[2] = ((uint16_t) buffer[5])<<8 | ((uint16_t) buffer[4]);*/

	Data[0] = ((int16_t) buffer[0])<<8 | ((int16_t) buffer[1]);
	Data[1] = ((int16_t) buffer[2])<<8 | ((int16_t) buffer[3]);
	Data[2] = ((int16_t) buffer[4])<<8 | ((int16_t) buffer[5]);
	return ret;
}

void convert_Sensor_Data(int16_t *rawData, DATA_TypeDef *Data, float conv_factor)
{
	Data->x = rawData[0]*conv_factor;
	Data->y = rawData[1]*conv_factor;
	Data->z = rawData[2]*conv_factor;
}

void LED_PWM (DATA_TypeDef *Gyro_Data){

	int min_PWM = 500;
	int max_PWM = 1000;
	int x = fabs(Gyro_Data->z)/256*(max_PWM-min_PWM);
	float threshold = 0.8;

	int green = 0;
	int blue = 0;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	if (Gyro_Data->z > threshold){ //Gyro_Data->z > 0 ==> Counterclockwise Rotation, LD4 (Blue LED should be on)
		blue = x;
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, blue);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, green);
	}
	else if (Gyro_Data->z < -threshold){ //Gyro_Data->z < 0 ==> Clockwise Rotation, LD3 (Blue LED should be on)
		green = x;
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, blue);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, green);
	}
	else{
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ // This interrupt handles the push of the blue button

	HAL_StatusTypeDef ret;
	static int16_t Mag_rawRefData[3];
	static DATA_TypeDef Mag_RefData;
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	//Set current Sensor values as Reference (Zero)
	ret=read_Sensor_data(Mag_rawRefData, MAGACC_DEVADDR, MAG_MSB_X);
	convert_Sensor_Data(Mag_rawRefData, &Mag_RefData, Mag_conv_factor);
	nullangle = atan2(Mag_RefData.y, Mag_RefData.x)*180/M_PI;

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //int perm_B1=1;
  //DATA_TypeDef Gyro_Data;
  int16_t Gyro_rawData[3];
  int16_t Mag_rawData[3];

  DATA_TypeDef Gyro_Data;
  DATA_TypeDef Mag_Data;
  HAL_StatusTypeDef ret;
  float debug_nullangle;
  float debug_currentangle;
  initialize_Sensors();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  ret=read_Sensor_data(Gyro_rawData, GYRO_DEVADDR, Gyro_MSB_X);
	  ret=read_Sensor_data(Mag_rawData, MAGACC_DEVADDR, MAG_MSB_X);
	  convert_Sensor_Data(Gyro_rawData, &Gyro_Data, Gyro_conv_factor);
	  convert_Sensor_Data(Mag_rawData, &Mag_Data, Mag_conv_factor*2);
	  currentangle = atan2(Mag_Data.y, Mag_Data.x)*180/M_PI;

	  debug_nullangle = nullangle;
	  debug_currentangle = currentangle;

	  LED_PWM(&Gyro_Data);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*
	    int x;
	    for(x=0; x<1000; x=x+1)
	    {
	      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, x);
	      HAL_Delay(1);
	    }
	    for(x=1000; x>0; x=x-1)
	    {
	      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, x);
	      HAL_Delay(1);
	    }
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
