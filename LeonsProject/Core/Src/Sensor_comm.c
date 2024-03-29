/*
 * Sensor_comm.c
 *
 *  Created on: Jul 26, 2022
 *      Author: sengu
 */
#include "Sensor_comm.h"
#include "main.h"
#include "stm32f0xx_hal_conf.h"



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
	uint8_t CTRL_REG0_input = BW<<6 | FSR; //Assembly of register Input

	ret = HAL_I2C_Mem_Write(&hi2c1, GYRO_DEVADDR, CTRL_REG0, 1, &CTRL_REG0_input, 1, 5);
	error_count += (ret!=HAL_OK);

	uint16_t CTRL_REG1 = 0x13;
	uint8_t ODR = 0b100; //Output Data Rate ODR = 50Hz
	uint8_t ACTIVE = 0b1; //active=1
	uint8_t READY = 0b0; //ready=X (0 or 1)
	uint8_t CTRL_REG1_input = ODR<<2 | ACTIVE<<1 | READY; //Assembly of register Input

	//ret = initialize_Sensor(GYRO_DEVADDR, Gyro_WHO_AM_I, CTRL_REG1, CTRL_REG1_input, output);
	ret = HAL_I2C_Mem_Write(&hi2c1, GYRO_DEVADDR, CTRL_REG1, 1, &CTRL_REG1_input, 1, 5);
	error_count += (ret!=HAL_OK);

	//INITIALIZE MAGNETOMETER
	ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_DEVADDR, MAGACC_WHO_AM_I, 1, &buffer, 1, 5); //This should return "0xc7" for the Magnetometer/Accelerometer in the buffer[0]
	error_count += (ret!=HAL_OK);

	uint16_t MA_CTRL_REG1 = 0x2A;
	uint8_t M_ODR = 0b100; //Output Data Rate ODR = 50Hz
	uint8_t M_LNOISE = 0b1; //low_noise=1;
	uint8_t M_ACTIVE = 0b1; //active=1;
	uint8_t MA_CTRL_REG1_input = M_ODR<<3 | M_LNOISE<<2 | M_ACTIVE; //Assembly of register Input

	ret = HAL_I2C_Mem_Write(&hi2c1, MAGACC_DEVADDR, MA_CTRL_REG1, 1, &MA_CTRL_REG1_input, 1, 5);
	error_count += (ret!=HAL_OK);
	//This sensor has a nominal sensitivity of 0.1 μT/LSB.

	uint16_t M_CTRL_REG1 = 0x5B;

	uint8_t M_ACAL = 0b1; //Auto-Calibration: enabled
	uint8_t M_OS = 0b111; //Oversample Ratio OSR=7
	uint8_t M_HMS = 0b01; //Hybrid Mode: Only Magnetometer is active
	uint8_t M_CTRL_REG1_input = M_ACAL<<7 | M_OS<<2 | M_HMS; //Assembly of register Input
	//ret = initialize_Sensor(MAGACC_DEVADDR, MAGACC_WHO_AM_I, M_CTRL_REG1, M_CTRL_REG1_input, output);
	ret = HAL_I2C_Mem_Write(&hi2c1, MAGACC_DEVADDR, M_CTRL_REG1, 1, &M_CTRL_REG1_input, 1, 5);
	error_count += (ret!=HAL_OK);
}

HAL_StatusTypeDef read_Sensor_data(int16_t *Data, uint16_t Sensor_address, uint16_t Start_register)
{
	HAL_StatusTypeDef ret;
	uint8_t buffer[6];
	ret=HAL_I2C_Mem_Read(&hi2c1, Sensor_address, Start_register, 1, buffer, 6, 5);
	//HAL_Delay(10);
		if (ret == HAL_ERROR){
			return ret;
		}

//	Data[0] = (buffer[1]<<8 | buffer[0]);
//	Data[1] = (buffer[3]<<8 | buffer[2]);
//	Data[2] = (buffer[5]<<8 | buffer[4]);

	Data[0] = (buffer[0])<<8 | (buffer[1]);
	Data[1] = (buffer[2])<<8 | (buffer[3]);
	Data[2] = (buffer[4])<<8 | (buffer[5]);
	return ret;
}

void convert_Sensor_Data(int16_t *rawData, DATA_TypeDef *Data, float conv_factor)
{
	Data->x = rawData[0]*conv_factor;
	Data->y = rawData[1]*conv_factor;
	Data->z = rawData[2]*conv_factor;
}

