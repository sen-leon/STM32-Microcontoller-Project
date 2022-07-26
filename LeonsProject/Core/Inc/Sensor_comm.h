 /*
 * Sensor_comm.h
 *
 *  Created on: Jul 26, 2022
 *      Author: sengu
 */
#ifndef INC_SENSOR_COMM_H_
#define INC_SENSOR_COMM_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
  
  typedef struct
  {
  float x;
  float y;
  float z;
  } DATA_TypeDef;

  static const uint16_t GYRO_DEVADDR = 0x21<<1;

  //Gyro Registers
  static const uint8_t Gyro_addr_config = 0x13;
  static const uint16_t Gyro_MSB_X = 0x01;
  static const uint16_t Gyro_LSB_X = 0x02;
  static const uint16_t Gyro_MSB_Y = 0x03;
  static const uint16_t Gyro_LSB_Y = 0x04;
  static const uint16_t Gyro_MSB_Z = 0x05;
  static const uint16_t Gyro_LSB_Z = 0x06;
  static const uint16_t Gyro_WHO_AM_I = 0x0C;

  static const float Gyro_conv_factor = 0.0078125; //7.8125 mdps/LSB
  static const int Gyro_FSR = 250;

  // FXOS8700CQ I2C Address (Accelerometer)
  static const uint16_t MAGACC_DEVADDR = 0x1F<<1; //0x1E or 0x1D 0x1C 0x1F // with pins SA0=0, SA1=0
  static const uint16_t MAGACC_WHO_AM_I = 0x0D;

  //Accelerometer Registers
  static const uint16_t ACC_MSB_X = 0x01;
  static const uint16_t ACC_LSB_X = 0x02;
  static const uint16_t ACC_MSB_Y = 0x03;
  static const uint16_t ACC_LSB_Y = 0x04;
  static const uint16_t ACC_MSB_Z = 0x05;
  static const uint16_t ACC_LSB_Z = 0x06;

  //Magnetometer Registers
  static const uint16_t MAG_MSB_X = 0x33;
  static const uint16_t MAG_LSB_X = 0x34;
  static const uint16_t MAG_MSB_Y = 0x35;
  static const uint16_t MAG_LSB_Y = 0x36;
  static const uint16_t MAG_MSB_Z = 0x37;
  static const uint16_t MAG_LSB_Z = 0x38;

  static const float Mag_conv_factor = 0.1; //0.1 μT/LSB.
  
  
  void initialize_Sensors();
  HAL_StatusTypeDef read_Sensor_data(int16_t *Data, uint16_t Sensor_address, uint16_t Start_register);
  void convert_Sensor_Data(int16_t *rawData, DATA_TypeDef *Data, float conv_factor);
  
  #endif /*INC_SENSOR_COMM_H_*/