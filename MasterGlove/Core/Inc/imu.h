/*
 * imu.h
 *
 *  Created on: Nov 16, 2021
 *      Author: hersh
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

typedef struct {
	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;

	float ax;
	float ay;
	float az;
} IMU_DATA;

typedef struct {
  I2C_HandleTypeDef   instance;
  uint16_t            sdaPin;
  GPIO_TypeDef*       sdaPort;
  uint16_t            sclPin;
  GPIO_TypeDef*       sclPort;
} I2C_Module;

//HAL_Delay error fix
//run this before any imu stuff (including imu_init)
void I2C_ClearBusyFlagErratum(I2C_Module* i2c);

//return 1 if error
//0 if success
uint8_t imu_init(I2C_HandleTypeDef* i2c_handle);

void imu_calibrate(int16_t x_offset, int16_t y_offset, int16_t z_offset);

void imu_read_accel(IMU_DATA* imu_data);

#endif /* INC_IMU_H_ */
