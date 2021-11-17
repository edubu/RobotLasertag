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

uint8_t imu_init(I2C_HandleTypeDef* i2c_handle);

void imu_read_accel(IMU_DATA* imu_data);

#endif /* INC_IMU_H_ */
