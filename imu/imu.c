/*
 * imu.c
 *
 *  Created on: Nov 16, 2021
 *      Author: hersh
 */

#include "imu.h"

#define IMU_I2C_ADDRESS 0xD0 // 0x68 << 1
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define SMPRT_DIV 0x19
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

static I2C_HandleTypeDef* hi2c;
static float G = 9.81;
static uint32_t timeout = 100;

uint8_t imu_init(I2C_HandleTypeDef* i2c_handle) {
	hi2c = i2c_handle;

	uint8_t buf[6];
	HAL_StatusTypeDef ret;

	buf[0] = WHO_AM_I;
	ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDRESS, buf, 1, timeout);
	if (ret != HAL_OK) { return 1; }
	ret = HAL_I2C_Master_Receive(hi2c, IMU_I2C_ADDRESS, buf, 1, timeout);
	if (ret != HAL_OK) { return 1; }

	if (buf[0] == 0x68) {

		//wake device by writing 0 to PWR_MGMT_1
		buf[0] = PWR_MGMT_1;
		buf[1] = 0;
		ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDRESS, buf, 2, timeout);
		if (ret != HAL_OK) { return 1; }

		//divide sample rate by 8 to achieve 1khz data rate
		buf[0] = SMPRT_DIV;
		buf[1] = 7;
		ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDRESS, buf, 2, timeout);
		if (ret != HAL_OK) { return 1; }

		//config accelerometer
		buf[0] = ACCEL_CONFIG;
		buf[1] = 0;
		ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDRESS, buf, 2, timeout);
		if (ret != HAL_OK) { return 1; }
	} else {
		return 1;
	}

	return 0;
}

void imu_read_accel(IMU_DATA* imu_data) {

	uint8_t buf[6];

	//write subregister
	buf[0] = ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDRESS, buf, 1, timeout);

	//recieve accel data
	HAL_I2C_Master_Receive(hi2c, IMU_I2C_ADDRESS, buf, 6, timeout);

	//get raw data
	imu_data->ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
	imu_data->ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
	imu_data->az_raw = (int16_t)((buf[4] << 8) | buf[5]);

	//scale raw data
	//divide by 16384 to get in Gs
	//mult by G=9.81 to get m/s^2
	imu_data->ax = imu_data->ax_raw / 16384.0 * G;
	imu_data->ay = imu_data->ay_raw / 16384.0 * G;
	imu_data->az = imu_data->az_raw / 16384.0 * G;

}
