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
#define ACCEL_XG_OFFS_USRH 0x06

static I2C_HandleTypeDef* hi2c;
static float G = 9.81;
static uint32_t timeout = 100;

void I2C_ClearBusyFlagErratum(I2C_Module* i2c) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  i2c->instance.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;
  GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->instance.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&(i2c->instance));
}

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

void imu_calibrate(int16_t x_offset, int16_t y_offset, int16_t z_offset) {
	uint8_t buf[7];
	buf[0] = ACCEL_XG_OFFS_USRH;
	buf[1] = (x_offset >> 8) & 0xFF;
	buf[2] = x_offset & 0xFF;
	buf[3] = (y_offset >> 8) & 0xFF;
	buf[4] = y_offset & 0xFF;
	buf[5] = (z_offset >> 8) & 0xFF;
	buf[6] = z_offset & 0xFF;

	HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDRESS, buf, 7, timeout);
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
