/*
 * stepper.h
 *
 *  Created on: Dec 3, 2021
 *  Author: hanchi / hmflynn
 */

#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim;

#define stepsperrev 4096

void stepper_init(TIM_HandleTypeDef htim_in);

void delay (uint16_t us);

void stepper_set_rpm (int rpm);

void stepper_half_drive (int step);

void stepper_stop();
