/*
 * joystick.h
 *
 *  Created on: Dec 3, 2021
 *      Author: hmflynn
 */

#include "stm32f4xx_hal.h"

static ADC_HandleTypeDef* hadc;

void joystick_init(ADC_HandleTypeDef* hadc_in);

uint8_t get_joystick_position();
