/*
 * joystick.c
 *
 *  Created on: Dec 3, 2021
 *      Author: hmflynn
 */
#include "joystick.h"

// call before calling other joystick functions
void joystick_init(ADC_HandleTypeDef* hadc_in) {
	hadc = hadc_in;
}

// determine which position joystick is in (0-left, 1-middle, 2-right)
uint8_t get_joystick_position() {
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	data = HAL_ADC_GetValue(hadc);

	// Joystick neutral position
	if (data >= 1500 && data <= 3500) {
		return 1;
	}
	// Joystick left position
	else if (data > 3500) {
		return 0;
	}
	// Joystick right position
	else {
		return 2;
	}
}
