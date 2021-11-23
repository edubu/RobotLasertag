/*
 * hand_control.c
 *
 *  Created on: Nov 19, 2021
 *      Author: edubuq
 */

#include "hand_control.h"
#include "math.h"

uint8_t getMagnitude(int16_t x, int16_t y, int16_t z){
	// g_vals come in x, y, z
	uint16_t half_mag = 16384/2;

	uint16_t z_offset = 16384 - z;

	if(z_offset > half_mag){
		z_offset = half_mag;
	}

	//convert to range between 0 to 10
	uint8_t newValue = ((z_offset * 10)/half_mag);

	return newValue;
}

float * getDirection(int16_t x, int16_t y){
	static float direction[2];
	float magnitude = sqrt(pow(x, 2) + pow(y, 2));

	direction[0] = x/magnitude;
	direction[1] = y/magnitude;

	return direction;
}


