/*
 * hand_control.h
 *
 *  Created on: Nov 19, 2021
 *      Author: edubuq
 */

#ifndef INC_HAND_CONTROL_H_
#define INC_HAND_CONTROL_H_


#include "stm32f4xx_hal.h"

uint8_t getMagnitude(int16_t x, int16_t y, int16_t z);

float * getDirection(int16_t x, int16_t y);


#endif /* INC_HAND_CONTROL_H_ */
