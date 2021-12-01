/*
 * piezo.h
 *
 *  Created on: Nov 30, 2021
 *      Author: hmflynn
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

float noteArray[8] = {262, 294, 330, 349, 392, 440, 494, 523}; //C D E F G A B C
const int C_LOW = 0, D = 1, E = 2, F = 3, G = 4, A = 5, B = 6, C_HIGH = 7;
const int QUARTER = 3000000, HALF = 5000000, FULL = 10000000, BURST = 300000;

void piezo_init(TIM_HandleTypeDef* htim_in);

void changeFrequency(int note);

void delay(int len);

void rest(int len);

void playNote(int len, int note, int restLen);

void playDeathSound();

void playHitSound();

void playFireSound();

void playVictors();

#endif /* INC_IMU_H_ */
