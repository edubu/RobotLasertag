/*
 * piezo.c
 *
 *  Created on: Nov 30, 2021
 *      Author: hmflynn
 */

#include "piezo.h"

static TIM_HandleTypeDef* htim;

void piezo_init(TIM_HandleTypeDef* htim_in) {
	htim = htim_in;
}

void changeFrequency(int note /* Hz */) {
	int arr = 1000000/noteArray[note];

	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	TIM_OC_InitTypeDef sConfigOC;
	htim->Init.Period = arr - 1;
	HAL_TIM_PWM_Init(htim);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = arr/2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void delay(int len) {
	int i;
	for (i = 0; i < len; ++i) {}
}

void rest(int len) {
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	delay(len);
}

void playNote(int len, int note, int restLen) {
	changeFrequency(note);
	delay(len);
	rest(restLen);
}

void playDeathSound() {
	playNote(HALF, D, BURST);
	playNote(QUARTER, D, BURST);
	playNote(FULL, G, BURST);
	playNote(HALF, D, BURST);
	playNote(QUARTER, G, BURST);
	playNote(FULL, B, BURST);
}

void playHitSound() {
	playNote(QUARTER, C_HIGH, 1);
	playNote(QUARTER, F, 1);
	playNote(QUARTER, C_HIGH, 1);
	playNote(QUARTER, F, 1);
	playNote(QUARTER, C_HIGH, 1);
	playNote(QUARTER, F, 1);
}

void playFireSound() {
	playNote(BURST, C_HIGH, 1);
	playNote(BURST, F, 1);
	playNote(BURST, C_LOW, 1);
}

void playVictors() {
	//Hail, to the victors valiant
	playNote(FULL, E, BURST);
	playNote(HALF, C_LOW, BURST);
	playNote(HALF, D, BURST);
	playNote(HALF, E, BURST);
	playNote(HALF, C_LOW, BURST);
	playNote(HALF, D, BURST);
	playNote(HALF, E, BURST);

	//Hail, to the conquering heroes
	playNote(FULL, F, BURST);
	playNote(HALF, D, BURST);
	playNote(HALF, E, BURST);
	playNote(HALF, F, BURST);
	playNote(HALF, D, BURST);
	playNote(HALF, E, BURST);
	playNote(HALF, F, BURST);

	//Hail, Hail to Michigan
	playNote(FULL, G, BURST);
	playNote(HALF, A, BURST);
	playNote(QUARTER, E, BURST);
	playNote(HALF, E, BURST);
	playNote(HALF, F, BURST);
	playNote(HALF, C_LOW, BURST);

	//The Champions of the West
	playNote(HALF, D, BURST);
	playNote(HALF, E, BURST);
	playNote(HALF, G, BURST);
	playNote(HALF, E, BURST);
	playNote(QUARTER, D, BURST);
	playNote(FULL, C_LOW, BURST);
}

