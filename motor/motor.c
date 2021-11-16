/*
 * motor.c
 *
 *  Created on: Nov 15, 2021
 *      Author: hersh
 */
#include "motor.h"

void motor_init(MOTOR* motor) {
	set_tim_ccr(motor->htim, motor->tim_channel, 0);
	HAL_TIM_PWM_Start(motor->htim, motor->tim_channel);
}

void motor_forward(MOTOR* motor, uint8_t speed) {
	HAL_GPIO_WritePin(motor->a1_port, motor->a1_pin, 1);
	HAL_GPIO_WritePin(motor->a2_port, motor->a2_pin, 0);

	set_tim_ccr(motor->htim, motor->tim_channel, speed_to_pulse(speed));
}

void motor_backward(MOTOR* motor, uint8_t speed) {
	HAL_GPIO_WritePin(motor->a1_port, motor->a1_pin, 0);
	HAL_GPIO_WritePin(motor->a2_port, motor->a2_pin, 1);

	set_tim_ccr(motor->htim, motor->tim_channel, speed_to_pulse(speed));
}

void motor_stop(MOTOR* motor) {
	set_tim_ccr(motor->htim, motor->tim_channel, 0);
}

void motor_shutdown(MOTOR* motor) {
	HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
}

uint32_t speed_to_pulse(uint8_t speed) {
	if (speed < 1) {
		return 0;
	}

	//ranges from 20% - 50% duty cycle
	//when 1 <= speed <= 10
	int pulse = 560 * speed + 2800;
	pulse = (pulse < 3360) ? 3360 : pulse;
	pulse = (pulse > 8400) ? 8400 : pulse;

	return pulse;
}

void set_tim_ccr(TIM_HandleTypeDef* tim_handle, uint32_t channel, uint32_t pulse) {
	channel >>= 2;

	switch(channel) {
		case 0:
			tim_handle->Instance->CCR1 = pulse;
			break;
		case 1:
			tim_handle->Instance->CCR2 = pulse;
			break;
		case 2:
			tim_handle->Instance->CCR3 = pulse;
			break;
		case 3:
			tim_handle->Instance->CCR4 = pulse;
			break;
	}
}

