/*
 * motors.h
 *
 *  Created on: Nov 15, 2021
 *      Author: hersh
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"

typedef struct {
	//pwm timer info
	TIM_HandleTypeDef* htim;
	uint32_t tim_channel;

	//direction control GPIO pins
	GPIO_TypeDef* a1_port;
	GPIO_TypeDef* a2_port;
	uint16_t a1_pin;
	uint16_t a2_pin;
} MOTOR;


//initialize motor (set duty cycle = 0, start pwm)
void motor_init(MOTOR* motor);

//move motor forward at desired speed
void motor_forward(MOTOR* motor, uint8_t speed);

//move motor backward at desired speed
void motor_backward(MOTOR* motor, uint8_t speed);

//stop motor
void motor_stop(MOTOR* motor);

//shutdown motor (stop pwm)
void motor_shutdown(MOTOR* motor);

//converts a speed integer to timer pulse value
uint32_t speed_to_pulse(uint8_t speed);

//set the ccr of a timer depending on it's channel
void set_tim_ccr(TIM_HandleTypeDef* tim_handle, uint32_t channel, uint32_t pulse);

#endif /* INC_MOTOR_H_ */
