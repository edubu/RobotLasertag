/*
 * motor_control.h
 *
 *  Created on: Nov 19, 2021
 *      Author: edubuq
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h"

typedef struct {
	int8_t motor1;
	int8_t motor2;
	int8_t motor3;
	int8_t motor4;
} motor_vels;

typedef struct {
	float x_vel;
	float y_vel;
	float angular_vel = 0;
} robot_vels;

uint8_t omni_init(float l, float w, float radius);

motor_vels getMotorVels(robot_vels *desiredVels);

void matrixMultiply(robot_vels *desiredVels, motor_vels *motor_cmds);

#endif /* SRC_MOTOR_CONTROL_H_ */
