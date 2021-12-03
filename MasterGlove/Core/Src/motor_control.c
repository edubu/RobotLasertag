/*
 * motor_control.c
 *
 *  Created on: Nov 19, 2021
 *      Author: edubuq
 */

#include "motor_control.h"
#include "math.h"

static float params[12];

uint8_t omni_init(){

	// Wheel 1 -- front left
	params[0] = 0;
	params[1] = -1;
	params[2] = 1;

	// Wheel 2 -- front right
	params[3] = 0;
	params[4] = 1;
	params[5] = 1;

	// Wheel 3 -- back right
	params[6] = 0;
	params[7] = -1;
	params[8] = 1;

	// Wheel 4 -- back left
	params[9] = 0;
	params[10] = 1;
	params[11] = 1;

	return 0;
}

int8_t * getMotorVels(uint8_t magnitude, float * direction){
	static int8_t motor_cmds[4];
	matrixMult(magnitude, direction, motor_cmds);
	return motor_cmds;
}

void matrixMult(uint8_t magnitude, float *direction, int8_t *motor_cmds){
	float motor1;
	float motor2;
	float motor3;
	float motor4;

	float xv = direction[0] * magnitude;
	int8_t x_vel = ceil(xv);


	float yv = direction[1] * magnitude;
	int8_t y_vel = ceil(yv);


	motor_cmds[0] = params[1] * x_vel + params[2] * y_vel;
	motor_cmds[1] = params[4] * x_vel + params[5] * y_vel;
	motor_cmds[2] = params[7] * x_vel + params[8] * y_vel;
	motor_cmds[3] = params[10] * x_vel + params[11] * y_vel;

}


