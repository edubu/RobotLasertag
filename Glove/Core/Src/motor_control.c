/*
 * motor_control.c
 *
 *  Created on: Nov 19, 2021
 *      Author: edubuq
 */

#include "motor_control.h"

static float params[12];

uint8_t omni_init(float l, float w, float radius){

	// Wheel 1 -- front left
	params[0] = (-l - w)/radius;
	params[1] = 1/radius;
	params[2] = -1/radius;

	// Wheel 2 -- front right
	params[3] = (l + w)/radius;
	params[4] = 1/radius;
	params[5] = 1/radius;

	// Wheel 3 -- back right
	params[6] = (l + w)/radius;
	params[7] = 1/radius;
	params[8] = -1/radius;

	// Wheel 4 -- back left
	params[9] = (-l - w)/radius;
	params[10] = 1/radius;
	params[11] = 1/radius;

	return 0;
}

motor_vels getMotorVels(robot_vels *desiredVels){
	motor_vels cmds;
	matrixMult(desiredVels, &cmds);
	return cmds;
}

void matrixMult(robot_vels *desiredVels, motor_vels *motor_cmds){
	float motor1;
	float motor2;
	float motor3;
	float motor4;

	motor1 = params[1] * desiredVels->x_vel + params[2] * desiredVels->y_vel;
	motor2 = params[4] * desiredVels->x_vel + params[5] * desiredVels->y_vel;
	motor3 = params[7] * desiredVels->x_vel + params[8] * desiredVels->y_vel;
	motor4 = params[10] * desiredVels->x_vel + params[11] * desiredVels->y_vel;


}


