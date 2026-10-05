/*
 * robot_state.h
 *
 *  Created on: Sep 18, 2026
 *      Author: dueli
 */
#include <stdbool.h>

#ifndef INC_ROBOT_STATE_H_
#define INC_ROBOT_STATE_H_

typedef struct {
	float x;
	float v;
	float theta;
	float omega;
	bool tipped;
} RobotState;

typedef struct {
	float k[4];
} ControlGains;

void robotState_Init(float AxOff, float AyOff, float AzOff, float GxOff, float GyOff, float GzOff);
void printRobotState(const RobotState *state);
void setControlGains(float kx, float kv, float ktheta, float komega);
float computeControlEffort(const RobotState *state);
float computeControlDuty(float u, float measuredVelocity);
RobotState getRobotState();

#endif /* INC_ROBOT_STATE_H_ */
