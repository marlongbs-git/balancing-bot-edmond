/*
 * robot_state.c
 *
 *  Created on: Sep 18, 2026
 *      Author: dueli
 */


#include <fit0450encoder.h>
#include "robot_state.h"
#include "mpu6050.h"
#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#define FALL_THRESHOLD_DEGREES 40.0f
#define RECOVERY_THRESHOLD_DEGREES 5.0f

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

char stringRobotBuffer[128];

//Calibration Offsets
static float AxOffsetError;
static float AyOffsetError;
static float AzOffsetError;
static float GxOffsetError;
static float GyOffsetError;
static float GzOffsetError;

//Estimation Values
static float compFilterOutput = 0.0f;
static float newGyroRate = 0.0f;
static uint32_t previousTick = 0;
//Complementary Filter Value (User Defined)
static float alpha = 0.95f; //Comp Filter Value

//System Parameters (User Defined)
static const float supplyVoltage = 7.1f; //Units: Volts
static const float wheelRadiusM = 0.065f; // Units: meters
static const float stallTorque = 0.07845f; //Units: Nm
static const float stallCurrent = 2.8f; //Units: A
static const float armResistance = supplyVoltage / stallCurrent; //Units: Ohm
static const float noLoadSpeed = 16.76f; //Units rad/s
static const float noLoadCurrent = 0.17f; //Units: A
static const float kt = stallTorque / stallCurrent; //Units: Nm/A
static const float kv = (supplyVoltage - noLoadCurrent*armResistance) / noLoadSpeed; //Units Vs/rad
static const float ktOverR = kt/armResistance;
static const float ktkvOverR = (kt*kv)/armResistance;


static ControlGains gains;
static void updateOrientationState(void);
static bool checkFallDetection(float thetaDeg);


RobotState getRobotState(){
	//@brief
	//inputs:
	//outputs:

	updateEncoderState(); 	  //Updates Absolute Position and Wheel Velocity
	updateOrientationState(); //Updates Pitch Angle and Angular Velocity

	RobotState state;
	state.x = getPositionEstimate(); //Units: m
	state.v = getVelocityEstimate(); //Units: m/s
	state.theta = compFilterOutput;  //Units: Degrees
	state.omega = newGyroRate;      //Units: Degrees/sec
	state.tipped = checkFallDetection(state.theta); //True = tipped
	return state;
}


void robotState_Init(float AxOff, float AyOff, float AzOff,
                      float GxOff, float GyOff, float GzOff){
	//@brief   Sets the Offset Errors for the Robot to get Accurate Readings
	//inputs:  Calibrated Accelerometer and Gyroscope Offsets
	//outputs: void
    AxOffsetError = AxOff;
    AyOffsetError = AyOff;
    AzOffsetError = AzOff;
    GxOffsetError = GxOff;
    GyOffsetError = GyOff;
    GzOffsetError = GzOff;
    previousTick = HAL_GetTick();
}

static void updateOrientationState(){
	//@brief Updates the pitch angle and angular velocity.
	// 		 Grabs information from the MPU6050 to calculate the pitch angle and rotation rate
	//		 Takes two time intervals to calculate the difference in time between samples
	//		 Uses time difference to integrate gyro to get an estimate of pitch angle
	//		 Calculates complimentary filter that combines measurement from accelerometer and gyro
	//inputs:  void
	//outputs: void
    float anglePitch = mpu6050_getAccelPitchAngle(AxOffsetError, AyOffsetError, AzOffsetError);
    float gyroRate = mpu6050_getGyroRate(GxOffsetError, GyOffsetError, GzOffsetError);

    //alpha = 0.95
    compFilterOutput = compFilter(anglePitch, gyroRate, alpha);
    newGyroRate = gyroRate;

    //sprintf(stringRobotBuffer, "compFilterOuput: %.3f, newPitchRate: %.3f \r\n", compFilterOutput, newPitchRate);
    //HAL_UART_Transmit(&huart2, (uint8_t*)stringRobotBuffer, strlen(stringRobotBuffer),100);
}

void printRobotState(const RobotState *state){
	//@brief Prints robot state values over UART
	//inputs: pointer to state structure (read-only)
	//outputs: void

	//Access state variables by dereferencing the pointer and accessing the member
	sprintf(stringRobotBuffer, "x: %.3f, v: %.3f, theta: %.3f, omega: %.3f \r\n",
			state->x, state->v, state->theta, state->omega);
	HAL_UART_Transmit(&huart2, (uint8_t*)stringRobotBuffer, strlen(stringRobotBuffer), 100);
}

void setControlGains(float kx, float kv, float ktheta, float komega){
	//@brief: Sets control gains given K-matrix from MATLAB LQR Optimization
	//inputs: Gain Matrix
	//outputs: void
	gains.k[0] = kx;
	gains.k[1] = kv;
	gains.k[2] = ktheta;
	gains.k[3] = komega;
}

float computeControlEffort(const RobotState *state){
	//@brief: Computes u (the dlqr gain matrix (K) multiplied by the current sensed values
	//inputs: state parameters
	//outputs: u
	//return -(gains.k[0]*state->x + gains.k[1]*state->v + gains.k[2]*state->theta + gains.k[3]*state->omega);
    const float DEG2RAD = (float)M_PI / 180.0f;
    float thetaRad = state->theta * DEG2RAD;
    float omegaRad = state->omega * DEG2RAD;

    return -(gains.k[0]*state->x + gains.k[1]*state->v
            + gains.k[2]*thetaRad + gains.k[3]*omegaRad);
}

float computeControlDuty(float u, float measuredVelocity){
	//@brief Computes appropriate duty cycle for specific torque value
	// 		 Uses linear relationship between Motor Torque, Motor Voltage and Wheel Velocity
	//inputs: control variable u (what we are manipulating using our LQR control law), linear wheel velocity
	//outputs: returns a duty cycle calculation (+ve/-ve) between -1 to +1 indicating direction

	float wheelOmega = measuredVelocity / wheelRadiusM;
	float desiredTorque = u * wheelRadiusM;
	float requiredVoltage = (desiredTorque + ktkvOverR*wheelOmega) / ktOverR;
	float dutyCycle = requiredVoltage / supplyVoltage;

	//
	if (dutyCycle > 1.0f){
		dutyCycle = 1.0f;
	}
	if (dutyCycle < -1.0f){
		dutyCycle = -1.0f;
	}

	return dutyCycle;


}

static bool checkFallDetection(float thetaDeg) {
	static bool tippedLatch = false;

	if(!tippedLatch && fabs(thetaDeg) > FALL_THRESHOLD_DEGREES){
		tippedLatch = true;
	}else if (tippedLatch && fabs(thetaDeg) < RECOVERY_THRESHOLD_DEGREES){
		tippedLatch = false;
	}
	return tippedLatch;
}







