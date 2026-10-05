/*
 * fit4050encoder.h
 *
 *  Created on: Aug 25, 2026
 *      Author: dueli
 */

#ifndef INC_FIT0450ENCODER_H_
#define INC_FIT0450ENCODER_H_

void getEncoderCount();
void getMotorSpeed();
void updateEncoderState();
float getVelocityEstimate();
float getPositionEstimate();
//void setMotorSpeed(int16_t PWM);
void motorDeadBandTest();
long mapDeadband(long PWM);


#endif /* INC_FIT0450ENCODER_H_ */
