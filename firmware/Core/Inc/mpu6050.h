/*
 * mpu6050.h
 *
 *  Created on: Jul 28, 2026 for robot_newtimers
 *      Author: Marlon Buchanan
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

//Hexidecimal device address
#define DEVICE_ADDRESS 0x68

#define FS_GYRO_250 0
#define FS_GYRO_500 8 //
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8 //0b00001000 set equivalent in binary
#define FS_ACC_8G 9 //0b00001001
#define FS_ACC_16G 10

//These are our registers look at the Datasheet for the information
#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107

#define REG_DATA_ACCELX 59
#define REG_DATA_ACCELY 61
#define REG_DATA_ACCELZ 63

#define REG_DATA_GYROX 67
#define REG_DATA_GYROY 69
#define REG_DATA_GYROZ 71


void MPU6050_Init();
void mpu6050_readAccel();
void mpu6050_readGyro();
float *getGyroscopeOffsets();
float *getAccelerometerOffsets();
float mpu6050_getGyroRate(float GxOffsetError, float GyOffsetError, float GzOffsetError);
float mpu6050_getAccelPitchAngle(float AxOffsetError, float AyOffsetError, float AzOffsetError);
float digitalLowPassFilter(float input, float alpha, float *state);
float digitalHighPassFilter(float input, float beta);
float mpu6050_getAccelPitchAngleTesting(float AxOffsetError, float AyOffsetError, float AzOffsetError);
float mpu6050_getGyroRateTesting(float GxOffsetError, float GyOffsetError, float GzOffsetError);
float compFilter(float anglePitch, float gyroAngularRate, float alpha);

//I2C Debugging
void I2C_BusRecovery(void);

//OLD
/*
void mpu6050_calibrate_gyroscope();
void mpu6050_calibrate_accelerometer();
float *get_accelerometerOffsets();
void calculateAccPitchYaw(float AxOffsetError, float AyOffsetError, float AzOffsetError);
void calculateGyPitchYaw(float GxOffsetError, float GyOffsetError, float GzOffsetError);
void calculateAccPitchYawDigitalFilter(float AxOffsetError, float AyOffsetError, float AzOffsetError); //For testing
void calculateGyPitchYawDigitalFilter(float GxOffsetError, float GyOffsetError, float GzOffsetError); //For testing
void YawComparison(float AxOffsetError, float AyOffsetError, float AzOffsetError,float GxOffsetError, float GyOffsetError, float GzOffsetError);
float compFilter(float AxOffsetError, float AyOffsetError, float AzOffsetError,float GxOffsetError, float GyOffsetError, float GzOffsetError);
*/
#endif /* INC_MPU6050_H_ */
