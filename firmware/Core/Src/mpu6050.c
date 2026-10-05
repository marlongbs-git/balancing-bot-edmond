/*
 * mpu6050.c
 *
 *  Created on: Sep 25, 2026
 *      Author: Marlon Buchanan
 */

#include <mpu6050.h>
#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//I2C Debugging
#define I2C_TIMEOUT_MS 10
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN  GPIO_PIN_6
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN  GPIO_PIN_7

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

//Creating character arrays to output to make sure everything is configured correctly
char msgDeviceConnected[64];
char msgGyroConnected[64];
char msgAccelerometerConnected[64];
char msgExitSleep[64];
char printstring[64];
char printstring2[64];
char printstring3[128];
char printstringArcTan[128];

uint32_t dt;
uint32_t getTickOld;


void MPU6050_Init() {
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1,
			(DEVICE_ADDRESS << 1) + 0, 1, 100);
	if (ret == HAL_OK) {
		sprintf(msgDeviceConnected, "The device is connected! \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgDeviceConnected,
				strlen(msgDeviceConnected), 100);
	} else {
		sprintf(msgDeviceConnected, "ERROR: The device is NOT connected! \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgDeviceConnected,
				strlen(msgDeviceConnected), 100);
	}

	uint8_t temp_data = FS_GYRO_500;
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_GYRO,
			1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
		sprintf(msgGyroConnected, "Configuring Gyroscope \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgGyroConnected,
				strlen(msgGyroConnected), 100);
	} else {
		sprintf(msgGyroConnected, "Failed to write to register 27 \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgGyroConnected,
				strlen(msgGyroConnected), 100);
	}

	uint8_t temp_data2 = FS_ACC_4G;
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_ACC,
			1, &temp_data2, 1, 100);
	if (ret == HAL_OK) {
		sprintf(msgAccelerometerConnected, "Configuring Accelerometer \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgAccelerometerConnected,
				strlen(msgAccelerometerConnected), 100);
	} else {
		sprintf(msgAccelerometerConnected,
				"Failed to write to register 28 \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgAccelerometerConnected,
				strlen(msgAccelerometerConnected), 100);
	}

	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, REG_USR_CTRL, 1,
			&temp_data, 1, 100);
	if (ret == HAL_OK) {
		sprintf(msgExitSleep, "Exiting Sleep Mode \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgExitSleep,
				strlen(msgExitSleep), 100);
	} else {
		sprintf(msgExitSleep, "Failed to exit from sleep mode \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msgExitSleep,
				strlen(msgExitSleep), 100);
	}

}


float* getAccelerometerOffsets() {
	//Calibrates both the accelerometer and gyroscope

	static float accelCalibrationValues[] = { 0.0f, 0.0f, 0.0f };

	//Buffers
	uint8_t dataacc_x[2];
	uint8_t dataacc_y[2];
	uint8_t dataacc_z[2];

	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;

	//Sensitivity Factor
	float SSFaccel = 8192;

	char testString[64]; //Test String
	char testStringComplete[128];

	//Change size depending on calibration and sampling time.
	static double calibrationAx[10];
	static double calibrationAy[10];
	static double calibrationAz[10];

	float accelxSum = 0;
	float accelxAvg;
	float accelySum = 0;
	float accelyAvg;
	float accelzSum = 0;
	float accelzAvg;

	//Takes 10 Samples
	uint8_t calibrationCount = sizeof(calibrationAx) / sizeof(calibrationAx[0]);


		sprintf(testString,
				"Preparing X-Axis Calibration (Lay Flat on Side) \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) testString, strlen(testString),
				100);

		for (int i = 5; i > 0; i--) {
			sprintf(testString, "Begin X Calibration in %d... \r\n", i);
			HAL_UART_Transmit(&huart2, (uint8_t*) testString,
					strlen(testString), 100);
			HAL_Delay(1000);
		}

		//Calibrate X-Direction
		for (int i = 0; i < calibrationCount; i++) {
			if (i == (calibrationCount - 1)) {

				accelxAvg = 1 - (accelxSum / (calibrationCount - 1));
				sprintf(testStringComplete,
						"X-Direction Calibration Complete | X: %.3f \r\n",
						accelxAvg);
				HAL_UART_Transmit(&huart2, (uint8_t*) testStringComplete,
						strlen(testStringComplete), 100);
				accelCalibrationValues[0] = accelxAvg;

			} else {

				HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1,
						REG_DATA_ACCELX, 1, dataacc_x, 2, HAL_MAX_DELAY);
				x_accel = (((int16_t) dataacc_x[0] << 8) + dataacc_x[1]);
				calibrationAx[i] = x_accel / SSFaccel;
				accelxSum = accelxSum + calibrationAx[i];

				sprintf(testString, "Calibrating Accelerometer X-Axis... \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) testString,
						strlen(testString), 100);
				HAL_Delay(100);
			}
		}

		sprintf(testString,
				"Preparing Y-Axis Calibration (Lay Flat on Back) \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) testString, strlen(testString),
				100);

		for (int i = 5; i > 0; i--) {
			sprintf(testString, "Begin Y Calibration in %d... \r\n", i);
			HAL_UART_Transmit(&huart2, (uint8_t*) testString,
					strlen(testString), 100);
			HAL_Delay(1000);
		}

		//Calibrate Y-Direction
		for (int i = 0; i < calibrationCount; i++) {
			if (i == (calibrationCount - 1)) {

				accelyAvg = 1 - (accelySum / (calibrationCount - 1)); //Sitting flat our g value should be 1
				sprintf(testStringComplete,
						"Y-Direction Calibration Complete | Y: %.3f \r\n",
						accelyAvg);
				HAL_UART_Transmit(&huart2, (uint8_t*) testStringComplete,
						strlen(testStringComplete), 100);
				accelCalibrationValues[1] = accelyAvg;

			} else {

				HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1,
						REG_DATA_ACCELY, 1, dataacc_y, 2, HAL_MAX_DELAY);
				y_accel = (((int16_t) dataacc_y[0] << 8) + dataacc_y[1]);
				calibrationAy[i] = y_accel / SSFaccel;
				accelySum = accelySum + calibrationAy[i];

				sprintf(testString, "Calibrating Accelerometer Y-Axis... \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) testString,
						strlen(testString), 100);
				HAL_Delay(100);
			}

		}

		sprintf(testString, "Preparing Z-Axis Calibration \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) testString, strlen(testString), 100);

		for (int i = 10; i > 0; i--) {
			sprintf(testString, "Begin Z Calibration in %d... \r\n", i);
			HAL_UART_Transmit(&huart2, (uint8_t*) testString, strlen(testString),
					100);
			HAL_Delay(1000);
		}

		//Calibrate Z-Direction
		for (int i = 0; i < calibrationCount; i++) {
			if (i == (calibrationCount - 1)) {

				accelzAvg = 1 - (accelzSum / (calibrationCount - 1)); //Sitting flat our g value should be 1
				sprintf(testStringComplete,
						"Z-Direction Calibration Complete | Z: %.3f \r\n",
						accelzAvg);
				HAL_UART_Transmit(&huart2, (uint8_t*) testStringComplete,
						strlen(testStringComplete), 100);
				accelCalibrationValues[2] = accelzAvg;

			} else {

				HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1,
						REG_DATA_ACCELZ, 1, dataacc_z, 2, HAL_MAX_DELAY);
				z_accel = (((int16_t) dataacc_z[0] << 8) + dataacc_z[1]);
				calibrationAz[i] = z_accel / SSFaccel;
				accelzSum = accelzSum + calibrationAz[i];

				sprintf(testString, "Calibrating Accelerometer Z-Axis... \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) testString,
						strlen(testString), 100);
				HAL_Delay(100);
			}
		}

		sprintf(testStringComplete,
				"Calibration Complete! Calibration Values | [0]: %.3f, [1]: %.3f, [2]: %.3f \r\n",
				accelCalibrationValues[0], accelCalibrationValues[1],
				accelCalibrationValues[2]);
		HAL_UART_Transmit(&huart2, (uint8_t*) testStringComplete,
				strlen(testStringComplete), 100);
		return accelCalibrationValues;
}

float *getGyroscopeOffsets(){

	static float gyroCalibrationValues[] = {0.0f, 0.0f, 0.0f};

	//Create array of two 8-bit unsigned integer buffers to hold gyroscope 8-bit value
	uint8_t datagyro_x[2];
	uint8_t datagyro_y[2];
	uint8_t datagyro_z[2];

	//Create a 16bit int to hold the cocatinated 8-bit integers
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

	//Create an array of doubles to hold calibration values during standstill
	static double calibrationGx[10],
				  calibrationGy[10],
				  calibrationGz[10];

	//Sensitivity Factor of Gyroscope
	float SSFgy = 65.5;

	//Create Float Variables to measured values for computing average and a sum
	float gyxAvg, gyyAvg, gyzAvg,
		  gyxSum = 0, gyySum = 0, gyzSum = 0;

	//Get the size of the array of calibration values
	uint8_t calibrationCount = sizeof(calibrationGx) / sizeof(calibrationGx[0]);

	//Create character array buffers to print to UART of 64
	char testString[64]; //Test String
	char testStringComplete[64];

	sprintf(testString,
			"Preparing Gyroscope X-Y-Z Calibration \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) testString, strlen(testString),
			100);

	for (int i = 5; i > 0; i--) {
		sprintf(testString, "Begin Calibration in %d... \r\n", i);
		HAL_UART_Transmit(&huart2, (uint8_t*) testString,
				strlen(testString), 100);
		HAL_Delay(1000);
	}

	//Create for loop to go through
	for (int i = 0; i < calibrationCount; i++){

		//When calibration is done print the values to the UART -> populate the calibration value float and print the values populated
		if (i == (calibrationCount - 1)){

			gyxAvg = gyxSum / (calibrationCount - 1);
			gyyAvg = gyySum / (calibrationCount - 1);
			gyzAvg = gyzSum / (calibrationCount - 1); //We really only care about this one

			sprintf(testStringComplete,
					"gyAvg Values | X: %.3f, Y: %.3f, Z: %.3f \r\n",
					gyxAvg, gyyAvg, gyzAvg);
			HAL_UART_Transmit(&huart2, (uint8_t*) testStringComplete,
					strlen(testStringComplete), 100);

			gyroCalibrationValues[0] = gyxAvg;
			gyroCalibrationValues[1] = gyyAvg;
			gyroCalibrationValues[2] = gyzAvg;
			sprintf(testStringComplete,
					"Calibration Values | [0]: %.3f, [1]: %.3f, [2]: %.3f \r\n",
					gyroCalibrationValues[0], gyroCalibrationValues[1], gyroCalibrationValues[2]);
			HAL_UART_Transmit(&huart2, (uint8_t*)testStringComplete,
					strlen(testStringComplete), 100);


		}else{

			HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROX,
					1, datagyro_x, 2, HAL_MAX_DELAY);
			x_gyro = (((int16_t) datagyro_x[0] << 8) + datagyro_x[1]);
			calibrationGx[i] = x_gyro / SSFgy;
			gyxSum = gyxSum + calibrationGx[i];

			HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROY,
					1, datagyro_y, 2, HAL_MAX_DELAY);
			y_gyro = (((int16_t) datagyro_y[0] << 8) + datagyro_y[1]);
			calibrationGy[i] = y_gyro / SSFgy;
			gyySum = gyySum + calibrationGy[i];

			HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROZ,
					1, datagyro_z, 2, HAL_MAX_DELAY);
			z_gyro = (((int16_t) datagyro_z[0] << 8) + datagyro_z[1]);
			calibrationGz[i] = z_gyro / SSFgy;
			gyzSum = gyzSum + calibrationGz[i];

			sprintf(testString, "Calibrating Gyroscope... \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) testString,
					strlen(testString), 100);
			HAL_Delay(100);


		}

	}

	return gyroCalibrationValues;
}

//Makes the MPU6050.c later more portable

float mpu6050_getAccelPitchAngle(float AxOffsetError, float AyOffsetError, float AzOffsetError){
		//@brief takes offset values to calculate accelerometer and gyro pitch, applys complementary filter estimate to output a estimated filter value
		//inputs: offset values in x,y,z direction for accelerometer and gyro
		//outputs: filtered pitch estimate

			uint8_t data_x[2];
			uint8_t data_y[2];
			uint8_t data_z[2];


			int16_t x_accel;
			int16_t y_accel;
			int16_t z_accel;


			double x_accelLSB;
			double y_accelLSB;
			double z_accelLSB;
			double SSF = 8192; //Sensitivity Scale Factor on Datasheet LSB/g

			//double anglePitch;
			static float lastGoodPitchFiltered = 0.0f;


			HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELX, 1,data_x, 2, I2C_TIMEOUT_MS);
			HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELY, 1,data_y, 2, I2C_TIMEOUT_MS);
			HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELZ, 1,data_z, 2, I2C_TIMEOUT_MS);


			//I2C Debugging
		    static uint16_t consecutiveFailures = 0;
			HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELX, 1, data_x, 2, I2C_TIMEOUT_MS);
			if (status != HAL_OK) {
			        uint32_t errorCode = HAL_I2C_GetError(&hi2c1);
			        sprintf(printstringArcTan, "I2C FAILED status=%d error=0x%lX \r\n", status, errorCode);
			        HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan), 100);

			        if (++consecutiveFailures > 5) {
			            I2C_BusRecovery();
			            consecutiveFailures = 0;
			        }
			        return lastGoodPitchFiltered;
			    }
			    consecutiveFailures = 0;

			x_accel = ((int16_t) data_x[0] << 8) + data_x[1];
			x_accelLSB = x_accel / SSF;
			x_accelLSB = x_accelLSB + AxOffsetError;
			y_accel = ((int16_t) data_y[0] << 8) + data_y[1];
			y_accelLSB = y_accel / SSF;
			y_accelLSB = y_accelLSB + AyOffsetError;
			z_accel = ((int16_t) data_z[0] << 8) + data_z[1];
			z_accelLSB = z_accel / SSF;
			z_accelLSB = z_accelLSB + AzOffsetError;

			//sprintf(printstringArcTan, "x_accelLSB: %.3f, y_accelLSB: %.3f, z_accelLSB %.3f \r\n", x_accelLSB, y_accelLSB, z_accelLSB);
			//HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan),100);


			//Applying Digital Filtering to measurements (User Defined)
			float alpha = 0.2;

			double x_accelLSBF;
			double y_accelLSBF;
			double z_accelLSBF;
			double anglePitchFiltered;

			//Storing LPF Outputs
			static float lpfStateXAccel = 0.0f;
			static float lpfStateYAccel = 0.0f;
			static float lpfStateZAccel = 0.0f;

			x_accelLSBF = digitalLowPassFilter(x_accelLSB, alpha, &lpfStateXAccel);
			y_accelLSBF = digitalLowPassFilter(y_accelLSB, alpha, &lpfStateYAccel);
			z_accelLSBF = digitalLowPassFilter(z_accelLSB, alpha, &lpfStateZAccel);

			//anglePitch = atan2(y_accelLSB, sqrt((x_accelLSB * x_accelLSB) + (z_accelLSB * z_accelLSB))) * (180 / M_PI);
			anglePitchFiltered = atan2(y_accelLSBF, sqrt((x_accelLSBF * x_accelLSBF) + (z_accelLSBF * z_accelLSBF))) * (180 / M_PI);
			lastGoodPitchFiltered = anglePitchFiltered;

			//Printing
			//sprintf(printstringArcTan, "anglePitch: %.3f, anglePitchFiltered: %.3f \r\n", anglePitch, anglePitchFiltered);
			//HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan),100);

			return lastGoodPitchFiltered;
}


float mpu6050_getGyroRate(float GxOffsetError, float GyOffsetError, float GzOffsetError){
		//@brief takes offset values to calculate accelerometer and gyro pitch, applys complementary filter estimate to output a estimated filter value
		//inputs: offset values in x,y,z direction for accelerometer and gyro
		//outputs: filtered pitch estimate

			//Gyroscope buffers
			uint8_t datagy_x[2];
			//uint8_t datagy_y[2];
			//uint8_t datagy_z[2];

			int16_t x_gyro;
			//int16_t y_gyro;
			//int16_t z_gyro;

			float x_gyroLSB;
			//float y_gyroLSB;
			//float z_gyroLSB;
			//static float x_gyroAng = 0;
			//static uint32_t getTickOld = 0;
			float SSFgy = 65.5;
		    static float lastGoodGyroLSB = 0.0f;

		    static float x_gyroLSBFiltered = 0.0f;
		    static float lpfStateXGyro = 0.0f;

		    //I2C Debugging
		    static uint16_t consecutiveFailures = 0;
			HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROX, 1, datagy_x, 2, I2C_TIMEOUT_MS);
			if (status != HAL_OK) {
			        uint32_t errorCode = HAL_I2C_GetError(&hi2c1);
			        sprintf(printstringArcTan, "I2C FAILED status=%d error=0x%lX \r\n", status, errorCode);
			        HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan), 100);

			        if (++consecutiveFailures > 5) {
			            I2C_BusRecovery();
			            consecutiveFailures = 0;
			        }
			        return lastGoodGyroLSB;
			    }
			    consecutiveFailures = 0;


			x_gyro = ((int16_t) datagy_x[0] << 8) + datagy_x[1];
			x_gyroLSB = x_gyro / SSFgy;
			x_gyroLSB = x_gyroLSB - (GxOffsetError);

			//sprintf(printstringArcTan, "x_gyroLSB: %.3f \r\n", x_gyroLSB);
			//HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan),100);

			//First-Order IIRC Filter8
			float alpha = 0.5;

			x_gyroLSBFiltered = digitalLowPassFilter(x_gyroLSB, alpha, &lpfStateXGyro);

			//uint32_t currentTick = HAL_GetTick();
			//float dt = (currentTick - getTickOld) / 1000.0f;
			//getTickOld = currentTick;

			lastGoodGyroLSB = x_gyroLSBFiltered;
			//x_gyroAng = x_gyroAng + x_gyroLSB * dt;
			//Printing to UART
			//sprintf(printstringArcTan, "x_gyroLSB: %.3f, x_gyroLSBFiltered: %.3f \r\n", x_gyroLSB, x_gyroLSBFiltered);
			//HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan),100);

			return lastGoodGyroLSB;
}


//Printouts over UART
void mpu6050_readAccel() {
	//
	//Buffers where we will store two bytes or two 8-bit integers between 0-255

	//Accelerometer Buffers
	uint8_t data_x[2];
	uint8_t data_y[2];
	uint8_t data_z[2];

	//16 bit that will store our full value but its signed so it can be numbers +/- 2^16
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;

	float x_accelLSB;
	float y_accelLSB;
	float z_accelLSB;
	float SSF = 8192; //Sensitivity Scale Factor on Datasheet LSB/g


	//Reading the data from the device
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELX, 1,
			data_x, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELY, 1,
			data_y, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACCELZ, 1,
			data_z, 2, HAL_MAX_DELAY);

	//Saving the data in a 16bit integer
	//We use bit shifting to represent the two captured register values, we will read from the buffer values
	//LSB Setting Configured to 4g LSB/g so we divide our values by SSF
	// Remember for interger division one must be a float to not truncate the value
	x_accel = ((int16_t) data_x[0] << 8) + data_x[1];
	x_accelLSB = x_accel / SSF;
	//x_accelLSB = x_accelLSB - xOffsetErrorAccel;
	y_accel = ((int16_t) data_y[0] << 8) + data_y[1];
	y_accelLSB = y_accel / SSF;
	//y_accelLSB = y_accelLSB - yOffsetErrorAccel;
	z_accel = ((int16_t) data_z[0] << 8) + data_z[1];
	z_accelLSB = z_accel / SSF;
	//z_accelLSB = z_accelLSB - zOffsetErrorAccel;

	sprintf(printstring,
			"Acceleration Values | X: %.3f, Y: %.3f, Z: %.3f \r \n", x_accelLSB,
			y_accelLSB, z_accelLSB);
	HAL_UART_Transmit(&huart2, (uint8_t*) printstring, strlen(printstring),
			100);

}

void mpu6050_readGyro() {

	//Gyroscope buffers
	uint8_t datagy_x[2];
	uint8_t datagy_y[2];
	uint8_t datagy_z[2];

	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

	float x_gyroLSB;
	float y_gyroLSB;
	float z_gyroLSB;
	float SSFgy = 65.5;

	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROX, 1,
			datagy_x, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROY, 1,
			datagy_y, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYROZ, 1,
			datagy_z, 2, HAL_MAX_DELAY);

	x_gyro = ((int16_t) datagy_x[0] << 8) + datagy_x[1];
	x_gyroLSB = x_gyro / SSFgy;
	y_gyro = ((int16_t) datagy_y[0] << 8) + datagy_y[1];
	y_gyroLSB = y_gyro / SSFgy;
	z_gyro = ((int16_t) datagy_z[0] << 8) + datagy_z[1];
	z_gyroLSB = z_gyro / SSFgy;

	sprintf(printstring2,
			"Gyroscope Values | X_gy: %.3f, Y_gy: %.3f, Z: %.3f \r \n",
			x_gyroLSB, y_gyroLSB, z_gyroLSB);
	HAL_UART_Transmit(&huart2, (uint8_t*) printstring2, strlen(printstring2),
			100);

	//Now we want gyro angles


}

//I2C Debugging
void I2C_BusRecovery(void)
{
    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = I2C_SCL_PIN;  HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = I2C_SDA_PIN;  HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);

    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);   HAL_Delay(1);
        if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET) break;
    }

    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);   HAL_Delay(1);

    MX_I2C1_Init();
}


//Filters
float digitalLowPassFilter(float input, float alpha, float *state) {
	//@brief Takes in an input variable and a pointer to a state, stores it and calculates the output filtered value
	//inputs:  input value, alpha value for adjusting filtering, and pointer to an axis state
	//outputs: a pointer to the axis state to prevent overwriting issues from previous call
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}

float digitalHighPassFilter(float input, float beta){

	 static float hpfOutput = 0;
	 static float inputPrevious = 0; //need to store the previous value

	 hpfOutput = (0.5f)*(2-beta)*(input - inputPrevious) + (1-beta)*hpfOutput;
	 inputPrevious = input;

	 return hpfOutput;
}

float compFilter(float anglePitch, float gyroAngularRate, float alpha) {
	//@brief takes offset values to calculate accelerometer and gyro pitch, applys complementary filter estimate to output a estimated filter value
	//inputs: offset values in x,y,z direction for accelerometer and gyro
	//outputs: filtered pitch estimate
		static float compFilterOutput = 0.0f;
		static float gyroPitchEstimate = 0.0f;
		static uint32_t getTickOld = 0;

		//Filter Parameters
		//float alpha = 0.95f; 	//alpha value to set filter

		uint32_t currentTick = HAL_GetTick();
		float dt = (currentTick - getTickOld) / 1000.0f;
		getTickOld = currentTick;
		gyroPitchEstimate = gyroPitchEstimate + gyroAngularRate * dt;

		compFilterOutput = alpha * (compFilterOutput + gyroAngularRate * dt) + (1.0f - alpha) * anglePitch;

		//sprintf(printstring3, "AccPitch: %.3f, GyPitchEst: %.3f, CompF: %.3f \r\n", anglePitch, gyroPitchEstimate, compFilterOutput);
		//HAL_UART_Transmit(&huart2, (uint8_t*) printstring3, strlen(printstring3), 100);

		return compFilterOutput;

}




