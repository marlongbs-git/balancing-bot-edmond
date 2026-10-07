# Digital Filtering used for Edmond

## EMA Filter (First-Order IIR Filter)

<p align = "center" >
<img width="795" height="267" alt="image" src="https://github.com/user-attachments/assets/2d574dfa-eebf-415d-9467-0b45868e66a1" />	
</p>

The first-order EMA filter acts as an exponential moving average that factors in both the current and previous sample and weighs them against one another using the filtering coefficient alpha that is set by the user. This digital filter is a good simple way to implement light filtering and only uses one delay element which is good for my embedded system.


```c
float digitalLowPassFilter(float input, float alpha, float *state) {
	//@brief Takes in an input variable and a pointer to a state, stores it and calculates the output filtered value
	//inputs:  input value, alpha value for adjusting filtering, and pointer to an axis state
	//outputs: a pointer to the axis state to prevent overwriting issues from previous call
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}
```

```c
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
```

The filter is applied on the raw accelerometer dataset and passed a coefficient of alpha = 0.2. We can compare the outputs using a scope. Where pink represents the unfiltered calculation of pitch angle and green is alpha = 0.2
<p align = "center" >
<img width="672" height="510" alt="image" src="https://github.com/user-attachments/assets/bc16d9aa-7393-4ad9-b4a9-570ad01623fb" />
</p>




## Complimentary Filter

<img width="786" height="311" alt="image" src="https://github.com/user-attachments/assets/e3958e32-ab5f-4481-894d-c7d763975e2e" />


```c
float compFilter(float anglePitch, float gyroAngularRate, float alpha) {
	//@brief takes offset values to calculate accelerometer and gyro pitch, applys complementary filter estimate to output an estimated filter value
	//inputs: pitch angle, gyro rate, and filtering coeff
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
```

## Drawbacks
The more filtering incorportated into the system adds delays in computation as it will stray further from the real data. This is dependent on the speed of the overall control loop and the communication or way data is transmitted/handled coming from the sensor. In my specific use case, this is fine as the system is still operational however, for more advanced systems filtering can be tuned for the specific application using various optimization techniques in MATLAB.

## Refrences

Phils Lab *The Simplest Digital Filter (STM32 Implementation) - Phil's Lab #92*
Phils Lab *Complementary Filter - Sensor Fusion #2 - Phil's Lab #34*
Phils Lab *IIR Filters - Theory and Implementation (STM32) - Phil's Lab #32*



