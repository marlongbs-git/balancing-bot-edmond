# Digital filters used were IIR Filters integrated into the STM32. 

--

## EMA Filter (First-Order IIR Filter)


<img width="795" height="267" alt="image" src="https://github.com/user-attachments/assets/2d574dfa-eebf-415d-9467-0b45868e66a1" />


```c
float digitalLowPassFilter(float input, float alpha, float *state) {
	//@brief Takes in an input variable and a pointer to a state, stores it and calculates the output filtered value
	//inputs:  input value, alpha value for adjusting filtering, and pointer to an axis state
	//outputs: a pointer to the axis state to prevent overwriting issues from previous call
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}
```
--

## Complimentary Filter

```c
float compFilter(float anglePitch, float gyroAngularRate, float alpha) {
	//@brief takes offset values to calculate accelerometer and gyro pitch, applys complementary filter estimate to output a estimated filter value
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
--

## Refrences

Phils Lab *The Simplest Digital Filter (STM32 Implementation) - Phil's Lab #92*
Phils Lab *Complementary Filter - Sensor Fusion #2 - Phil's Lab #34*



