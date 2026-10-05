/*
 * control.c
 *
 *  Created on: Aug 25, 2026
 *      Author: dueli
 */

#include <mpu6050.h>
#include <main.h> //Including main will allow us to access all the functions used
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim3;

char stringBuffer[128];


int mapDutyCycle(float controlDutyCycle){
	//@brief Take in controlDutyCycle that corresponds with Torque and wheel velocity (-1.0f to 1.0f)
	//		 Translate that value to PWM signal from -65535 to +65535 where -1.0 = -65535 and +1.0 is 65535
	//
	//input: dutyCycle % between -1.0 and 1.0
	//output: PWM signal representative of PWM Required for a Given Torque

	//ARR Value
	const int32_t outPWMmax = 65535;

	//Clamping
	if (controlDutyCycle > 1.0f){
		controlDutyCycle = 1.0f;
	}
	if (controlDutyCycle < -1.0f){
		controlDutyCycle = -1.0f;
	}

	int32_t mappedValue = controlDutyCycle * (float)outPWMmax;
	//sprintf(stringBuffer, "PWM: %.3ld \r\n", mappedValue);
	//HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);

	return mappedValue;

}

void setPWMControl(float controlDutyCycle) {
	//@brief Controls PWM inputs to Motor Driver and Direction of Wheels
	// 		 Reads the magnitude of the Duty Cycle
	// 	     Sets GPIO for Wheel Direction
	//		 Sets the PWM Pin with a duty cycle from 0->100%
	//inputs: Duty Cycle from +/- 65535
	//outputs
    int PWMMagnitude;
    PWMMagnitude = abs(mapDutyCycle(controlDutyCycle));

    // Check PWM
    if (controlDutyCycle > 0){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);

        //Test Prints
        //sprintf(stringBuffer,"Forward Direction \r\n");
        //HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);

    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);

        //sprintf(stringBuffer,"BackwardDirection \r\n");
        //HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMMagnitude);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWMMagnitude);

    //sprintf(stringBuffer,"PWM Magnitude: %.d \r\n", PWMMagnitude);
    //HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);

}


/*
void setPWMPitchControl(float filtValue) {
    int PWMmagnitude;
    float PWM;

    PWMmagnitude = map(filtValue);
    PWM = PWMmagnitude;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM);

    // Check filtValue's sign, NOT the PWM magnitude
    if (filtValue > 0){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
    }
}

int map(float pitchAngle){
	//@brief Maps accelerometer pitch range to pwm signal
	//input: Calculated Pitch Angle Range: +/- 40 Degrees otherwise clamped
	//output: PWM signal representative of pitch angle


	const float pitchMin = 0;
	const float pitchMax = 40.0f;
	const uint16_t outPWMmin = 0;
	const uint16_t outPWMmax = 65535;
	static uint16_t mappedValue = 0;
	//static float calculatedDuty = 0;

	//Clamping to maximum beyond 40 degrees
	if (pitchAngle > pitchMax || pitchAngle < -pitchMax){
		mappedValue = outPWMmax;
		//sprintf(stringBuffer, "PWM: %.3d \r\n", PWM);
		//sprintf(stringBuffer, "PWM: %.3f \r\n", calculatedDuty);
		//HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);
	}else if (pitchAngle > 0){
		mappedValue = (pitchAngle - pitchMin) * (outPWMmax - outPWMmin) / (pitchMax - pitchMin) + outPWMmin;
		//sprintf(stringBuffer, "PWM: %.3f \r\n", calculatedDuty);
		//HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);

	} else if (pitchAngle < 0){
		mappedValue = (-pitchAngle - pitchMin) * (outPWMmax - outPWMmin) / (pitchMax - pitchMin) + outPWMmin;
		//sprintf(stringBuffer, "PWM: %.3f \r\n", calculatedDuty);
		//HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);
	} else {
		mappedValue = 0;
		//sprintf(stringBuffer, "PWM: %.3d \r\n", PWM);
		//HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);
	}

	//calculatedDuty = ((float)mappedValue / (float)outPWMmax)*100.0f;

	return mappedValue;
	//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,PWM);
	//sprintf(stringBuffer, "PWM: %.3d \r\n", mappedValue);
	//HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, strlen(stringBuffer),100);

}

*/


