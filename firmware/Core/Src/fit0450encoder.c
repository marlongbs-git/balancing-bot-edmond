
//for robot_newtimers

#include <mpu6050.h>
#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;


//global variables
int16_t countsPerRound = 1920;
int8_t secPerMinute = 60;
char stringEncBuffer[128];

//Deadband parameters
int16_t inPWMmin = 0;
int16_t inPWMmax = 4096;
int16_t outPWMmin = 1352;
int16_t outPWMmax = 4096;

//static state variables for positon and motor measurements
static int32_t lastCount = 0;
static int64_t positionAccumulator = 0; //Keeps the absolute value of encoder counts
static uint32_t lastTick = 0;
static float measuredVelocity = 0.0f;
static float measuredPosition = 0.0f;
static const int encoderCountPerRev = 1920;
static const float wheelRadius = 65.0f; // Units: mm
static const float wheelCircumfrance = 2.0f*(float)M_PI*wheelRadius;
static const float linearDistanceConverter = wheelCircumfrance/1000.0f; //Units: m

void updateEncoderState(){
	//@brief: Samples encoder once for two count values between two timer intervals,
	//		  Computes a signed delta value to handle the 16-bit wrap around
	// 		  Keeps track of position accumulated and updates its position
	//		  Calculated velocity from elapsed time interval
	//		  Must be called before every Control Loop Iteration
	//inputs: void
	//outputs: void

	int32_t currentCount  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
	int32_t delta = currentCount - lastCount; //Difference between current and past encoder readings

	//Prevention of wrap around for values beyond timers 16bit register

	if (delta > 32768) {
		delta -= 65366;
	} else if (delta < -32768)  {
		delta += 65536;
	}
	lastCount = currentCount;
	positionAccumulator += delta; //keeps track of absolute position

	//Grabbing Time interval between encoder samples
	uint32_t currentTick = HAL_GetTick();
	float dt = (currentTick - lastTick) / 1000.0f;
	lastTick = currentTick;

	if (dt > 0.0f){
		float revsCurrentSample = (float)delta / encoderCountPerRev;
		measuredVelocity = (revsCurrentSample / dt) * linearDistanceConverter;
	} else {
		measuredVelocity = 0.0f;
	}

	measuredPosition = ((float)positionAccumulator / encoderCountPerRev) * linearDistanceConverter ; //measured position in m

}

float getVelocityEstimate(){
	//@brief: returns latest computed Velocity
	//inputs:  void
	//outputs: velocity in revolutions per second (signed value)
	return measuredVelocity;
}

float getPositionEstimate(){
	//@brief: returns latest absolute wheel position relative to starting position
	//inputs: void
	//outputs: position in revolutions (signed, absolute, and relative to powering on.
	return measuredPosition;
}

void getEncoderCount(){
	//@brief: gets encoder pulse reading raw values
	//inputs: void
	//outputs: void
	static int32_t encoderCount = 0;
	static int32_t pastencoderCount = 0;

	encoderCount = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

	if(encoderCount != pastencoderCount){
		sprintf(stringEncBuffer, "Encoder Count: %ld \r\n", encoderCount);
		HAL_UART_Transmit(&huart2,(uint8_t*)stringEncBuffer,strlen(stringEncBuffer),100);
		pastencoderCount = encoderCount;
	}
}

/*
void getMotorSpeed(){
	//@brief: counts 1920 ticks of encoder which is 2pi radians and gets two time measurements to get motor speed in rad/s
	//inputs: void
	//outputs: void
	//1920 counts per round

	static int32_t encoderCount = 0;
	static int32_t pastencoderCount = 0;
	static int32_t getTickOld = 0;
	float dt ;
	const int encoderCountPerRev = 1920; //1920 counts per rotation of shaft
	float motorSpeed;
	float revolutions;

	encoderCount = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

	if(encoderCount >= encoderCountPerRev){
		dt = (HAL_GetTick() - getTickOld)/1000.0f;
		getTickOld = HAL_GetTick();
		revolutions = (float)encoderCount / encoderCountPerRev;
		motorSpeed =  (revolutions / dt) * 60.0f; //testing revolutions per minute

		sprintf(stringEncBuffer, "motorSpeed (rev/min): %.3f \r\n", motorSpeed);
		HAL_UART_Transmit(&huart2, (uint8_t*)stringEncBuffer, strlen(stringEncBuffer),100);
		__HAL_TIM_SET_COUNTER(&htim2, 0); //reset the hardware timer to 0
		pastencoderCount = 0;

	}else if(encoderCount != pastencoderCount){
		//sprintf(stringBuffer, "Encoder Count: %ld \r\n", encoderCount);
		//HAL_UART_Transmit(&huart2,(uint8_t*)stringBuffer,strlen(stringBuffer),100);
		pastencoderCount = encoderCount;
		}
}
*/



/*
void setMotorSpeed(int16_t PWM){
	//@brief: sets motor speed via PWM
	//inptus: PWM between 0->4096
	//outputs: void

	//error checking
	if(PWM > 4096){
		sprintf(stringEncBuffer, "PWM Input out of Range");
		HAL_UART_Transmit(&huart2, (uint8_t*)stringEncBuffer, strlen(stringEncBuffer),100);
		PWM = 0;
	}else{
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
	}
}

void motorDeadBandTest(){
	//@brief: for finding deadband region
	//inputs: void
	//outputs: void

	for(int i = 0; i < 4096*0.5; i++){
		setMotorSpeed(i);
		HAL_Delay(50);
	}
}
*/

long mapDeadband(long PWM){
	//@brief Remapping the desired duty cycle so it is within the capabilities of the motor
	//input: input PWM value [0->4096] = [0->100%];
	//output: output PWM value [1352->4096] = [33->100%] Duty Cycle;
    if (PWM <= 0) {
        return 0; // truly no command -> motor off
    }
    return (PWM - inPWMmin) * (outPWMmax - outPWMmin) / (inPWMmax - inPWMmin) + outPWMmin;
}



