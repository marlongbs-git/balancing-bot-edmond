#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_


//Put functions here
void setPWMPitchControl(float filtValue);
int map(float pitchAngle);
int mapDutyCycle(float controlDutyCycle);
void setPWMControl(float controlDutyCycle);

#endif
