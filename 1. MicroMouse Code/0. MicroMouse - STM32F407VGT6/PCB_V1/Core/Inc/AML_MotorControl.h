#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_LaserSensor.h"
#include "AML_Encoder.h"
#include "pid.h"
#include "math.h"


void AML_MotorControl_Setup(void);


short AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd);



void AML_MotorControl_LeftPWM(int16_t PWMValue);
void AML_MotorControl_RightPWM(int16_t PWMValue);

void AML_MotorControl_SetDirection(short dir);

void AML_MotorControl_SetLeftSpeed(double speed, short direction);
void AML_MotorControl_SetRightSpeed(double speed, short direction);

void AML_MotorControl_MoveLeft(double distance, short direction);
void AML_MotorControl_MoveRight(double distance, short direction);


#endif // AML_MOTORCONTROL_H