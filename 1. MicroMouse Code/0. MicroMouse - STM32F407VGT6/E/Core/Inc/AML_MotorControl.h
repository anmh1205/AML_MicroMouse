#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_Encoder.h"
#include "pid.h"
#include "math.h"


void AML_MotorControl_Setup(void);

void AML_MotorControl_LeftPWM(int16_t PWMValue);
void AML_MotorControl_RightPWM(int16_t PWMValue);

void AML_MotorControl_SetDirection(short dir);

void AML_MotorControl_SetLeftSpeed(float speed, short direction);
void AML_MotorControl_SetRightSpeed(float speed, short direction);

void AML_MotorControl_MoveLeft(float distance, short direction);
void AML_MotorControl_MoveRight(float distance, short direction);

#endif // AML_MOTORCONTROL_H