#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_Encoder.h"
// #include "AML_PID.h"

void AML_MotorControl_Setup();
void AML_MotorControl_SetLeftSpeed(int16_t PWMValue);
void AML_MotorControl_SetRightSpeed(int16_t PWMValue);
void AML_MotorControl_SetDirection(short dir);

#endif // AML_MOTORCONTROL_H