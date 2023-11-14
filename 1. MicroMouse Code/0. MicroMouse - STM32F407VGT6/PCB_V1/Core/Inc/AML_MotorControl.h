#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_LaserSensor.h"
#include "AML_Encoder.h"
#include "pid.h"
// #include "math.h"

void AML_MotorControl_Setup(void);

void AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd);

void AML_MotorControl_LeftPWM(int32_t PWMValue);
void AML_MotorControl_RightPWM(int32_t PWMValue);

void AML_MotorControl_SetDirection(GPIO_PinState dir);

void AML_MotorControl_SetLeftSpeed(double speed, GPIO_PinState direction);
void AML_MotorControl_SetRightSpeed(double speed, GPIO_PinState direction);
void AML_MotorControl_Stop();

void AML_MotorControl_SetLeftWallValue();
void AML_MotorControl_SetRightWallValue();

void AML_MotorControl_LeftWallFollow();
void AML_MotorControl_RightWallFollow();

#endif // AML_MOTORCONTROL_H