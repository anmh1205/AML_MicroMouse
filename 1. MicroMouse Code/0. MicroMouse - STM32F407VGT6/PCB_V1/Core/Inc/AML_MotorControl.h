#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "AML_LaserSensor.h"
#include "AML_Encoder.h"
#include "AML_MPUSensor.h"
#include "AML_Parameter.h"
#include "pid.h"
#include <stdint.h>
#include <limits.h>
// #include "math.h"

void AML_MotorControl_Setup();

void AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd);

void AML_MotorControl_LeftPWM(int32_t PWMValue);
void AML_MotorControl_RightPWM(int32_t PWMValue);

void AML_MotorControl_SetDirection(GPIO_PinState dir);
void AML_MotorControl_SetMouseSpeed(int32_t speed);

void AML_MotorControl_SetLeftSpeed(double speed, GPIO_PinState direction);
void AML_MotorControl_SetRightSpeed(double speed, GPIO_PinState direction);
void AML_MotorControl_Stop();

void AML_MotorControl_SetLeftWallValue();
void AML_MotorControl_SetRightWallValue();

void AML_MotorControl_LeftWallFollow();
void AML_MotorControl_RightWallFollow();
void AML_MotorControl_GoStraight();
void AML_MotorControl_TurnOnWallFollow();
void AML_MotorControl_TurnOffWallFollow();

void AML_MotorControl_TurnLeft90();
void AML_MotorControl_TurnRight90();
void AML_MotorControl_TurnLeft180();
void AML_MotorControl_TurnRight180();

void AML_MotorControl_LeftStillTurn();
void AML_MotorControl_RightStillTurn();

#endif // AML_MOTORCONTROL_H