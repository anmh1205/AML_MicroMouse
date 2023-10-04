#ifndef AML_MotorControl_h
#define AML_MotorControl_h


#include <Arduino.h>
#include <Servo.h>


void AML_MotorControl_ESC(int speedL, int speedR);
void AML_MotorControl_PWM(int pwmL, int pwmR);
void AML_MotorControl_setupL298();


#endif