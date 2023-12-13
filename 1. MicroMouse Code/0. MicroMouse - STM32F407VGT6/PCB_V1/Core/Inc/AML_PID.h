#ifndef AML_PID_H
#define AML_PID_H


#include "stm32f407xx.h"

typedef struct 
{
    double Kp;
    double Ki;
    double Kd;

    double tau;

    double limMin;
    double limMax;

    double linMinInt;
    double linMaxInt;

    uint32_t sampleTime;
    uint32_t lastTime;


    double integratol;
    double prevError;
    double differentiator;
    double prevMeasurement;

    double out;
} AML_PID_Struct;


void AML_PID_Init(AML_PID_Struct *pid, double ki, double kp, double kd, double tau, double sampleTime);
double AML_PID_Update(AML_PID_Struct *pid, double setpoint, double measurement);



#endif // AML_PID_H