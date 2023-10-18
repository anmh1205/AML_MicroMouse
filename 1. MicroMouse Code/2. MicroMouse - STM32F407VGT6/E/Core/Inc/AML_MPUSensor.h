#ifndef AML_MPUSensor_H
#define AML_MPUSensor_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "pid.h"


void AML_MPUSensor_Setup();
uint8_t AML_MPUSensor_ResetAngle();
double AML_MPUSensor_GetAngle();

#endif /* test_h */