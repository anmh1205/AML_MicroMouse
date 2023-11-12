#ifndef AML_REMOTE_H
#define AML_REMOTE_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_MotorControl.h"
#include "pid.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"


void AML_Remote_Setup();
void AML_Remote_Handle();
void AML_Remote_SendData(uint8_t *data, uint8_t size);

#endif // AML_REMOTE_H