#ifndef AML_ENCODER_H
#define AML_ENCODER_H

#include "stm32f4xx_hal.h"


void AML_Encoder_Setup(void);
int16_t AML_Encoder_GetLeftValue();
int16_t AML_Encoder_GetRightValue();
void AML_Encoder_ResetLeftValue();
void AML_Encoder_ResetRightValue();

#endif // AML_ENCODER_H