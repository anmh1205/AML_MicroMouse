#ifndef AML_KEYBOARD_H
#define AML_KEYBOARD_H

#include "stm32f4xx_hal.h"
#include "main.h"

void AML_Keyboard_Setup(void);
uint8_t AML_Keyboard_GetKey(void);
// void AML_Keyboard_WaitStartKey(void);

#endif // AML_KEYBOARD_H