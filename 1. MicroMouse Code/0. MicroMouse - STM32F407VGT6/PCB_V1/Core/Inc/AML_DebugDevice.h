#ifndef AML_DEBUGDEVICE_H
#define AML_DEBUGDEVICE_H

#include "stm32f4xx_hal.h"
#include "main.h"

// Led number
#define LEDPORT GPIOC

typedef enum
{
    WHITE,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    ORANGE,
    PURPLE,
    PINK
} COLOR;

void AML_DebugDevice_TurnOnLED(COLOR color);
void AML_DebugDevice_TurnOffLED(COLOR color);
void AML_DebugDevice_ToggleLED(COLOR color);
void AML_DebugDevice_SetLED(COLOR color, uint8_t state);
void AML_DebugDevice_SetAllLED(uint8_t state);

#endif // AML_DEBUGDEVICE_H
