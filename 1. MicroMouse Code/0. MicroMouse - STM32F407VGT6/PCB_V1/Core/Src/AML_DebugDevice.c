#include "AML_DebugDevice.h"

uint16_t Led[8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_10};

void AML_DebugDevice_TurnOnLED(COLOR color)
{
    HAL_GPIO_WritePin(LEDPORT, Led[color], GPIO_PIN_SET);
}

void AML_DebugDevice_TurnOffLED(COLOR color)
{
    HAL_GPIO_WritePin(LEDPORT, Led[color], GPIO_PIN_RESET);
}

void AML_DebugDevice_ToggleLED(COLOR color)
{
    HAL_GPIO_TogglePin(LEDPORT, Led[color]);
}

void AML_DebugDevice_SetLED(COLOR color, uint8_t state)
{
    HAL_GPIO_WritePin(LEDPORT, Led[color], state);
}

void AML_DebugDevice_SetAllLED(uint8_t state)
{
    HAL_GPIO_WritePin(LEDPORT, Led[0], state);
    HAL_GPIO_WritePin(LEDPORT, Led[1], state);
    HAL_GPIO_WritePin(LEDPORT, Led[2], state);
    HAL_GPIO_WritePin(LEDPORT, Led[3], state);
    HAL_GPIO_WritePin(LEDPORT, Led[4], state);
    HAL_GPIO_WritePin(LEDPORT, Led[5], state);
    HAL_GPIO_WritePin(LEDPORT, Led[6], state);
    HAL_GPIO_WritePin(LEDPORT, Led[7], state);
}


