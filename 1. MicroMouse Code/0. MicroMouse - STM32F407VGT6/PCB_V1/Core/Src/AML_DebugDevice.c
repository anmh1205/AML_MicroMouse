#include "AML_DebugDevice.h"

uint16_t Led[8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_10};

void AML_DebugDevice_BuzzerBeep(uint16_t delay)
{
    uint32_t InitTime = HAL_GetTick();

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    while (HAL_GetTick() - InitTime < (uint32_t)delay)
        ;

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void AML_DebugDevice_TurnOnLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOC, Led[color], GPIO_PIN_SET);
}

void AML_DebugDevice_TurnOffLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOC, Led[color], GPIO_PIN_RESET);
}

void AML_DebugDevice_ToggleLED(COLOR color)
{
    HAL_GPIO_TogglePin(GPIOC, Led[color]);
}

void AML_DebugDevice_SetLED(COLOR color, GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, Led[color], state);
}

void AML_DebugDevice_SetAllLED(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, Led[0], state);
    HAL_GPIO_WritePin(GPIOC, Led[1], state);
    HAL_GPIO_WritePin(GPIOC, Led[2], state);
    HAL_GPIO_WritePin(GPIOC, Led[3], state);
    HAL_GPIO_WritePin(GPIOC, Led[4], state);
    HAL_GPIO_WritePin(GPIOC, Led[5], state);
    HAL_GPIO_WritePin(GPIOC, Led[6], state);
    HAL_GPIO_WritePin(GPIOC, Led[7], state);
}


