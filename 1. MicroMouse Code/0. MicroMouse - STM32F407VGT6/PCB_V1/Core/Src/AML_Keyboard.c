#include "AML_Keyboard.h"

typedef enum
{
    SW1,
    SW2,
    SW3,
    SW4,
    SW5
} SW;

volatile uint16_t Button[5] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5};

GPIO_PinState AML_Keyboard_GetKey(short key)
{
    return HAL_GPIO_ReadPin(ButtonPORT, Button[key]);
}

void AML_Keyboard_Setup()
{
}


