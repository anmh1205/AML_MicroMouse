#include "AML_Keyboard.h"

// extern ADC_HandleTypeDef hadc1;
extern int16_t debug[100];

typedef enum
{
    SW1,
    SW2,
    SW3,
    SW4,
    SW5
} SW;

uint16_t Button[5] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5};

uint8_t AML_Keyboard_GetKey(short key)
{
    return HAL_GPIO_ReadPin(ButtonPORT, Button[key]);
}

void AML_Keyboard_Setup()
{
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Button[0])
    {
        debug[8] = 1;
    }
    else if (GPIO_Pin == Button[1])
    {
        debug[8] = 2;
    }
    else if (GPIO_Pin == Button[2])
    {
        debug[8] = 3;
    }
    else if (GPIO_Pin == Button[3])
    {
        debug[8] = 4;
    }
    else if (GPIO_Pin == Button[4])
    {
        debug[8] = 5;
    }
}
