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
    for (int i = 0; i < 5; i++)
    {
        if (GPIO_Pin == Button[i])
        {
            debug[8] = i;
        }
    }
}
