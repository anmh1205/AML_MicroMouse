#include "AML_Keyboard.h"

extern ADC_HandleTypeDef hadc1;
extern int16_t debug[100];

typedef enum
{
    A,
    B,
    C,
    D,
    E,
    nothing
} ButtonName;

uint32_t ADCValue = 4100;
uint16_t CompareValue[] = {150, 450, 750, 1000, 3900, 4100};
//         button         A,   B,   C,    D,    E,  nothing

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == hadc1.Instance) // kiểm tra ADC nào gây ra ngắt
    {
        ADCValue = HAL_ADC_GetValue(&hadc1);
    }
}

uint8_t AML_Keyboard_GetKey()
{
    ADCValue = HAL_ADC_GetValue(&hadc1);
    for (int i = 0; i < 5; i++)
    {
        if (ADCValue < CompareValue[i])
            return i;
    }
}

void AML_Keyboard_WaitStartKey()
{
    while (1)
    {
        if (AML_Keyboard_GetKey() != 5)
            return;
    }
}

void    AML_Keyboard_Setup()
{
    HAL_ADC_Start_IT(&hadc1);
}
