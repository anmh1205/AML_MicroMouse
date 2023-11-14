#include "AML_Encoder.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;


void AML_Encoder_Setup()
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // left encoder
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // right encoder
}

int16_t AML_Encoder_GetLeftValue()
{
    return __HAL_TIM_GET_COUNTER(&htim1);
}

void AML_Encoder_ResetLeftValue()
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

int16_t AML_Encoder_GetRightValue()
{
    return __HAL_TIM_GET_COUNTER(&htim4);
}

void AML_Encoder_ResetRightValue()
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}