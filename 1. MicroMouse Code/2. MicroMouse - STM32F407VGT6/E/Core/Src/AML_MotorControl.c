#include "AML_MotorControl.h"

extern TIM_HandleTypeDef htim2;
short direction = 0;
uint32_t period = (16000000 / 5000) - 1;

void AML_MotorControl_Setup()
{
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    htim2.Instance->ARR = period;
}

void AML_MotorControl_SetLeftSpeed(int16_t PWMValue)
{
    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, direction);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, !direction);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWMValue);
        htim2.Instance->CCR1 = (period * PWMValue) / 100;
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, !direction);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, direction);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWMValue);
        htim2.Instance->CCR1 = (period * (-PWMValue)) / 100;
    }
    else if (!PWMValue)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        htim2.Instance->CCR1 = 0;
    }
}

void AML_MotorControl_SetRightSpeed(int16_t PWMValue)
{
    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, !direction);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMValue);
        htim2.Instance->CCR2 = (period * PWMValue) / 100;
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, !direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, direction);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMValue);
        htim2.Instance->CCR2 = (period * -(PWMValue)) / 100;
    }
    else if (!PWMValue)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        htim2.Instance->CCR2 = 0;
    }
}

void AML_MotorControl_SetDirection(short dir)
{
    direction = dir;
}

void AML_MotorControl_ToggleDirection()
{
    direction = ~direction;
}

