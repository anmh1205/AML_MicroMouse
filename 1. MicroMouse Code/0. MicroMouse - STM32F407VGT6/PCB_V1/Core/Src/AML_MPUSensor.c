#include "AML_MPUSensor.h"

uint8_t ResetCommand[] = {0xFF, 0xAA, 0x52};

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

uint8_t data[35];
uint8_t buffer = 119;
extern int16_t debug[100];
double Angle, PreviousAngle = 0, SaveAngle = 0;

uint8_t AML_MPUSensor_ResetAngle()
{
    SaveAngle = 0;
    return HAL_UART_Transmit(&huart3, ResetCommand, 3, 1000);
}

void AML_MPUSensor_Setup()
{
    AML_MPUSensor_ResetAngle();
    HAL_UART_Receive_DMA(&huart3, data, 33);
}

void handle()
{
    while (buffer != 85) // wait 0x55
    {
        HAL_UART_Receive(&huart3, &buffer, 1, 1000);
    }
    buffer = 100;
    HAL_UART_Receive_DMA(&huart3, data, 33);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    if (huart->Instance == USART3)
    {
        if (data[0] != 83)
        {
            handle();
            return;
        }

        PreviousAngle = Angle;
        Angle = (((data[6] << 8) | data[5]) / 32768.0) * 180;

        if (Angle != 0)
        {
            if (Angle - PreviousAngle > 350) // 0 -> 360 degree
            {
                SaveAngle = 360;
            }
            else if (Angle - PreviousAngle < -350) // 360 -> 0 degree
            {
                SaveAngle = 0;
            }
        }
        HAL_UART_Receive_DMA(&huart3, data, 33);
    }
    else if (huart->Instance == USART6)
    {
        AML_Remote_Handle();
    }
}



double AML_MPUSensor_GetAngle()
{

    return Angle - SaveAngle;
}
