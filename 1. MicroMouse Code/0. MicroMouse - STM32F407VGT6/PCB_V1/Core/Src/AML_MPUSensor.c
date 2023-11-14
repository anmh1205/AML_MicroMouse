#include "AML_MPUSensor.h"

uint8_t ResetCommand[] = {0xFF, 0xAA, 0x52};

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

volatile uint8_t MPUData[36];
volatile uint8_t buffer = 119;
// extern int16_t debug[100];
volatile double Angle, PreviousAngle = 0, SaveAngle = 0;
volatile uint8_t error = 0;

void AML_MPUSensor_ResetAngle()
{
    SaveAngle = 0;
    // return HAL_UART_Transmit(&huart3, ResetCommand, 3, 10);
}

void AML_MPUSensor_Setup()
{
    AML_MPUSensor_ResetAngle();
    HAL_UART_Receive_DMA(&huart3, MPUData, 33);
}

void handle()
{
    while (buffer != 85) // wait 0x55
    {
        HAL_UART_Receive(&huart3, &buffer, 1, 500);
    }
    buffer = 100;
    HAL_UART_Receive_DMA(&huart3, MPUData, 33);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    if (huart->Instance == USART3)
    {
        if (MPUData[0] != 83)
        {
            handle();
            // return;
            error = 1;
        }

        if (!error)
        {
            PreviousAngle = Angle;
            Angle = (((MPUData[6] << 8) | MPUData[5]) / 32768.0) * 180;
            // memset(MPUData, 0, 33);

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
            HAL_UART_Receive_DMA(&huart3, MPUData, 33);
        }
        
        error = 0;
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