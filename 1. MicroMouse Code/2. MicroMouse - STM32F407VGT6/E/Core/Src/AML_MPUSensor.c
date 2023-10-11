#include "AML_MPUSensor.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
PID_TypeDef pid;

uint8_t data[33];
uint8_t buffer = 100;
extern int16_t debug[100];
double angle, out, SetPoint = 0, Kp = 1.5, Ki = 0, Kd = 0;


void AML_MPUSensor_Setup()
{
    HAL_UART_Receive_DMA(&huart1, data, 33);
}

void test()
{
    
}

void handle()
{
    while (buffer != 85) // wait 0x55
    {
        HAL_UART_Receive(&huart1, &buffer, 1, 1000);
    }
    buffer = 70;
    HAL_UART_Receive_DMA(&huart1, data, 33);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    if (huart->Instance == USART1)
    {
        if (data[0] != 83)
        {
            handle();
            return;
        }
        angle = (((data[6] << 8) | data[5]) / 32768.0) * 180;
        PID_Compute(&pid);
        HAL_UART_Receive_DMA(&huart1, data, 33);
    }
}