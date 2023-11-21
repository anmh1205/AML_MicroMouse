#include "AML_Remote.h"

extern UART_HandleTypeDef huart6;
// extern int16_t debug[100];

volatile uint8_t RemoteData[48];
volatile uint8_t RemoteBuffer = 119;
volatile uint8_t RemoteIndex = 0;
uint8_t ErrorString[] = "E";
uint8_t SuccessString[] = "S";

double ki, kp, kd;

void AML_Remote_Setup()
{
    HAL_UART_Receive_IT(&huart6, &RemoteBuffer, 1);
}

void AML_Remote_Handle()
{
    if (RemoteBuffer != 0)
    {
        RemoteData[RemoteIndex++] = RemoteBuffer;
    }
    else if (RemoteBuffer == 0)
    {

        sscanf((char *)RemoteData, "%lf %lf %lf\0", &kp, &ki, &kd);
        AML_MotorControl_PIDSetTunnings(kp, ki, kd);

        RemoteIndex = 0;
        memset(RemoteData, 0, sizeof(RemoteData));
    }
    HAL_UART_Receive_IT(&huart6, &RemoteBuffer, 1);
}

