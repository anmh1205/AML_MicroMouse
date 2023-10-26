#include "AML_MotorControl.h"

#define Pi 3.14159265359
#define WheelDiameter 48

extern TIM_HandleTypeDef htim2;
extern int32_t LeftEncoderValue, RightEncoderValue;
int32_t PreviousLeftEncoderValue = 0, PreviousRightEncoderValue = 0;

short direction = 0;
uint32_t period = (16000000 / 5000) - 1;

PID_TypeDef PID_Left;
PID_TypeDef PID_Right;
double Input_Left, Output_Left, Setpoint_Left;
double Input_Right, Output_Right, Setpoint_Right;

double kp = 1.5;
double ki = 0.8;
double kd = 0;

int64_t PreviousTime = 0;

void AML_MotorControl_PIDSetTunings(double Kp, double Ki, double Kd)
{
    PID_Left.Kp = Kp;
    PID_Left.Ki = Ki;
    PID_Left.Kd = Kd;

    PID_Right.Kp = Kp;
    PID_Right.Ki = Ki;
    PID_Right.Kd = Kd;
}

void AML_MotorControl_PIDSetSampleTime(uint32_t NewSampleTime)
{
    PID_Left.SampleTime = NewSampleTime;
    PID_Right.SampleTime = NewSampleTime;
}

void AML_MotorControl_PIDSetOutputLimits(double Min, double Max)
{
    PID_Left.OutMin = Min;
    PID_Left.OutMax = Max;

    PID_Right.OutMin = Min;
    PID_Right.OutMax = Max;
}

void AML_MotorControl_PIDSetMode(PIDMode_TypeDef Mode)
{
    PID_SetMode(&PID_Left, Mode);
    PID_SetMode(&PID_Right, Mode);
}

void AML_MotorControl_PIDSetup()
{

    PID_Init(&PID_Left);
    PID_Init(&PID_Right);

    PID(&PID_Left, &Input_Left, &Output_Left, &Setpoint_Left, kp, ki, kd, PID_Left.POn, PID_Left.ControllerDirection);
    PID(&PID_Right, &Input_Right, &Output_Right, &Setpoint_Right, kp, ki, kd, PID_Right.POn, PID_Right.ControllerDirection);

    AML_MotorControl_PIDSetTunings(kp, ki, kd);
    AML_MotorControl_PIDSetSampleTime(10);
    AML_MotorControl_PIDSetOutputLimits(0, 100);
    AML_MotorControl_PIDSetMode(_PID_MODE_AUTOMATIC);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AML_MotorControl_Setup()
{
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    htim2.Instance->ARR = period;

    AML_MotorControl_PIDSetup();
}

void AML_MotorControl_LeftPWM(int16_t PWMValue)
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

void AML_MotorControl_RightPWM(int16_t PWMValue)
{
    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, !direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, direction);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMValue);
        htim2.Instance->CCR2 = (period * PWMValue) / 100;
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, !direction);
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

void AML_MotorControl_SetLeftSpeed(float speed, short direction)
{
    // Setpoint_Left = speed / (Pi * WheelDiameter);  // rpm

    Setpoint_Left = speed;
    Input_Left = abs(AML_Encoder_GetLeftValue() / 96);

    PID_Compute(&PID_Left);
    AML_Encoder_ResetLeftValue();

    AML_MotorControl_LeftPWM(Output_Left * direction);

}

void AML_MotorControl_SetRightSpeed(float speed, short direction)
{


    Setpoint_Right = speed;
    Input_Right = abs(AML_Encoder_GetRightValue() / 96);

    PID_Compute(&PID_Right);
    AML_Encoder_ResetRightValue();

    AML_MotorControl_RightPWM(Output_Right * direction);
}