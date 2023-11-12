#include "AML_MotorControl.h"

#define Pi 3.14159265359
#define WheelDiameter 48
#define Ratio 90 / 14
#define PulsePerRound 96

extern TIM_HandleTypeDef htim2;
extern int32_t LeftEncoderValue, RightEncoderValue;
int32_t PreviousLeftEncoderValue = 0, PreviousRightEncoderValue = 0;

short direction = 1;
uint32_t period = (16000000 / 5000) - 1;

double Input_Left, Output_Left, Setpoint_Left;
double Input_Right, Output_Right, Setpoint_Right;

PID_TypeDef PID_SpeedLeft;
double Speed_Kp_Left = 1.0;
double Speed_Ki_Left = 0.1;
double Speed_Kd_Left = 0.1;

PID_TypeDef PID_SpeedRight;
double Speed_Kp_Right = 1.0;
double Speed_Ki_Right = 0.0;
double Speed_Kd_Right = 0;

PID_TypeDef PID_PositionLeft;
double Position_Kp = 0.005;
double Position_Ki = 0.007;
double Position_Kd = 0.0;

PID_TypeDef PID_PositionRight;

PID_TypeDef PID_LeftWallFollow;
double LeftWallDistance, SetpointDistance;

PID_TypeDef PID_RightWallFollow;
double RightWallDistance, SetpointDistance;

double WallFollow_Kp = 0.005;
double WallFollow_Ki = 0.007;
double WallFollow_Kd = 0.0;

short OutputMin = 0;
short OutputMax = 70;

int64_t PreviousTime = 0;

double ABS(double value)
{
    return value > 0 ? value : -value;
}

// void AML_MotorControl_PIDSetTunings(double Kp, double Ki, double Kd)
// {
//     PID_SpeedLeft.Kp = Kp;
//     PID_SpeedLeft.Ki = Ki;
//     PID_SpeedLeft.Kd = Kd;

//     PID_SpeedRight.Kp = Kp;
//     PID_SpeedRight.Ki = Ki;
//     PID_SpeedRight.Kd = Kd;
// }

short AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd)
{
    PID_TypeDef *pid = &PID_LeftWallFollow;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    return 1;
}

void AML_MotorControl_PIDSetSampleTime(uint32_t NewSampleTime)
{
    PID_SpeedLeft.SampleTime = NewSampleTime;
    PID_SpeedRight.SampleTime = NewSampleTime;

    PID_PositionLeft.SampleTime = NewSampleTime;
    PID_PositionRight.SampleTime = NewSampleTime;
}

void AML_MotorControl_PIDSetOutputLimits(double Min, double Max)
{
    PID_SpeedLeft.OutMin = Min;
    PID_SpeedLeft.OutMax = Max;

    PID_SpeedRight.OutMin = Min;
    PID_SpeedRight.OutMax = Max;

    PID_PositionLeft.OutMin = Min;
    PID_PositionLeft.OutMax = Max;

    PID_PositionRight.OutMin = Min;
    PID_PositionRight.OutMax = Max;
}

void AML_MotorControl_PIDSetMode(PIDMode_TypeDef Mode)
{
    PID_SetMode(&PID_SpeedLeft, Mode);
    PID_SetMode(&PID_SpeedRight, Mode);

    PID_SetMode(&PID_PositionLeft, Mode);
    PID_SetMode(&PID_PositionRight, Mode);
}

void AML_MotorControl_PIDSetup()
{
    PID_Init(&PID_SpeedLeft);
    PID_Init(&PID_SpeedRight);

    PID_Init(&PID_PositionLeft);
    PID_Init(&PID_PositionRight);

    PID(&PID_SpeedLeft, &Input_Left, &Output_Left, &Setpoint_Left, Speed_Kp_Left, Speed_Ki_Left, Speed_Kd_Left, _PID_P_ON_E, PID_SpeedLeft.ControllerDirection);
    PID(&PID_SpeedRight, &Input_Right, &Output_Right, &Setpoint_Right, Speed_Kp_Right, Speed_Ki_Right, Speed_Kd_Right, _PID_P_ON_E, PID_SpeedRight.ControllerDirection);

    PID(&PID_PositionLeft, &Input_Left, &Output_Left, &Setpoint_Left, Position_Kp, Position_Ki, Position_Kd, _PID_P_ON_E, PID_PositionLeft.ControllerDirection);
    PID(&PID_PositionRight, &Input_Right, &Output_Right, &Setpoint_Right, Position_Kp, Position_Ki, Position_Kd, _PID_P_ON_E, PID_PositionRight.ControllerDirection);

    PID(&PID_LeftWallFollow, &LeftWallDistance, &Output_Left, &SetpointDistance, WallFollow_Kp, WallFollow_Ki, WallFollow_Kd, _PID_P_ON_E, PID_LeftWallFollow.ControllerDirection);
    PID(&PID_RightWallFollow, &RightWallDistance, &Output_Right, &SetpointDistance, WallFollow_Kp, WallFollow_Ki, WallFollow_Kd, _PID_P_ON_E, PID_RightWallFollow.ControllerDirection);
    // AML_MotorControl_PIDSetTunings();

    AML_MotorControl_PIDSetSampleTime(5);
    AML_MotorControl_PIDSetOutputLimits(OutputMin, OutputMax);
    AML_MotorControl_PIDSetMode(_PID_MODE_AUTOMATIC);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// init motor control
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

void AML_MotorControl_SetLeftSpeed(double speed, short direction)
{
    Setpoint_Left = speed * Ratio;
    Input_Left = ABS((double)AML_Encoder_GetLeftValue() / PulsePerRound);
    AML_Encoder_ResetLeftValue();

    PID_Compute(&PID_SpeedLeft);

    AML_MotorControl_LeftPWM(Output_Left * direction);
}

void AML_MotorControl_SetRightSpeed(double speed, short direction)
{
    Setpoint_Right = speed * Ratio;
    Input_Right = ABS((double)AML_Encoder_GetRightValue() / PulsePerRound);
    AML_Encoder_ResetRightValue();

    PID_Compute(&PID_SpeedRight);

    AML_MotorControl_RightPWM(Output_Right * direction);
}

void AML_MotorControl_MoveLeft(double distance, short direction)
{
    Setpoint_Left = distance;
    Input_Left = abs(AML_Encoder_GetLeftValue());

    PID_Compute(&PID_PositionLeft);

    AML_MotorControl_LeftPWM(Output_Left * direction);
}

void AML_MotorControl_MoveRight(double distance, short direction)
{
    Setpoint_Right = distance;
    Input_Right = abs(AML_Encoder_GetRightValue());

    PID_Compute(&PID_PositionRight);

    AML_MotorControl_RightPWM(Output_Right * direction);
}

void AML_MotorControl_LeftWallFollow()
{
    uint16_t LeftWallDistance = AML_LaserSensor_ReadSingle(BL);
    SetpointDistance = 20;

    PID_Compute(&PID_LeftWallFollow);
    


}