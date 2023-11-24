#include "AML_MotorControl.h"

#define Pi 3.14159265359 // Pi number
#define WheelDiameter 50 // mm
#define Ratio 90 / 14    // 90:14
#define PulsePerRound 96 // 96 pulse per round encoder
int32_t MouseSpeed = 20; // % PWM

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim9;
extern int16_t debug[100];
extern uint8_t ReadButton;
extern VL53L0X_RangingMeasurementData_t SensorValue[7];
// int32_t LeftValue, RightValue;
// int32_t PreviousLeftEncoderValue = 0, PreviousRightEncoderValue = 0;

GPIO_PinState direction = GPIO_PIN_SET;

double Input_Left, Output_Left, Setpoint_Left;
double Input_Right, Output_Right, Setpoint_Right;

PID_TypeDef PID_SpeedLeft;
double Speed_Kp_Left = 1.0f;
double Speed_Ki_Left = 0.1f;
double Speed_Kd_Left = 0.1f;

PID_TypeDef PID_SpeedRight;
double Speed_Kp_Right = 1.0f;
double Speed_Ki_Right = 0.0f;
double Speed_Kd_Right = 0.f;

PID_TypeDef PID_PositionLeft;
double Position_Kp = 0.005f;
double Position_Ki = 0.007f;
double Position_Kd = 0.0f;

PID_TypeDef PID_PositionRight;

double SetpointDistance;

///////////////////////////////////

PID_TypeDef PID_LeftWallFollow;
double LeftWallDistance;

PID_TypeDef PID_RightWallFollow;
double RightWallDistance;

double WallFollow_Kp = 0.025f;
double WallFollow_Ki = 0.0f;
double WallFollow_Kd = 0.02f;

//////////////////////////////////

double MinRotateSpeed = -25;
double MaxRotateSpeed = 25;

PID_TypeDef PID_TurnLeft;
double Input_TurnLeft, Output_TurnLeft, Setpoint_TurnLeft;
double TurnLeft_Kp = 0.42f;
double TurnLeft_Ki = 0.0f;
double TurnLeft_Kd = 0.01f;

PID_TypeDef PID_TurnRight;
double Input_TurnRight, Output_TurnRight, Setpoint_TurnRight;
double TurnRight_Kp = 0.42f;
double TurnRight_Ki = 0.0f;
double TurnRight_Kd = 0.01f;

uint16_t MinLeftWallDistance, MaxLeftWallDistance;
uint16_t MinRightWallDistance, MaxRightWallDistance;
uint16_t MinFrontWallDistance, MaxFrontWallDistance;

double OutputMin = 0;
double OutputMax = 70;

// int64_t PreviousTime = 0;

double ABS(double value)
{
    return value > 0 ? value : -value;
}

int32_t double_to_int32_t(double value)
{
    if (value < INT32_MIN)
    {
        return INT32_MIN;
    }
    else if (value > INT32_MAX)
    {
        return INT32_MAX;
    }
    else
    {
        return (int32_t)value;
    }
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

void AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd)
{
    PID_TypeDef *pid = &PID_TurnLeft;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void AML_MotorControl_PIDSetSampleTime(uint32_t NewSampleTime)
{
    PID_SpeedLeft.SampleTime = NewSampleTime;
    PID_SpeedRight.SampleTime = NewSampleTime;

    PID_PositionLeft.SampleTime = NewSampleTime;
    PID_PositionRight.SampleTime = NewSampleTime;

    PID_LeftWallFollow.SampleTime = NewSampleTime;
    PID_RightWallFollow.SampleTime = NewSampleTime;

    PID_TurnLeft.SampleTime = NewSampleTime;
    PID_TurnRight.SampleTime = NewSampleTime;
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

    PID_LeftWallFollow.OutMin = -15;
    PID_LeftWallFollow.OutMax = 15;

    PID_RightWallFollow.OutMin = -15;
    PID_RightWallFollow.OutMax = 15;

    PID_TurnLeft.OutMin = MinRotateSpeed;
    PID_TurnLeft.OutMax = MaxRotateSpeed;

    PID_TurnRight.OutMin = MinRotateSpeed;
    PID_TurnRight.OutMax = MaxRotateSpeed;
}

void AML_MotorControl_PIDSetMode(PIDMode_TypeDef Mode)
{
    PID_SetMode(&PID_SpeedLeft, Mode);
    PID_SetMode(&PID_SpeedRight, Mode);

    PID_SetMode(&PID_PositionLeft, Mode);
    PID_SetMode(&PID_PositionRight, Mode);

    PID_SetMode(&PID_LeftWallFollow, Mode);
    PID_SetMode(&PID_RightWallFollow, Mode);

    PID_SetMode(&PID_TurnLeft, Mode);
    PID_SetMode(&PID_TurnRight, Mode);
}

void AML_MotorControl_PIDSetup()
{
    PID_Init(&PID_SpeedLeft);
    PID_Init(&PID_SpeedRight);

    PID_Init(&PID_PositionLeft);
    PID_Init(&PID_PositionRight);

    PID_Init(&PID_LeftWallFollow);
    PID_Init(&PID_RightWallFollow);

    PID(&PID_SpeedLeft, &Input_Left, &Output_Left, &Setpoint_Left, Speed_Kp_Left, Speed_Ki_Left, Speed_Kd_Left, _PID_P_ON_E, PID_SpeedLeft.ControllerDirection);
    PID(&PID_SpeedRight, &Input_Right, &Output_Right, &Setpoint_Right, Speed_Kp_Right, Speed_Ki_Right, Speed_Kd_Right, _PID_P_ON_E, PID_SpeedRight.ControllerDirection);

    PID(&PID_PositionLeft, &Input_Left, &Output_Left, &Setpoint_Left, Position_Kp, Position_Ki, Position_Kd, _PID_P_ON_E, PID_PositionLeft.ControllerDirection);
    PID(&PID_PositionRight, &Input_Right, &Output_Right, &Setpoint_Right, Position_Kp, Position_Ki, Position_Kd, _PID_P_ON_E, PID_PositionRight.ControllerDirection);

    PID(&PID_LeftWallFollow, &LeftWallDistance, &Output_Left, &SetpointDistance, WallFollow_Kp, WallFollow_Ki, WallFollow_Kd, _PID_P_ON_E, PID_LeftWallFollow.ControllerDirection);
    PID(&PID_RightWallFollow, &RightWallDistance, &Output_Right, &SetpointDistance, WallFollow_Kp, WallFollow_Ki, WallFollow_Kd, _PID_P_ON_E, PID_RightWallFollow.ControllerDirection);

    PID(&PID_TurnLeft, &Input_TurnLeft, &Output_TurnLeft, &Setpoint_TurnLeft, TurnLeft_Kp, TurnLeft_Ki, TurnLeft_Kd, _PID_P_ON_E, PID_TurnLeft.ControllerDirection);
    PID(&PID_TurnRight, &Input_TurnRight, &Output_TurnRight, &Setpoint_TurnRight, TurnRight_Kp, TurnRight_Ki, TurnRight_Kd, _PID_P_ON_E, PID_TurnRight.ControllerDirection);
    // AML_MotorControl_PIDSetTunings();

    AML_MotorControl_PIDSetSampleTime(20);
    AML_MotorControl_PIDSetOutputLimits(OutputMin, OutputMax);
    AML_MotorControl_PIDSetMode(_PID_MODE_AUTOMATIC);
}

void AML_MotorControl_PIDReset(PID_TypeDef *uPID)
{
    uPID->OutputSum = 0;
    uPID->LastInput = 0;
    uPID->LastTime = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// init motor control
void AML_MotorControl_Setup(void)
{
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    AML_MotorControl_PIDSetup();
}

void AML_MotorControl_LeftPWM(int32_t PWMValue)
{
    if (PWMValue > 100)
    {
        PWMValue = 100;
    }
    else if (PWMValue < -100)
    {
        PWMValue = -100;
    }

    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, direction);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, !direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWMValue);
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, !direction);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -PWMValue);
    }
    else if (PWMValue == 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }
}

void AML_MotorControl_RightPWM(int32_t PWMValue)
{
    if (PWMValue > 100)
    {
        PWMValue = 100;
    }
    else if (PWMValue < -100)
    {
        PWMValue = -100;
    }

    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, !direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMValue);
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, !direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -PWMValue);
    }
    else if (PWMValue == 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void AML_MotorControl_SetDirection(GPIO_PinState dir)
{
    direction = dir;
}

void AML_MotorControl_ToggleDirection(void)
{
    direction = ~direction;
}

void AML_MotorControl_SetMouseSpeed(int32_t speed)
{
    MouseSpeed = speed;
}

void AML_MotorControl_SetLeftSpeed(double speed, GPIO_PinState direction)
{
    Setpoint_Left = speed * Ratio;
    // Input_Left = ABS((double)AML_Encoder_GetLeftValue() / PulsePerRound);
    // AML_Encoder_ResetLeftValue();

    PID_Compute(&PID_SpeedLeft);

    AML_MotorControl_LeftPWM(Output_Left * direction);
}

void AML_MotorControl_SetRightSpeed(double speed, GPIO_PinState direction)
{
    Setpoint_Right = speed * Ratio;
    // Input_Right = ABS((double)AML_Encoder_GetRightValue() / PulsePerRound);
    // AML_Encoder_ResetRightValue();

    PID_Compute(&PID_SpeedRight);

    AML_MotorControl_RightPWM(Output_Right * direction);
}

void AML_MotorControl_Stop(void)
{
    AML_MotorControl_LeftPWM(0);
    AML_MotorControl_RightPWM(0);
}

void AML_MotorControl_ShortBreak(char c)
{
    uint8_t TurnSpeed = 20;
    uint8_t GoStraightSpeed = 40;

    if (c == 'L')
    {
        
        AML_MotorControl_LeftPWM(TurnSpeed);
        AML_MotorControl_RightPWM(-TurnSpeed);
    }
    else if (c == 'R')
    {
        AML_MotorControl_LeftPWM(-TurnSpeed);
        AML_MotorControl_RightPWM(TurnSpeed);
    }
    else if (c == 'F')
    {
        AML_MotorControl_LeftPWM(-GoStraightSpeed);
        AML_MotorControl_RightPWM(-GoStraightSpeed);
    }
    else if (c == 'B')
    {
        AML_MotorControl_LeftPWM(GoStraightSpeed);
        AML_MotorControl_RightPWM(GoStraightSpeed);
    }

    HAL_Delay(20);
    AML_MotorControl_Stop();
}

void AML_MotorControl_SetLeftWallValue(void)
{
    MinLeftWallDistance = AML_LaserSensor_ReadSingle(BL);
    MaxRightWallDistance = AML_LaserSensor_ReadSingle(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingle(FF);
}

void AML_MotorControl_SetRightWallValue(void)
{
    MaxLeftWallDistance = AML_LaserSensor_ReadSingle(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingle(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingle(FF);
}

void AML_MotorControl_LeftWallFollow(void)
{
    AML_DebugDevice_TurnOnLED(2);

    AML_DebugDevice_TurnOffLED(3);
    AML_DebugDevice_TurnOffLED(4);
    AML_DebugDevice_TurnOffLED(5);

    LeftWallDistance = (double)AML_LaserSensor_ReadSingle(BL);
    

    SetpointDistance = (double)(MinLeftWallDistance + 35);

    PID_Compute(&PID_LeftWallFollow);

    AML_MotorControl_LeftPWM((MouseSpeed + (int32_t)Output_Left));
    AML_MotorControl_RightPWM((MouseSpeed - (int32_t)Output_Left));
}

void AML_MotorControl_RightWallFollow(void)
{
    AML_DebugDevice_TurnOnLED(3);

    AML_DebugDevice_TurnOffLED(2);
    AML_DebugDevice_TurnOffLED(4);
    AML_DebugDevice_TurnOffLED(5);

    RightWallDistance = (double)AML_LaserSensor_ReadSingle(BR);

    SetpointDistance = (double)(MinRightWallDistance + 35);

    PID_Compute(&PID_RightWallFollow);

    AML_MotorControl_LeftPWM((MouseSpeed - (int32_t)Output_Right));
    AML_MotorControl_RightPWM((MouseSpeed + (int32_t)Output_Right));
}

void AML_MotorControl_GoStraight(void)
{
    if (AML_LaserSensor_ReadSingle(BL) < LEFT_WALL)
    {
        AML_MotorControl_LeftWallFollow();
    }
    else if (AML_LaserSensor_ReadSingle(BR) < RIGHT_WALL)
    {
        AML_MotorControl_RightWallFollow();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // UNUSED(htim);
    if (htim->Instance == htim9.Instance)
    {
        // AML_MotorControl_GoStraight();
        AML_MotorControl_LeftWallFollow();
        // AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
        debug[11]++;
    }

    debug[12]++;
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     /* Prevent unused argument(s) compilation warning */
//     UNUSED(htim);

//     /* NOTE : This function should not be modified, when the callback is needed,
//               the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
//      */
// }

void AML_MotorControl_TurnOnWallFollow(void)
{
    HAL_TIM_Base_Start_IT(&htim9);
    debug[10] = 1;
}

void AML_MotorControl_TurnOffWallFollow(void)
{
    HAL_TIM_Base_Stop_IT(&htim9);
    debug[10] = 0;
}

void AML_MotorControl_TurnLeft90(void)
{
    AML_MPUSensor_ResetAngle();

    Input_TurnLeft = (double)AML_MPUSensor_GetAngle();
    Setpoint_TurnLeft = Input_Left + 90.0f;
    // HAL_Delay(1000);

    while ((ReadButton != 2) && (ABS(Input_TurnLeft - Setpoint_TurnLeft) > 10.0f))
    {
        Input_TurnLeft = AML_MPUSensor_GetAngle();
        PID_Compute(&PID_TurnLeft);

        AML_MotorControl_LeftPWM(-(int32_t)Output_TurnLeft);
        AML_MotorControl_RightPWM((int32_t)Output_TurnLeft);
    }

    AML_MotorControl_ShortBreak('L');
    // AML_MotorControl_Stop();
    // AML_MotorControl_PIDReset(&PID_TurnLeft);
    AML_MPUSensor_ResetAngle();
}

void AML_MotorControl_TurnRight90(void)
{
    AML_MPUSensor_ResetAngle();

    Input_TurnRight = (double)AML_MPUSensor_GetAngle();
    Setpoint_TurnRight = Input_Right - 90.0f;
    // HAL_Delay(1000);

    while ((ReadButton != 2) && (ABS(Input_TurnRight - Setpoint_TurnRight) > 10.0f))
    {
        Input_TurnRight = AML_MPUSensor_GetAngle();
        PID_Compute(&PID_TurnRight);

        AML_MotorControl_LeftPWM(-(int32_t)Output_TurnRight);
        AML_MotorControl_RightPWM((int32_t)Output_TurnRight);
    }

    AML_MotorControl_ShortBreak('R');
    // AML_MotorControl_Stop();
    // AML_MotorControl_PIDReset(&PID_TurnRight);
    AML_MPUSensor_ResetAngle();
}

void AML_MotorControl_TurnLeft180(void)
{
    AML_MPUSensor_ResetAngle();

    Input_TurnLeft = (double)AML_MPUSensor_GetAngle();
    Setpoint_TurnLeft = Input_Left + 180.0f;
    // HAL_Delay(1000);

    while ((ReadButton != 2) && (ABS(Input_TurnLeft - Setpoint_TurnLeft) > 10.0f))
    {
        Input_TurnLeft = AML_MPUSensor_GetAngle();
        PID_Compute(&PID_TurnLeft);

        AML_MotorControl_LeftPWM(-(int32_t)Output_TurnLeft);
        AML_MotorControl_RightPWM((int32_t)Output_TurnLeft);
    }

    AML_MotorControl_ShortBreak('L');
    // AML_MotorControl_Stop();
    // AML_MotorControl_PIDReset(&PID_TurnLeft);
    AML_MPUSensor_ResetAngle();
}

void AML_MotorControl_TurnRight180(void)
{
    AML_MPUSensor_ResetAngle();

    Input_TurnRight = (double)AML_MPUSensor_GetAngle();
    Setpoint_TurnRight = Input_Right - 180.0f;
    // HAL_Delay(1000);

    while ((ReadButton != 2) && (ABS(Input_TurnRight - Setpoint_TurnRight) > 10.0f))
    {
        Input_TurnRight = AML_MPUSensor_GetAngle();
        PID_Compute(&PID_TurnRight);

        AML_MotorControl_LeftPWM(-(int32_t)Output_TurnRight);
        AML_MotorControl_RightPWM((int32_t)Output_TurnRight);
    }

    AML_MotorControl_ShortBreak('R');
    // AML_MotorControl_Stop();
    // AML_MotorControl_PIDReset(&PID_TurnRight);
    AML_MPUSensor_ResetAngle();
}

void AML_MotorControl_LeftStillTurn(void)
{
    AML_DebugDevice_TurnOnLED(4);

    AML_DebugDevice_TurnOffLED(2);
    AML_DebugDevice_TurnOffLED(3);
    AML_DebugDevice_TurnOffLED(5);

    AML_MotorControl_TurnOnWallFollow();
    // AML_MotorControl_TurnOffWallFollow();

    while (AML_LaserSensor_ReadSingle(FF) > FRONT_WALL)
    {
        // AML_MotorControl_GoStraight();
    }

    AML_MotorControl_TurnOffWallFollow();

    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_Stop();

    AML_MotorControl_TurnLeft90();

    AML_MotorControl_TurnOnWallFollow();
}

void AML_MotorControl_RightStillTurn(void)
{
    AML_DebugDevice_TurnOnLED(5);

    AML_DebugDevice_TurnOffLED(2);
    AML_DebugDevice_TurnOffLED(3);
    AML_DebugDevice_TurnOffLED(4);

    AML_MotorControl_TurnOnWallFollow();
    // AML_MotorControl_TurnOffWallFollow();

    while (AML_LaserSensor_ReadSingle(FF) > FRONT_WALL)
    {
        // AML_MotorControl_GoStraight();
    }

    AML_MotorControl_TurnOffWallFollow();

    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_Stop();

    AML_MotorControl_TurnRight90();

    AML_MotorControl_TurnOnWallFollow();
}

void AML_MotorControl_BackStillTurn(void)
{

    AML_MotorControl_TurnOnWallFollow();
    // AML_MotorControl_TurnOffWallFollow();

    while (AML_LaserSensor_ReadSingle(FF) > FRONT_WALL)
    {
        // AML_MotorControl_GoStraight();
    }

    AML_MotorControl_TurnOffWallFollow();

    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_Stop();

    AML_MotorControl_TurnLeft180();

    AML_MotorControl_TurnOnWallFollow();
}


