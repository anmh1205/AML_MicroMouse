#include "AML_LaserSensor.h"

uint8_t LaserSensorAddress[] = {0x29, 0x59, 0x60, 0x32, 0x57};

SimpleKalmanFilter KalmanFilter[5];

extern I2C_HandleTypeDef hi2c1;

VL53L0X_RangingMeasurementData_t SensorValue[7];
VL53L0X_Dev_t Dev_Val[7];
VL53L0X_DEV Laser[7];

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

void AML_LaserSensor_Init(uint8_t i)
{
    VL53L0X_WaitDeviceBooted(Laser[i]);
    VL53L0X_DataInit(Laser[i]);
    VL53L0X_StaticInit(Laser[i]);
    VL53L0X_PerformRefCalibration(Laser[i], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Laser[i], &refSpadCount, &isApertureSpads);

    // VL53L0X_SetDeviceMode(Laser[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);
    VL53L0X_SetDeviceMode(Laser[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_StartMeasurement(Laser[i]);

    // Enable/Disable Sigma and Signal check
    VL53L0X_SetLimitCheckEnable(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
    VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Laser[i], 20000);
    VL53L0X_SetVcselPulsePeriod(Laser[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(Laser[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

void AML_LaserSensor_Setup(void)
{
    uint8_t DelayTime = 70;
    // disable all laser
    HAL_GPIO_WritePin(XSHUT_FL_GPIO_Port, XSHUT_FL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FF_GPIO_Port, XSHUT_FF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FR_GPIO_Port, XSHUT_FR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_BR_GPIO_Port, XSHUT_BR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_BL_GPIO_Port, XSHUT_BL_Pin, GPIO_PIN_RESET);
    HAL_Delay(DelayTime);

    // enable laser FL and init
    HAL_GPIO_WritePin(XSHUT_FL_GPIO_Port, XSHUT_FL_Pin, GPIO_PIN_SET);
    HAL_Delay(DelayTime);
    Laser[FL] = &Dev_Val[FL];
    Laser[FL]->I2cHandle = &hi2c1;
    Laser[FL]->I2cDevAddr = 0x52;
    VL53L0X_SetDeviceAddress(Laser[FL], LaserSensorAddress[FL]);
    Laser[FL]->I2cDevAddr = LaserSensorAddress[FL];
    AML_LaserSensor_Init(FL);

    // enable laser FF and init
    HAL_GPIO_WritePin(XSHUT_FF_GPIO_Port, XSHUT_FF_Pin, GPIO_PIN_SET);
    HAL_Delay(DelayTime);
    Laser[FF] = &Dev_Val[FF];
    Laser[FF]->I2cHandle = &hi2c1;
    Laser[FF]->I2cDevAddr = 0x52;
    VL53L0X_SetDeviceAddress(Laser[FF], LaserSensorAddress[FF]);
    Laser[FF]->I2cDevAddr = LaserSensorAddress[FF];
    AML_LaserSensor_Init(FF);

    // enable laser FR and init
    HAL_GPIO_WritePin(XSHUT_FR_GPIO_Port, XSHUT_FR_Pin, GPIO_PIN_SET);
    HAL_Delay(DelayTime);
    Laser[FR] = &Dev_Val[FR];
    Laser[FR]->I2cHandle = &hi2c1;
    Laser[FR]->I2cDevAddr = 0x52;
    VL53L0X_SetDeviceAddress(Laser[FR], LaserSensorAddress[FR]);
    Laser[FR]->I2cDevAddr = LaserSensorAddress[FR];
    AML_LaserSensor_Init(FR);

    // enable laser BR and init
    HAL_GPIO_WritePin(XSHUT_BR_GPIO_Port, XSHUT_BR_Pin, GPIO_PIN_SET);
    HAL_Delay(DelayTime);
    Laser[BR] = &Dev_Val[BR];
    Laser[BR]->I2cHandle = &hi2c1;
    Laser[BR]->I2cDevAddr = 0x52;
    VL53L0X_SetDeviceAddress(Laser[BR], LaserSensorAddress[BR]);
    Laser[BR]->I2cDevAddr = LaserSensorAddress[BR];
    AML_LaserSensor_Init(BR);

    // enable laser BL and init
    HAL_GPIO_WritePin(XSHUT_BL_GPIO_Port, XSHUT_BL_Pin, GPIO_PIN_SET);
    HAL_Delay(DelayTime);
    Laser[BL] = &Dev_Val[BL];
    Laser[BL]->I2cHandle = &hi2c1;
    Laser[BL]->I2cDevAddr = 0x52;
    VL53L0X_SetDeviceAddress(Laser[BL], LaserSensorAddress[BL]);
    Laser[BL]->I2cDevAddr = LaserSensorAddress[BL];
    AML_LaserSensor_Init(BL);

    for (uint8_t i = 0; i < 5; i++)
    {
        SimpleKalmanFilter_Init(&KalmanFilter[i], 0.01, 0.01, 0.001);
    }
}

void AML_LaserSensor_ReadAll(void)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        // VL53L0X_GetRangingMeasurementData(Laser[i], &SensorValue[i]);
        VL53L0X_PerformSingleRangingMeasurement(Laser[i], &SensorValue[i]);
        SensorValue[i].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[i], SensorValue[i].RangeMilliMeter);
    }
}

int32_t AML_LaserSensor_ReadSingle(uint8_t name)
{
    // VL53L0X_PerformSingleRangingMeasurement(Laser[name], &SensorValue[name]);
    VL53L0X_GetRangingMeasurementData(Laser[name], &SensorValue[name]);

    SensorValue[name].RangeMilliMeter = (uint16_t)SimpleKalmanFilter_updateEstimate(&KalmanFilter[name], SensorValue[name].RangeMilliMeter);
    VL53L0X_StartMeasurement(Laser[name]);
    return (int32_t)SensorValue[name].RangeMilliMeter;
}

uint8_t AML_LaserSensor_WallFavor(void)
{
    if (AML_LaserSensor_ReadSingle(FL < 100))  //North
        return 0; 
    else if (AML_LaserSensor_ReadSingle(FF < 100))  //East
        return 1;
    else if (AML_LaserSensor_ReadSingle(FR < 100))  //South
        return 2;
    else if (AML_LaserSensor_ReadSingle(BR < 100))  //West
        return 3;
}
