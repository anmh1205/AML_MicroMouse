#include "AML_LaserSensor.h"

#define I2C_TIMEOUT 100

uint8_t LaserSensorAddress[] = {0x29, 0x59, 0x31, 0x32, 0x57, 0x58};

extern I2C_HandleTypeDef hi2c1;
extern int16_t debug[100];

VL53L0X_RangingMeasurementData_t SensorValue[6];
VL53L0X_Dev_t Dev_Val[6];
VL53L0X_DEV Laser[6];

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

void AML_LaserSensor_ScanI2CDevice(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;
    for (i = 1; i < 128; i++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, i << 1, 2, I2C_TIMEOUT) == HAL_OK)
        {
            // printf("I2C device found at address 0x%X\n", i);
            debug[i] = 1;
        }
    }
}

void AML_LaserSensor_Init(uint8_t i)
{
    VL53L0X_WaitDeviceBooted(Laser[i]);
    VL53L0X_DataInit(Laser[i]);
    VL53L0X_StaticInit(Laser[i]);
    VL53L0X_PerformRefCalibration(Laser[i], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Laser[i], &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(Laser[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);

    // Enable/Disable Sigma and Signal check
    VL53L0X_SetLimitCheckEnable(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
    VL53L0X_SetLimitCheckValue(Laser[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Laser[i], 33000);
    VL53L0X_SetVcselPulsePeriod(Laser[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(Laser[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

uint8_t AML_LaserSensor_Setup()
{
    uint8_t DelayTime = 80;
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

    return 1;
}

void AML_LaserSensor_ReadAll()
{
    VL53L0X_PerformSingleRangingMeasurement(Laser[FL], &SensorValue[FL]);
    VL53L0X_PerformSingleRangingMeasurement(Laser[FF], &SensorValue[FF]);
    VL53L0X_PerformSingleRangingMeasurement(Laser[FR], &SensorValue[FR]);
    VL53L0X_PerformSingleRangingMeasurement(Laser[BR], &SensorValue[BR]);
    VL53L0X_PerformSingleRangingMeasurement(Laser[BL], &SensorValue[BL]);
}

uint16_t AML_LaserSensor_ReadSingle(uint8_t name)
{
    VL53L0X_PerformSingleRangingMeasurement(Laser[name], &SensorValue[name]);
    // VL53L0X_GetRangingMeasurementData(Laser[name], &SensorValue[name]);
    return SensorValue[name].RangeMilliMeter;
}