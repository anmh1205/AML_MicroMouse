#include "AML_LaserSensor.h"

#define I2C_TIMEOUT 100

extern I2C_HandleTypeDef hi2c1;
extern int16_t debug[100];

VL53L0X_RangingMeasurementData_t SensorValue;

VL53L0X_Dev_t Laser_Val;
VL53L0X_DEV Dev = &Laser_Val;

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

void scan_i2c_bus(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;
    for (i = 1; i < 128; i++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, i << 1, 2, I2C_TIMEOUT) == HAL_OK)
        {
            printf("I2C device found at address 0x%X\n", i);
            debug[7] = i;
        }
        debug[4]++;
    }
}

uint8_t AML_LaserSensor_Setup()
{

    Dev->I2cHandle = &hi2c1;
    Dev->I2cDevAddr = 0x52;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(20);
    //
    // VL53L0X init for Single Measurement
    //

    VL53L0X_WaitDeviceBooted(Dev);
    VL53L0X_DataInit(Dev);
    VL53L0X_StaticInit(Dev);
    VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    // Enable/Disable Sigma and Signal check
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

void AML_LaserSensor_ReadSingle()
{
    // debug[3]++;
    debug[5] = VL53L0X_PerformSingleRangingMeasurement(Dev, &SensorValue);
    debug[6] = SensorValue.RangeMilliMeter;
}
