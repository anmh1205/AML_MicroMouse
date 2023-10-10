#ifndef AML_LASERSENSOR_H
#define AML_LASERSENSOR_H

#include "stm32f4xx.h"
#include "vl53l0x_api.h"

typedef enum
{
    FL,
    FF,
    FR,
    RR,
    BR,
    BL,
    RL

} Laser;

void scan_i2c_bus(I2C_HandleTypeDef *hi2c);
uint8_t AML_LaserSensor_Setup();
void AML_LaserSensor_ReadSingle();

#endif