/*
    Last updated: 11AM 14/06/2023 UTC+7
    Authors: anmh1205

*/

#include <AML_MPUSensor.h>

// Setup MPU class
MPU6050 mpu(Wire);

void AML_MPUSensor_setup()
{
  Serial2.begin(115200);

}

// Trả về giá trị góc quay theo trục Z
float AML_MPUSensor_readZAngle()
{

}
