/*
    Last updated: 11AM 14/06/2023 UTC+7
    Authors: Trung

*/

#include <AML_MPUSensor.h>

// Setup MPU class
MPU6050 mpu(Wire);

void AML_MPUSensor_setup()
{
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");
}

// Trả về giá trị góc quay theo trục Z
float AML_MPUSensor_readZAngle()
{
  // int i = 1;
  float zAngle = 0;

  mpu.update();

  zAngle = mpu.getAngleZ();
  return zAngle;
}
