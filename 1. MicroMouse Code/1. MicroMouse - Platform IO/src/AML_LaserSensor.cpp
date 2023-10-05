/*
  Library for 7 VL53L0x sensor

  Last updated: 11PM 13/06/2023 UTC+7
  Authors: anmh1205

*/

#include <AML_LaserSensor.h>

// Khởi tạo theo thư viện
VL53L0X FL;
VL53L0X FF;
VL53L0X FR;
VL53L0X RR;
VL53L0X BR;
VL53L0X BL;
VL53L0X RL;

// set the pins to shutdown for all 7 sensors

#define SHT_L0X_FL 22 // XSHUT Pin laser 1
#define SHT_L0X_FF 24 // XSHUT Pin laser 2
#define SHT_L0X_FR 26 // XSHUT Pin laser 3
#define SHT_L0X_RR 28 // XSHUT Pin laser 4
#define SHT_L0X_BR 30 // XSHUT Pin laser 5
#define SHT_L0X_BL 32 // XSHUT Pin laser 6
#define SHT_L0X_RL 34 // XSHUT Pin laser 7

// address we will assign for all 7 sensor

#define L0X_FL_ADDRESS 0x28
#define L0X_FF_ADDRESS 0x30
#define L0X_FR_ADDRESS 0x31
#define L0X_RR_ADDRESS 0x32
#define L0X_BR_ADDRESS 0x33
#define L0X_BL_ADDRESS 0x34
#define L0X_RL_ADDRESS 0x35

// Mảng giá trị cảm biến
int AML_laserSensorValue[8];

// bật từng laser lên để cài địa chỉ bằng chân XSHUT
void AML_LaserSensor_setup()
{
  pinMode(SHT_L0X_FL, OUTPUT);
  pinMode(SHT_L0X_FF, OUTPUT);
  pinMode(SHT_L0X_FR, OUTPUT);
  pinMode(SHT_L0X_RR, OUTPUT);
  pinMode(SHT_L0X_BR, OUTPUT);
  pinMode(SHT_L0X_BL, OUTPUT);
  pinMode(SHT_L0X_RL, OUTPUT);

  Serial.println(0);
  // Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_L0X_FL, LOW);
  digitalWrite(SHT_L0X_FF, LOW);
  digitalWrite(SHT_L0X_FR, LOW);
  digitalWrite(SHT_L0X_RR, LOW);
  digitalWrite(SHT_L0X_BR, LOW);
  digitalWrite(SHT_L0X_BL, LOW);
  digitalWrite(SHT_L0X_RL, LOW);

  Serial.println(1);

  // all unreset
  digitalWrite(SHT_L0X_FL, HIGH);
  digitalWrite(SHT_L0X_FF, HIGH);
  digitalWrite(SHT_L0X_FR, HIGH);
  digitalWrite(SHT_L0X_RR, HIGH);
  digitalWrite(SHT_L0X_BR, HIGH);
  digitalWrite(SHT_L0X_BL, HIGH);
  digitalWrite(SHT_L0X_RL, HIGH);
  delay(10);

  Serial.println(2);

  // activating L0X_L and reseting L0X_F
  digitalWrite(SHT_L0X_FL, HIGH);
  digitalWrite(SHT_L0X_FF, LOW);
  digitalWrite(SHT_L0X_FR, LOW);
  digitalWrite(SHT_L0X_RR, LOW);
  digitalWrite(SHT_L0X_BR, LOW);
  digitalWrite(SHT_L0X_BL, LOW);
  digitalWrite(SHT_L0X_RL, LOW);

  Serial.println(3);

  delay(10);
  FL.setAddress(L0X_FL_ADDRESS);

  Serial.println(4);

  digitalWrite(SHT_L0X_FF, HIGH);
  delay(10);
  FF.setAddress(L0X_FF_ADDRESS);

  Serial.println(5);

  digitalWrite(SHT_L0X_FR, HIGH);
  delay(10);
  FR.setAddress(L0X_FR_ADDRESS);

  Serial.println(6);

  digitalWrite(SHT_L0X_RR, HIGH);
  delay(10);
  RR.setAddress(L0X_RR_ADDRESS);

  Serial.println(7);

  digitalWrite(SHT_L0X_BR, HIGH);
  delay(10);
  BR.setAddress(L0X_BR_ADDRESS);

  Serial.println(8);

  digitalWrite(SHT_L0X_BL, HIGH);
  delay(10);
  BL.setAddress(L0X_BL_ADDRESS);

  Serial.println(9);

  digitalWrite(SHT_L0X_RL, HIGH);
  delay(10);
  RL.setAddress(L0X_RL_ADDRESS);

  Serial.println(10);

  delay(10);

  FL.init();
  FF.init();
  FR.init();
  RR.init();
  BR.init();
  BL.init();
  RL.init();

  Serial.println(11);

  // Gắn cờ lỗi
  FL.setTimeout(500);
  FF.setTimeout(500);
  FR.setTimeout(500);
  RR.setTimeout(500);
  BR.setTimeout(500);
  BL.setTimeout(500);
  RL.setTimeout(500);

  Serial.println(12);

  // // reduce timing budget to 10 ms (default is about 33 ms)
  FL.setMeasurementTimingBudget(10);
  FF.setMeasurementTimingBudget(10);
  FR.setMeasurementTimingBudget(10);
  RR.setMeasurementTimingBudget(10);
  BR.setMeasurementTimingBudget(10);
  BL.setMeasurementTimingBudget(10);
  RL.setMeasurementTimingBudget(10);

  FF.startContinuous();
  FR.startContinuous();
  RR.startContinuous();
  BR.startContinuous();
  BL.startContinuous();
  RL.startContinuous();
  FL.startContinuous();

  Serial.println("Laser sensor inited...");
}

// Đọc cảm biến có số thứ tự là số truyền vào hàm
float AML_LaserSensor_readSingle(int laserNumber)
{
  AML_LaserSensor_readAll(AML_laserSensorValue);
  return AML_laserSensorValue[laserNumber];
}


// Đọc laser (không ghi ra serial)
void AML_LaserSensor_readAll(int *userArrayAddress)
{
  *(userArrayAddress + 0) = FL.readRangeContinuousMillimeters();

  *(userArrayAddress + 1) = FF.readRangeContinuousMillimeters();

  *(userArrayAddress + 2) = FR.readRangeContinuousMillimeters();

  *(userArrayAddress + 3) = RR.readRangeContinuousMillimeters();

  *(userArrayAddress + 4) = BR.readRangeContinuousMillimeters();

  *(userArrayAddress + 5) = BL.readRangeContinuousMillimeters();

  *(userArrayAddress + 6) = RL.readRangeContinuousMillimeters();
}

// Hàm này giống hệt readLaserSensor(), khác mỗi cái có ghi ra serial để xem giá trị
void AML_LaserSensor_readAllTest(int *userArrayAddress)
{
  *(userArrayAddress + 0) = FL.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[0]);

  Serial.print("   ");

  *(userArrayAddress + 1) = FF.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[1]);

  Serial.print("   ");

  *(userArrayAddress + 2) = FR.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[2]);

  Serial.print("   ");

  *(userArrayAddress + 3) = RR.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[3]);

  Serial.print("   ");

  *(userArrayAddress + 4) = BR.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[4]);

  Serial.print("   ");

  *(userArrayAddress + 5) = BL.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[5]);

  Serial.print("   ");

  *(userArrayAddress + 6) = RL.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[6]);

  Serial.println("   ");
}