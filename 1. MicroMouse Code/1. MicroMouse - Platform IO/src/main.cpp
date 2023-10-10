/*
  SumoRobot UTC 2023
  Stated: 30/03/2023
  LastUpdate: 3AM 17/04/2023

  Authors: An, Trung, Bình
*/

#include <Arduino.h>
#include <math.h>

#include <AML_LaserSensor.h>
#include <AML_Keyboard.h>
#include <AML_MotorControl.h>
#include <AML_IRSensor.h>
#include <AML_MPUSensor.h>

#include <MPU6050_light.h>

#include <SPI.h>
#include <Wire.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp = 1.5, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter laserKF(2, 2, 0.01);

int count = 0;

int sensorValue[8]; // Mảng giá trị cảm biến
int mark[8];        // Mảng đánh dấu số thứ tự của laser khi sắp xếp

void (*func)(int);  // con trỏ trỏ đến hàm tương ứng với nút đã bấm
void (*plan)(void); // con trỏ trỏ đến hàm chiến thuật tương ứng với nút đã bấm

void scanI2CAddress()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(500); // wait 5 seconds for next scan
}

void setupLCD()
{
  lcd.init();
  lcd.backlight();
}

// truyền xâu vào đây để ghi lên LCD
void printLCD(String s)
{
  lcd.setCursor(0, 0);
  lcd.print(s);
}

// nút C
void plan1()
{
}

// nút D
void plan2()
{
}

// nút E
void plan3()
{
}

void plan4()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  AML_MotorControl_setupL298();
  AML_MotorControl_PWM(0, 0);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  // AML_MPUSensor_setup();

  // scanI2CAddress();
  AML_LaserSensor_setup();
  // scanI2CAddress();

  // setupLCD();
  // printLCD("Standby");

  // switch (AML_Keyboard_readKeyLoop())
  // {
  // case 0:
  //   // func = &search2_An;
  //   plan = &plan4;
  //   break;
  // case 2: // nút C
  //   // func = &search2_An;
  //   plan = &plan1;
  //   // printLCD("Select AN");
  //   break;
  // case 3: // nút D
  //   // func = &search2_An;
  //   plan = &plan2;
  //   break;
  // case 4:
  //   // func = &search2_An;
  //   plan = &plan3;
  //   break;
  // }

  digitalWrite(13, 1);
  // delay(700); // Theo luật thi đấu, cơ mà giờ đang test nên không cần
  // printLCD("              ");
  // AML_IRSensor_setup();

  // AML_Keyboard_readKeyLoop();
  // AML_MotorControl_PWM(255, 255);

  Input = 0;
  Setpoint = 50;
  myPID.SetOutputLimits(-100, 100);
  myPID.SetMode(AUTOMATIC);
}

void tune()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 'p')
    {
      Kp += 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (c == 'o')
    {
      Kp -= 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (c == 'i')
    {
      Ki += 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (c == 'u')
    {
      Ki -= 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (c == 'd')
    {
      Kd += 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (c == 's')
    {
      Kd -= 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (c == 'r')
    {
      Kp = 0.4;
      Ki = 0.1;
      Kd = 0.3;
      myPID.SetTunings(Kp, Ki, Kd);
    }
  }
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print("   Ki: ");
  Serial.print(Ki);
  Serial.print("   Kd: ");
  Serial.print(Kd);
}

void pidFunction()
{
  AML_LaserSensor_readAll(sensorValue);

  // Input = sensorValue[Laser_RL] - sensorValue[Laser_RR];
  // Input = sensorValue[Laser_RL];
  Input = laserKF.updateEstimate(sensorValue[Laser_RL]);
  myPID.Compute();
  // AML_MotorControl_PWM(255 - Output, 255 + Output);

  Serial.print("   Input: ");
  Serial.print(Input);
  Serial.print("   Output: ");
  Serial.println(Output);
}

void loop()
{
  // AML_IRSensor_standby();

  // Serial.println(AML_LaserSensor_readSingle(Laser_RL));

  // AML_LaserSensor_readAllTest(sensorValue);
  // func(minSensorValue()); // Gọi hàm được trỏ

  tune();
  pidFunction();
  AML_Keyboard_readResetKey(); // Quét nút
}