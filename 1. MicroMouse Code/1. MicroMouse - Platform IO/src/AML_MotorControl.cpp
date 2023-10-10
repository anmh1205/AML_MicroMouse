/*
  Library for control 2 motor 

  Last updated: 11AM 14/06/2023 UTC+7
  Authors: anmh1205

*/


#include <AML_MotorControl.h>

Servo motorL;
Servo motorR;


#define LPWM1 4 // Chân kết nối đến chân IN1 của động cơ L
#define RPWM1 5 // Chân kết nối đến chân IN2 của động cơ L
#define LPWM2 6 // Chân kết nối đến chân IN3 của động cơ R
#define RPWM2 7 // Chân kết nối đến chân IN4 của động cơ R




// Hàm điều khiển động cơ với ESC. Nhận giá trị -100 đến 100, giá trị âm đi lùi, dương đi tiến
void AML_MotorControl_ESC(int speedL, int speedR)
{
  if (speedL < 0)
  {
    speedL = map(abs(speedL), 0, 100, 90, 0);
  }
  else if (speedL > 0)
  {
    speedL = map(speedL, 0, 100, 110, 180);
  }
  else if (speedL == 0)
  {
    speedL = 100;
  }

  if (speedR < 0)
  {
    speedR = map(abs(speedR), 0, 100, 90, 0);
  }
  else if (speedR > 0)
  {
    speedR = map(speedR, 0, 100, 110, 180);
  }
  else if (speedR == 0)
  {
    speedR = 100;
  }

  motorL.write(speedL);
  motorR.write(speedR);

  Serial.print(speedL);
  Serial.print("      ");
  Serial.println(speedR);
}

// Xuất xung điều khiển động cơ: Truyền giá trị dương đi tiến, giá trị âm đi lùi
void AML_MotorControl_PWM(int pwmL, int pwmR)
{
  if (pwmL > 0) // L tiến
  {
    analogWrite(LPWM1, pwmL);
    analogWrite(RPWM1, 0);
  }
  else if (pwmL < 0) // L lùi
  {
    analogWrite(LPWM1, 0);
    analogWrite(RPWM1, abs(pwmL));
  }
  else if (pwmL == 0) // L đứng yên
  {
    analogWrite(LPWM1, 0);
    analogWrite(RPWM1, 0);
  }

  if (pwmR > 0) // R tiến
  {
    analogWrite(LPWM2, pwmR);
    analogWrite(RPWM2, 0);
  }
  else if (pwmR < 0) // R lùi
  {
    analogWrite(LPWM2, 0);
    analogWrite(RPWM2, abs(pwmR));
  }
  else if (pwmR == 0) // R đứng yên
  {
    analogWrite(LPWM2, 0);
    analogWrite(RPWM2, 0);
  }
}


void AML_MotorControl_setupL298()
{
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
}
