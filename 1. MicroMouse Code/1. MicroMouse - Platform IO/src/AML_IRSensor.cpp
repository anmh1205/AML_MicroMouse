/*
  IR sensor library for emergency escape 

  Last updated: 10AM 14/06/2023 UTC+7
  Authors: anmh1205

*/

#include <AML_IRSensor.h>


#define IR_FL 3  // IR sensor FL pin
#define IR_FR 2  // IR sensor FR pin
#define IR_BR 19 // IR sensor BR pin
#define IR_BL 18 // IR sensor BL pin


// cờ interrupt
boolean flagInterrupt0 = false;
boolean flagInterrupt1 = false;
boolean flagInterrupt4 = false;
boolean flagInterrupt5 = false;


long timer1 = 0;


// Hàm khi kích hoạt ngắt sẽ treo cho đến khi thoát hiểm thành công
void AML_IRSensor_standby()
{
  unsigned time = 700; // tạo biến để chỉnh thời gian chạy thoát hiểm cho dễ

  // treo
  while (flagInterrupt0 && millis() - timer1 < time)
    ;

  while (flagInterrupt1 && millis() - timer1 < time)
    ;

  while (flagInterrupt4 && millis() - timer1 < time)
    ;
  while (flagInterrupt5 && millis() - timer1 < time)
    ;

  // reset hết cờ
  flagInterrupt0 = false;
  flagInterrupt1 = false;
  flagInterrupt4 = false;
  flagInterrupt5 = false;
}

// FrontLeft
void Interrupt_0()
{
  AML_MotorControl_PWM(-150, -150);       // thoát hiểm ngay
  flagInterrupt0 = true; // bật cờ
  timer1 = millis();     // lấy mốc thời gian lúc bắt được thoát hiểm

  // tắt các cờ khác để luôn chỉ có 1 thoát hiểm
  flagInterrupt1 = false;
  flagInterrupt4 = false;
  flagInterrupt5 = false;
}

// FrontRight
void Interrupt_1()
{
  AML_MotorControl_PWM(-150, -150);
  flagInterrupt1 = true;
  timer1 = millis();

  flagInterrupt0 = false;
  flagInterrupt4 = false;
  flagInterrupt5 = false;
}

// BackLeft
void Interrupt_4()
{
  AML_MotorControl_PWM(150, 150);
  flagInterrupt4 = true;
  timer1 = millis();

  flagInterrupt0 = false;
  flagInterrupt1 = false;
  flagInterrupt5 = false;
}

// BackRight
void Interrupt_5()
{
  AML_MotorControl_PWM(150, 150);
  flagInterrupt5 = true;
  timer1 = millis();

  flagInterrupt0 = false;
  flagInterrupt1 = false;
  flagInterrupt4 = false;
}


// 
void AML_IRSensor_setup()
{
  attachInterrupt(digitalPinToInterrupt(IR_FL), Interrupt_0, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_FR), Interrupt_1, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_BR), Interrupt_4, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_BL), Interrupt_5, RISING);
}

