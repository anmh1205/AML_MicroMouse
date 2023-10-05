/*
  IR sensor library for emergency escape

  Last updated: 10AM 14/06/2023 UTC+7
  Authors: anmh1205

*/

#include <AML_IRSensor.h>

const int IR_FL = 3;
const int IR_FR = 2;
const int IR_BR = 19;
const int IR_BL = 18;

volatile boolean flagInterrupt[4] = {false, false, false, false};
volatile unsigned long timer1 = 0;

// Wait for all interrupts to clear or timeout
void AML_IRSensor_standby()
{
  unsigned time = 700;

  while (millis() - timer1 < time && (flagInterrupt[0] || flagInterrupt[1] || flagInterrupt[2] || flagInterrupt[3]))
    ;

  // Reset all flags
  for (int i = 0; i < 4; i++)
    flagInterrupt[i] = false;
}

// Handle all interrupts
void InterruptHandler(int index, int pwmL, int pwmR)
{
  AML_MotorControl_PWM(pwmL, pwmR);
  flagInterrupt[index] = true;
  timer1 = millis();

  // Reset all flags except the current one
  for (int i = 0; i < 4; i++)
  {
    if (i != index)
      flagInterrupt[i] = false;
  }
}

// FrontLeft
void Interrupt_0() { InterruptHandler(0, -150, -150); }

// FrontRight
void Interrupt_1() { InterruptHandler(1, -150, -150); }

// BackLeft
void Interrupt_2() { InterruptHandler(2, 150, 150); }

// BackRight
void Interrupt_3() { InterruptHandler(3, 150, 150); }

// Set up interrupt handlers
void AML_IRSensor_setup()
{
  attachInterrupt(digitalPinToInterrupt(IR_FL), Interrupt_0, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_FR), Interrupt_1, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_BR), Interrupt_2, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_BL), Interrupt_3, RISING);
}