#ifndef AML_Keyboard_h
#define AML_Keyboard_h

#include <Arduino.h>
#include <math.h>

int AML_Keyboard_getKey(int inputValue);
int AML_Keyboard_readKeyLoop();
void AML_Keyboard_readResetKey();

#endif