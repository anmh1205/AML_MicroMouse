/*
  ADKeyboard 1.0 library

  Last updated: 10PM 13/06/2023 UTC+7
  Authors: anmh1205

*/

#include <AML_Keyboard.h>

#define analogKey A9

// ADKeyboard Module
int adc_key_val[5] = {30, 70, 110, 150, 600};
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;
int keyValue; // lưu nút đã bấm

void (*resetFunc)(void) = 0; // declare reset function at address 0

// Convert ADC value to key number
int AML_Keyboard_getKey(int inputValue)
{
  int k;
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (inputValue < adc_key_val[k])
    {
      return k;
    }
  }
  
  return k;
}

// Đọc nút cho đến khi phát hiện bấm nút
int AML_Keyboard_readKeyLoop()
{
  int oldkey = -1;
  int keyValue = -1;

  while (keyValue == -1)
  {
    int adc_key_in = analogRead(0); // read the value from the sensor pin A0
    int key = AML_Keyboard_getKey(adc_key_in);  // convert into key press

    if (key != oldkey) // if keypress is detected
    {
      delay(5); // wait for debounce time
      adc_key_in = analogRead(0); // read the value from the sensor
      key = AML_Keyboard_getKey(adc_key_in); // convert into key press

      if (key != oldkey && key >= 0)
      {
        oldkey = key;
        keyValue = key;
      }
    }
  }

  return keyValue;
}

// Đọc nút reset
void AML_Keyboard_readResetKey()
{
  int adc_key_in = analogRead(0); // read the value from the sensor pin A0
  int key = AML_Keyboard_getKey(adc_key_in); // convert into key press

  if (key != oldkey) // if keypress is detected
  {
    delay(5); // wait for debounce time
    adc_key_in = analogRead(0); // read the value from the sensor
    key = AML_Keyboard_getKey(adc_key_in); // convert into key press

    if (key != oldkey && key == 1) // if keypress is detected and it's button B
    {
      resetFunc(); // reset command
    }

    oldkey = key;
  }
}