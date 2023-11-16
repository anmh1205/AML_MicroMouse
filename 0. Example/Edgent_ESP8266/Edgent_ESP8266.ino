// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLgf0Sc9xE"
#define BLYNK_DEVICE_NAME "Esmart"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

#include "BlynkEdgent.h"

#define led1 14 //D5
#define led2 12 //D6
#define led3 13 //D7
#define led4 15 //D8 
#define button1 5 //D1
#define button2 4 //D2
#define button3 0 //D3
#define button4 2 //D4
boolean bt1_state=HIGH;
boolean bt2_state=HIGH;
boolean bt3_state=HIGH;
boolean bt4_state=HIGH;
unsigned long times=millis();
WidgetLED led_connect(V0);

void setup()
{
  Serial.begin(115200);
  delay(100);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();
  
  if(millis()-times>1000){
    Blynk.virtualWrite(V5, millis()/1000);
    if (led_connect.getValue()){
      led_connect.off();
    }else {
      led_connect.on();
    }
    times=millis();
  }
  check_button();
}
BLYNK_WRITE(V1){
  int p = param.asInt();
  digitalWrite(led1, p); 
}
BLYNK_WRITE(V2){
  int p = param.asInt();
  digitalWrite(led2, p); 
}
BLYNK_WRITE(V3){
  int p = param.asInt();
  digitalWrite(led3, p); 
}
BLYNK_WRITE(V4){
  int p = param.asInt();
  digitalWrite(led4, p); 
}
void check_button(){
  if(digitalRead(button1)==LOW){
    if(bt1_state==HIGH){
      digitalWrite(led1,!digitalRead(led1));
      Blynk.virtualWrite(V1,digitalRead(led1));
      bt1_state=LOW;
      delay(200);
    }
  }else{
    bt1_state=HIGH;
  }
  if(digitalRead(button2)==LOW){
    if(bt2_state==HIGH){
      digitalWrite(led2,!digitalRead(led2));
      Blynk.virtualWrite(V2,digitalRead(led2));
      bt2_state=LOW;
      delay(200);
    }
  }else{
    bt2_state=HIGH;
  }
  if(digitalRead(button3)==LOW){
    if(bt3_state==HIGH){
      digitalWrite(led3,!digitalRead(led3));
      Blynk.virtualWrite(V3,digitalRead(led3));
      bt3_state=LOW;
      delay(200);
    }
  }else{
    bt3_state=HIGH;
  }
  if(digitalRead(button4)==LOW){
    if(bt4_state==HIGH){
      digitalWrite(led4,!digitalRead(led4));
      Blynk.virtualWrite(V4,digitalRead(led4));
      bt4_state=LOW;
      delay(200);
    }
  }else{
    bt4_state=HIGH;
  }
}
