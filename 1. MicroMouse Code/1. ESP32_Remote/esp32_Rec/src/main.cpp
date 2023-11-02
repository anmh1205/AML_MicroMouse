#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

#define resetSTM32Pin 18

int i = -1;

uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x57, 0x8C, 0x74};
; // Địa chỉ ESP32 trên tay điều khiển

// Define variables to store incoming readings
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  for (int i = 0; i < 23; i++)
  // while (i++ < 23)
  {
    Serial.print((char)*(incomingData + i));
  }

  if (*(incomingData + 20) == '1')
  {
    digitalWrite(resetSTM32Pin, LOW);
    delay(10);
    digitalWrite(resetSTM32Pin, HIGH);
  }

  // i = -1;
}

void setup()
{

  Serial.begin(38400);

  pinMode(resetSTM32Pin, OUTPUT);
  digitalWrite(resetSTM32Pin, HIGH);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
}
