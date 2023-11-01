#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x45, 0x49, 0x90}; // Địa chỉ MAC của ESP32 trên Robot

int i = 0;
char buffer;
char Readings[24];

esp_now_peer_info_t peerInfo;

// Hàm được gọi khi dữ liệu được gửi
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  status == ESP_NOW_SEND_SUCCESS;

  // Kiểm tra gửi có thành công hay không
  if (status == ESP_OK)
  {
    Serial.println("ok");
  }
}

// Hàm được gọi khi nhận được dữ liệu
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(38400);

  // Cài đặt thiết bị ở chế độ Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    // Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}


void loop()
{
  while (Serial2.available())
  {   
    buffer = Serial2.read();
    Readings[i++] = buffer;
    if (buffer == '\0')
    {
      Readings[i] = '\0';
      esp_now_send(broadcastAddress, (uint8_t *)&Readings, 23);

      // for (int j = 0; j < 23; j++)
      // {
      //   Serial.print(Readings[j]);
      // }
      // Serial.println();
      
      i = 0;
      return;
    }
  }
}