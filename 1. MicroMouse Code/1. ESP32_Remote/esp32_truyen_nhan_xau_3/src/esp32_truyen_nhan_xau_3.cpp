// -------------Khai báo thư viện----------------------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

//--------------Cấu hình chân và khai báo các biến cần dùng-------------------------------------------------------

uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x45, 0x49, 0x90};

char coef[15]; // biến lưu các giá trị hệ số kp ki kd

char recvChar[15]; // biến lưu giá trị gửi từ Serial

boolean newData = false; // biến đánh dấu xem có dữ liệu mới từ Serial hay không

//-------------------------Hàm xóa dòng trước khi in-------------------------------------------------------

void clearPreviousLine()
{
  // Serial.write("\033[A"); // Di chuyển con trỏ lên 1 dòng
  Serial.print("\033[K"); // Xóa dòng hiện tại
  Serial.write("\033[A"); // Di chuyển con trỏ lên 1 dòng (để tránh hiện tượng nhấp nháy)
}

//--------------------------Struct để gửi dữ liệu--------------------------------------------------------------------
// cấu trúc phải phù hợp với bên nhận

typedef struct struct_message_send
{

  char coef[15];

} struct_message_send;

struct_message_send myDataSend; // tạo 1 biến để gửi dữ liệu

//-----------------------Callback khi dữ liệu được gửi--------------------------------------------------------------

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println("\r\nTrạng thái gửi gần nhất:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Thành công" : "Lỗi gửi");
}

//--------------------------Struct để nhận dữ liệu--------------------------------------------------------------------

typedef struct struct_message_receive
{

  char coef[15];

} struct_message_receive;

struct_message_receive myDataReceive; // tạo biến để nhận dữ liệu

//----------------Callback khi dữ liệu được nhận-----------------------------------------------------------------------

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataReceive, int len)
{
  memcpy(&myDataReceive, incomingDataReceive, sizeof(myDataReceive));

  // strcpy(coef, myDataReceive.coef);
  clearPreviousLine();

  Serial.print("Gia tri cua kp ki kd: ");
  Serial.println(myDataReceive.coef);
}

//---------------------Hàm setup-------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);


  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("Địa chỉ Mac của esp32:   ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Lỗi khởi tạo ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent); // Đăng ký callback khi dữ liệu được gửi

  esp_now_register_recv_cb(OnDataRecv); // Đăng ký callback khi dữ liệu được nhận

  esp_now_peer_info_t peerInfo;

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Không thể thêm thiết bị nhận");
    return;
  }

  Serial.println("ESP32 Sending");

  Serial.println("Nhap cac he so kp ki kd");
}

//-----------------------------Hàm loop----------------------------------------------------------------------------------

void loop()
{

  if (newData)
  {
    Serial.print(">>>>> ");
    Serial.println("Gửi dữ liệu");
    strcpy(myDataSend.coef, recvChar); // Sử dụng strcpy (xâu động) thay vì strncpy

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myDataSend, sizeof(myDataSend));

    if (result == ESP_OK)
    {
      Serial.println("Gửi thành công");
    }
    else
    {
      Serial.println("Lỗi gửi dữ liệu");
    }

    memset(recvChar, 0, sizeof(recvChar));
    memset(myDataSend.coef, 0, sizeof(myDataSend.coef));
    
  }

  newData = false;
}

//-----------------------------Hàm serialEvent---------------------------------------------------------------------------------------

void serialEvent()
{
  int index = 0;
  while (Serial.available())
  {
    char incomingChar = Serial.read();
    if (incomingChar != '\n' && incomingChar != '\r')
    {
      if (index < 16) // Check if index is within array bounds
      {
        recvChar[index++] = incomingChar;
        // index++; // Tăng chỉ mục để lưu trữ ký tự tiếp theo
        newData = true;
      }
    }
  }
  recvChar[index] = '\0'; // Kết thúc xâu
}
