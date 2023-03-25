// Sending/Receiving example
#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Ultrasonic.h"

#define ULTRASONIC1 14
#define ULTRASONIC2 15

Ultrasonic ultrasonic1(ULTRASONIC1);
Ultrasonic ultrasonic2(ULTRASONIC2);

long RangeInCentimeters1 = 0;
long RangeInCentimeters2 = 0;

int plastic_percent;
int metal_percent;

int fake_plastic = 0;
int fake_metal = 0;
bool isSend = false;
bool isSendfull = false;


//TaskHandle_t TaskA = NULL;

HardwareSerial Sender(1);   // Define a Serial port instance called 'Sender' using serial port 1

#define Sender_Txd_pin 17
#define Sender_Rxd_pin 16

uint8_t NinaAddress[] = {0x3C, 0x61, 0x05, 0x03, 0x42, 0x70};
uint8_t FaiiAddress[] = {0x24, 0x6F, 0x28, 0x25, 0x86, 0xDC};

typedef struct state_coin { 
  int state;
  int state_1;
  int state_5;
}state_coin;

typedef struct percent_tsh {
  int state;
  int plastic;
  int metal;
} percent_tsh;

percent_tsh recv_percent_tsh;
percent_tsh send_percent_tsh;
state_coin stc;
esp_now_peer_info_t peerInfo1;
esp_now_peer_info_t peerInfo2;

// bool compareMac(const uint8_t * a,const uint8_t * b){
//   for(int i=0;i<6;i++){
//     if(a[i]!=b[i])
//       return false;    
//   }
//   return true;
// }

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  if (mac_addr[0] == 0x24 && mac_addr[1] == 0x6F && mac_addr[2] == 0x28 && mac_addr[3] == 0x25 && mac_addr[4] == 0x86 && mac_addr[5] == 0xDC){
    memcpy(&stc, incomingData, sizeof(stc));

    if(stc.state == 12){
      if(stc.state_1 == 0 && stc.state_5 == 1){
        Sender.print(1);    
      }

      else if(stc.state_1 == 1 && stc.state_5 == 0){
        Sender.print(2);    
      }

      else if(stc.state_1 == 0 && stc.state_5 == 0){
        Sender.print(3);    
      }
    }
  }

  else if (mac_addr[0] == 0x3C && mac_addr[1] == 0x61 && mac_addr[2] == 0x05 && mac_addr[3] == 0x03 && mac_addr[4] == 0x42 && mac_addr[5] == 0x70){
    memcpy(&recv_percent_tsh, incomingData, sizeof(recv_percent_tsh));

    if(recv_percent_tsh.state == 14){
      send_percent_tsh.state = 0;
      send_percent_tsh.plastic = plastic_percent;
      send_percent_tsh.metal = metal_percent;

      esp_err_t result = esp_now_send(NinaAddress, (uint8_t *) &send_percent_tsh, sizeof(send_percent_tsh));
        if (result == ESP_OK) {
            Serial.println("Sent with success");
        }
        else {
            Serial.println("Error sending the data");
        }
    }
  }
} 

void setup() {
  Serial.begin(115200);                                             // Define and start serial monitor
  Sender.begin(115200, SERIAL_8N1, Sender_Txd_pin, Sender_Rxd_pin); // Define and start Sender serial port
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo1.peer_addr, FaiiAddress, 6);
  peerInfo1.channel = 0;  
  peerInfo1.encrypt = false;

  memcpy(peerInfo2.peer_addr, NinaAddress, 6);
  peerInfo2.channel = 0;  
  peerInfo2.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo1) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if (esp_now_add_peer(&peerInfo2) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //xTaskCreatePinnedToCore(check_trash, "check_trash", 2048, NULL, 1, &TaskA, 0);
}

void loop() {
  RangeInCentimeters1 = ultrasonic1.MeasureInCentimeters();
  RangeInCentimeters2 = ultrasonic2.MeasureInCentimeters();

  printf("plastic: %d\n",RangeInCentimeters1);
  printf("metal: %d\n",RangeInCentimeters2);


  plastic_percent = (41 - int(RangeInCentimeters1))*100/41;
  metal_percent = (41    - int(RangeInCentimeters2))*100/41;

  printf("plastic percent: %d\n",plastic_percent);
  printf("metal percent: %d\n",metal_percent);

  if(plastic_percent <= 0 && metal_percent <= 0){
    if(!isSend){
      send_percent_tsh.state = 0;
      send_percent_tsh.plastic = 0;
      send_percent_tsh.metal = 0;
      isSend = true;
      Sender.print(4);
      esp_err_t result = esp_now_send(NinaAddress, (uint8_t *) &send_percent_tsh, sizeof(send_percent_tsh));
        if (result == ESP_OK) {
            Serial.println("Sent with success");
        }
        else {
            Serial.println("Error sending the data");
        }
    }else if (isSendfull){
      isSendfull = false;
    }
  }
  else{
    isSend = false;
  }

  if(plastic_percent >= 85 && !isSendfull){
    // plastic + 100
    fake_plastic = 100 + plastic_percent;
    Sender.print(fake_plastic);
    Serial.printf("send fake plastic: ");
    Serial.println(fake_plastic);
    isSendfull = true;
  }
  if(metal_percent >= 85 && !isSendfull){
    fake_metal = 500 + metal_percent;
    Sender.print(fake_metal);
    Serial.printf("send fake metal: ");
    Serial.println(fake_metal);
    isSendfull = true;
  }

  delay(1000);
}

// void check_trash(void *param){
//   while(1){
//     RangeInCentimeters1 = ultrasonic1.MeasureInCentimeters();
//     RangeInCentimeters2 = ultrasonic2.MeasureInCentimeters();

//     Serial.println(RangeInCentimeters1);
//     Serial.println(RangeInCentimeters2);

//     plastic_percent = int(RangeInCentimeters1);
//     metal_percent = int(RangeInCentimeters2);
//   }
// }