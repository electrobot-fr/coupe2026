
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

#undef LED_BUILTIN
#define LED_BUILTIN 8

#define BUFFER_SIZE 250 // max of 250 bytes
uint8_t buf_recv[BUFFER_SIZE];

uint8_t emetteurAddress[] = {0x88, 0x13, 0xbf, 0x24, 0x87, 0x20};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * macAddr, const uint8_t *incomingData, int len) {
  if (macAddr[0] != emetteurAddress[0] ||
      macAddr[1] != emetteurAddress[1] ||
      macAddr[2] != emetteurAddress[2] ||
      macAddr[3] != emetteurAddress[3] ||
      macAddr[4] != emetteurAddress[4] ||
      macAddr[5] != emetteurAddress[5]) {
    return; // drop packets from other devices
  }
  memcpy(&buf_recv, incomingData, sizeof(buf_recv));
  digitalWrite(LED_BUILTIN, LOW);
  // Serial.write(buf_recv, len);
  Serial1.write(buf_recv, len);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // // Initialize Serial Monitor
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX, TX);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(40); 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
}
