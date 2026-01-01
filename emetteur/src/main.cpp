// Source: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
// MAC = 88:13:bf:24:87:20 (telecommande)
// 58:8c:81:9e:aa:48 (supermini)
// 58:8c:81:9e:c0:20 (ecureuil)
// 08:a6:f7:6b:2f:90 (bandeau led)

#include <esp_now.h>
#include <WiFi.h>

#define LED_BUILTIN 2

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x58, 0x8c, 0x81, 0x9e, 0xaa, 0x48};

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
 
#define TX_PIN     21 // default UART0 is pin 1 (shared by USB)
#define RX_PIN     19 // default UART0 is pin 3 (shared by USB)
#define BAUD_RATE  115200
#define SER_PARAMS SERIAL_8N1

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial2.begin(BAUD_RATE, SER_PARAMS, RX_PIN, TX_PIN);
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
#define BUFFER_SIZE 250 // max of 250 bytes
const uint32_t timeout_micros = (int)(1.0 / BAUD_RATE * 1E6) * 20;
uint8_t buf_recv[BUFFER_SIZE];
uint8_t buf_send[BUFFER_SIZE];
uint8_t buf_size = 0;
uint32_t send_timeout = 0;

void loop() {
  if (Serial2.available()) {
    while (Serial2.available() && buf_size < BUFFER_SIZE) {
      buf_send[buf_size] = Serial2.read();
      send_timeout = micros() + timeout_micros;
      buf_size++;
    }
  }
  if (buf_size == BUFFER_SIZE || (buf_size > 0 && micros() >= send_timeout)) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &buf_send, buf_size);
    buf_size = 0;
  }
}
