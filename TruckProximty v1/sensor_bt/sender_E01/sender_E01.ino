/*
  ESP-NOW Remote Sensor - Transmitter (Multiple Version)
  esp-now-xmit-multiple.ino
  Sends Temperature & Humidity data to other ESP32 via ESP-NOW
  Uses DHT22
  Multiple Transmitter modification
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Include required libraries
#include <WiFi.h>
#include <esp_now.h>

// Integer for identification (make unique for each transmitter)
int ident = 1; // E01 //COM7|

// Responder MAC Address (Replace with your responders MAC Address)
uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x38, 0xA6, 0x4C};  //E03

// Define data structure
typedef struct sensor_msg {
  int id; // ID of sender
  int val;  // Sensor value
} sensor_msg;

// Create structured data object
sensor_msg myData;

// Variables for distance measured
int distance;

// Register peer
esp_now_peer_info_t peerInfo;

// Sent data callback function
void OnDataSent(const uint8_t *macAddr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Setup Serial monitor
  Serial.begin(115200);
  delay(100);

  // Set ESP32 WiFi mode to Station temporarly
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Define callback
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Define device ID
  myData.id = ident;
}

void loop() {

  // Update sensor value
  distance = (distance + 50) % 1000;

  Serial.print("Distance: ");
  Serial.println(distance);

  // Add to structured data object
  myData.val = distance;

  // Send data
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  // Delay for DHT22 sensor
  delay(500);
}
