//EEPROM
#include <EEPROM.h>
#define EEPROM_SIZE 13

//ESP_NOW
#include <WiFi.h>
#include <esp_now.h>

unsigned long lastTime;
#define TIME_SEND 1000000
int ID = 1;

const int trigPin = 4;
const int echoPin = 2;

//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701

float distanceCm;
int state = 0;

int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin

// Responder MAC Address
uint8_t broadcastAddress[6];


// Define data structure
typedef struct sensor_msg {
  int id; // ID of sender
  int val; // Sensor value
  int batteryvalue; //Battery value
} sensor_msg;

typedef struct master_msg {
  uint8_t macAddr[6]; // Mac Address
  int id; // ID of sensor
} master_msg;

// Sent data callback function
void OnDataSent(const uint8_t *macAddr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// Create structured data object
sensor_msg myDataSensor;
master_msg myDataMaster;

// Register peer
esp_now_peer_info_t peerInfo;

//To non bloquant approach
unsigned long previousMillis = 0;
unsigned long interval = 10000;

int flag = 0;



void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Received from Master: ");
  for (int i=0; i<6; i++) {Serial.print(mac[i], HEX); Serial.print(" ");} Serial.println();
  memcpy(&myDataMaster, incomingData, sizeof(myDataMaster));
  
  // memcpy(peerInfo.peer_addr, myDataMaster.macAddr, 6);
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(broadcastAddress, mac, 6);
  myDataSensor.id = ID;
  myDataSensor.val = int(getDistance());
  myDataSensor.batteryvalue = int(getBatteryVoltage());
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataSensor, sizeof(myDataSensor));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  esp_now_del_peer(peerInfo.peer_addr);
}

float getDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  float distanceCm = duration * SOUND_VELOCITY / 2;

  return distanceCm;
}

int getBatteryVoltage(){
  return 0;
}

void setup() {
  
  Serial.begin(115200);
  // Set ESP32 WiFi mode to Station temporarly
  WiFi.mode(WIFI_STA);
  btStop();

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
//  pinMode(BUTTON_PIN, INPUT_PULLUP);// initialize the pushbutton pin as an pull-up input

  Serial.println("Sensor initialized.");

  esp_now_register_recv_cb(OnDataRecv);//Agit comme une interruption lance la fonction dès la reception d'un message

  
  
}





void loop() {
  
  
//  //To read the actual time
//  //Non bloquant approach
//  //Every half sec, it will activate the sleep mode
//
//  if(flag == 1){
//    WiFi.setSleep(true);
//    if (!setCpuFrequencyMhz(3)){
//        Serial2.println("Not valid frequency!");
//    }
//    Serial.println("Sleep");
//    flag = 2;
//  }
//  else if(flag == 0){
//    if (!setCpuFrequencyMhz(80)){
//        Serial2.println("Not valid frequency!");
//    }
//    WiFi.setSleep(false);
//      if (esp_now_init() != 0) {
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }
//
//    flag = 3;
//    Serial.println("Wake");
//  }
//
//  unsigned long currentMillis = millis();
//  if (currentMillis > previousMillis + interval) {
//    previousMillis = currentMillis;
//    if(flag == 2){
//      flag = 0;
//    }
//    else if(flag == 3) {
//      flag = 1;
//    }
//  }
    

//  delay(2000);
//  // put your main code here, to run repeatedly:
//  // Enable timer wakeup for 500ms
//  esp_sleep_enable_timer_wakeup(2000000);
//
//  // Go to sleep
//  esp_light_sleep_start();
//
//  // Normal operation
//  delay(2000);
 
}
