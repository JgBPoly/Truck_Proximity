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

// ESP32 MAC Adress: E8:31:CD:FC:97:34

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

////// Sleep 
#define mS_TO_S_FACTOR 1000     /* Conversion factor for milli seconds to seconds */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define T_AUTO_SLEEPMODE  10        /* Time ESP32 will go to sleep (in seconds) */
#define T_SLEEPMODE_SLEEP  2    // Time sleeping  during Sleep Mode (in seconds)
#define T_SLEEPMODE_WAKEUP  0.5   // Time awake     during Sleep Mode (in seconds)

unsigned long previousMillis = 0;
unsigned long interval = 10000;
// const unsigned long DT_AUTO_SLEEP = 5 * 1000;// 5 sec    * 60 ; //5 minutes

// bool msg_go_sleep = false;
// RTC_DATA_ATTR bool msg_go_sleep = false;
RTC_DATA_ATTR bool sleep_mode_state = false;

// int flag = 0;

const unsigned int MAX_MESSAGE_LENGTH = 12;



void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  sleep_mode_state = false;
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
  
  // Postpone sleep mode
  Serial.println("--> Continuing measurements for " + String(T_AUTO_SLEEPMODE) + "s");
  previousMillis = millis(); // Postepone sleep mode

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataSensor, sizeof(myDataSensor));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  esp_now_del_peer(peerInfo.peer_addr);
  delay(30);
  
  // if (myDataMaster.type == MSG_GO_SLEEP)
  // {
  //   Serial.println("Master request: Entering SLEEP MODE");
  //   sleep_mode_state = true;
  //   //esp_deep_sleep_start();
  //   //Serial.println("This will never be printed");
  // }
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

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.println("\n\n");
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void readSerial(){
  //Check to see if anything is available in the serial receive buffer
  while (Serial.available() > 0)
  {
    //Create a place to hold the incoming message
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();

    //Message coming in (check not terminating character) and guard for over message size
    if ( inByte != '\n' && inByte != '.' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
    {
      //Add the incoming byte to our message
      message[message_pos] = inByte;
      message_pos++;
    }
    //Full message received...
    else
    {
      //Add null character to string
      message[message_pos] = '\0';

      //Print the message (or do other things)
      Serial.println("Serial msg received: " + String(message));
      // if (String(message) == String("WAKEUP")){
        Serial.println("--> Continuing measurements for " + String(T_AUTO_SLEEPMODE) + "s");
        previousMillis = millis(); // Postepone sleep mode
        sleep_mode_state = false;
      // }
      //Reset for the next message
      message_pos = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Set ESP32 WiFi mode to Station temporarly
  WiFi.mode(WIFI_STA);
  btStop();

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("\n\nError initializing ESP-NOW");
    return; // Should reboot instead
  }

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
//  pinMode(BUTTON_PIN, INPUT_PULLUP);// initialize the pushbutton pin as an pull-up input

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  Serial.println("Sleep Mode state: " + String(sleep_mode_state));
  // Set up RTC timer for sleep
  esp_sleep_enable_timer_wakeup(T_SLEEPMODE_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 SLEEP MODE for " + String(T_SLEEPMODE_SLEEP) +
  "s sleeping, and "+ String(T_SLEEPMODE_WAKEUP) + "s awake.");

  esp_now_register_recv_cb(OnDataRecv);//Agit comme une interruption lance la fonction dès la reception d'un message
  
  Serial.println("Sensor initialized.");
}


void loop() {
  readSerial();

  // If Sleep Mode ON & device wake up more than T_SLEEPMODE_WAKEUP sec
  unsigned long currentMillis = millis();
  if(sleep_mode_state && (currentMillis > previousMillis + T_SLEEPMODE_WAKEUP * mS_TO_S_FACTOR))
  {
    previousMillis = currentMillis;
    Serial.println("--> Going to sleep for " + String(T_SLEEPMODE_SLEEP) + "s...");
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }

  currentMillis = millis();
  if (currentMillis > previousMillis + T_AUTO_SLEEPMODE * mS_TO_S_FACTOR) { // after 5 secs
    previousMillis = currentMillis;
    sleep_mode_state = true;
    Serial.println("AUTO: Entering Sleep Mode after " + String(T_AUTO_SLEEPMODE) + "s");
    //esp_deep_sleep_start();
  }    
  
/*
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

*/
 
}



/*
Simple Deep Sleep with Timer Wake Up
=====================================
*/

/*
void setup(){
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(T_AUTO_SLEEPMODE * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(T_AUTO_SLEEPMODE) +
  " Seconds");
  
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}
*/


