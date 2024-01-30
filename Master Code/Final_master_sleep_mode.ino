///// SLEEP
//EEPROM
#include <EEPROM.h>
#define EEPROM_SIZE 13
//ESP_NOW 
#include <WiFi.h>
#include <esp_now.h>

//ESPBLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

//#include <Arduino.h>

// Define data structure
typedef struct sensor_msg {
  int id; // ID of sender
  int val; // Sensor value
  int batteryvalue; //Battery value
} sensor_msg;

typedef struct master_msg {
  uint8_t macAddr[6]; // Mac Address
  int id; // ID of sender
} master_msg;


// Sent data callback function
void OnDataSent(const uint8_t *macAddr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Create structured data object
sensor_msg myData;
master_msg myDatar;

uint8_t myMac[] = {0xE8,0x31,0xCD,0xFC,0x98,0x60};//E8:31:CD:FC:98:60 E04
uint8_t broadcastAddress[] = {0,0,0,0,0,0};
uint8_t broadcastAddress1[] = {0xAC,0x67,0xB2,0x36,0x53,0xDC};

// Register peer
esp_now_peer_info_t peerInfo;

int i;
int j=0;
int nbcapteur = 1;
String Message_app;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "3dc109c8-75eb-4eb5-953f-e168d2ef086c"
#define CHARACTERISTIC_UUID "44567dfe-508b-11ed-bdc3-0242ac120002"

bool deviceConnected = false;
bool oldDeviceConnected = false;

bool sensor_sleeping = false; // By default
bool sensor_msg_rcv = false;

//DMTimer timer_wakeup_sensor(500000); // 0.5s en microsecondes
unsigned long previous_time = 0;
unsigned long current_time;
int i_try = 0;

String strToString(std::string str) {
  return str.c_str();
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    sensor_sleeping = false;
    sensor_msg_rcv = true;
    Serial.println("Sensor msg received");
    memcpy(&myData, incomingData, sizeof(myData));
    Message_app+=String(myData.id)+":"+String(myData.val)+",";
    j++;
}

BLECharacteristic Characteristic (
                                   CHARACTERISTIC_UUID,
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_WRITE
                                   );
                   
                                       
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected!");
      pServer->getAdvertising()->start();
    }



};
class MyCallbacksDistanceCaptor: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic * pMyCharacDisCap) {
      std::string rxValue = pMyCharacDisCap->getValue();
      String msg = strToString(rxValue);
      Serial.println(msg);
      nbcapteur = msg.substring(0).toInt();
      Message_app ="";
      Serial.println(nbcapteur);
      i = 0;
      j = 0;
      Serial.println("App msg received.");
      sensor_sleeping = true;
      while (i<nbcapteur) 
      {
        //Message du type : 3_1:ABCDEFGHIJKL2:ABCDEFGHIJKL3:ABCDEFGHIJKL4:ABCDEFGHIJKL
          memcpy(myDatar.macAddr, myMac, 6);
          myDatar.id = msg.substring(3+(i*14)).toInt();
          String baddres = msg.substring(4+i*14, 2+(i+1)*14);
          Serial.println("flag1");
          Serial.println(baddres);
          for (int j=0; j<6;j++){
            int str_len = 3; 
            // Prepare the character array (the buffer) 
            char char_array[3];
            String SEC = baddres.substring(0+(j*2),2+(j*2));
            // Copy it over 
            SEC.toCharArray(char_array, str_len);
            broadcastAddress[j] = strtoul(char_array, NULL, 16 );//("0x"+baddres.substring(0+(j*2),1+(j*2))).toInt();  
//            Serial.println(broadcastAddress[j]);
//            Serial.println(SEC);
          }
          Serial.println("flag2");
          memcpy(peerInfo.peer_addr, broadcastAddress, 6);
          peerInfo.channel = 0;  
          peerInfo.encrypt = false;
          if (esp_now_add_peer(&peerInfo) != ESP_OK){
            Serial.println("Failed to add peer");
            return;
          }

          // Make sure sensor received rqt
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDatar, sizeof(myDatar));
          if (result == ESP_OK) {
            Serial.println("--> Sent with success.");
          }

//          else {
//            Serial.println("Error sending the data");
//          }                    
          esp_now_del_peer(peerInfo.peer_addr);
          i++;
      }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    
    // Set ESP32 WiFi mode to Station temporarly
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != 0) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    BLEDevice::init("Master Pilot");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pService->addCharacteristic(&Characteristic);
    
    Characteristic.setCallbacks(new MyCallbacksDistanceCaptor());
    Characteristic.setValue("start");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone !");

    esp_now_register_recv_cb(OnDataRecv);//Agit comme une interruption lance la fonction dès la reception d'un message
}


void loop() 
{
  if(j>=nbcapteur){

    //Characteristic.setValue(&Message_app);
    Serial.println(Message_app);
    Serial.println("Value send to phone");
    j=0;
    nbcapteur=1;
  }

//  i_try = 1;  
  if (sensor_sleeping){ // Wait a response from sensor
    current_time = millis();
    
    float dt_try = 0.5; //sec             <-------------------------------------------------------------
    
    if (current_time > previous_time + dt_try*1000) {
      previous_time = current_time;
      
      memcpy(peerInfo.peer_addr, broadcastAddress, 6);
      peerInfo.channel = 0;  
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
      }
      
      Serial.print("Error sending rqt to: ");
      for(int k = 0; k < 6; k++) {Serial.print(broadcastAddress[k], HEX); Serial.print(" ");} Serial.println();
      i_try++;
      Serial.println(String(i_try) + " retry in " + String(dt_try) + "s");
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDatar, sizeof(myDatar));
//      delay(30);
      esp_now_del_peer(peerInfo.peer_addr);
    }
  }
  if (sensor_msg_rcv){
    Serial.println("--> Rcv after " + String(i_try) + " attempts.");
    i_try = 0;
    sensor_msg_rcv = false;
  }
}
