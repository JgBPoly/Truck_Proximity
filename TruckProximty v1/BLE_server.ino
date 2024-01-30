/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "3dc109c8-75eb-4eb5-953f-e168d2ef086c"
#define CHARACTERISTIC_UUID "44567dfe-508b-11ed-bdc3-0242ac120002"

bool deviceConnected = false;
bool oldDeviceConnected = false;

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
      std::string DistanceState = pMyCharacDisCap->getValue(); 
      Serial.println(DistanceState[0]);    
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("Capteur 1");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pService->addCharacteristic(&Characteristic);

  Characteristic.setValue("10");
  Characteristic.setCallbacks(new MyCallbacksDistanceCaptor());
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  // put your main code here, to run repeatedly:
  Characteristic.setValue("0");// Here to change the value of the distance that it's sent to the app, 0 corresponds to 20 cm because it's a percentage
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("10");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("20");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("30");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("40");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("50");// Here to change the value of the distance that it's sent to the app, 50 corresponds to 10 cm because it's a percentage
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("60");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("70");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("80");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("90");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
  Characteristic.setValue("100");// Here to change the value of the distance that it's sent to the app
  Characteristic.notify(); // To send the value to the app
  delay(1000);
}
