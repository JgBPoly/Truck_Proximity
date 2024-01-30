#include <HardwareSerial.h>
#include <Arduino.h>

HardwareSerial SerialPort(2); // use UART2

// String BT_SPEAKERS = "TG-191";
String BT_SPEAKERS = "TG-191";
int N_SENSORS = 4;

void setup()  
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
  Serial.println("Master initialized.");
} 
void loop()  
{ 
  //for(int i=0; i<5; i++){
  String msg = ("<0," + String(N_SENSORS) + ',' + BT_SPEAKERS + '>');
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,1>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,2>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,3>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,4>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,5>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,6>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,7>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,8,8,8,8>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  msg = "<1,10,10,10,9>";
  Serial.println("Sending " + msg);
  SerialPort.print(msg);
  delay(2000); 
  //}
  //for(int i=0; i<5; i++){
  
  // Serial.println("Sending <1,8,8,8,0>");
  // SerialPort.print("<1,8,8,8,0>");
  // delay(500); 
  
  //}
}
