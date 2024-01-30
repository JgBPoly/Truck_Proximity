/*
  Streaming of sound data with Bluetooth to other Bluetooth device.
  We generate 2 tones which will be sent to the 2 channels.
  
  Copyright (C) 2020 Phil Schatzmann
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "BluetoothA2DPSource.h"
#include <HardwareSerial.h>
#include <math.h> 
#include <Arduino.h>

#define RX_PIN 16
#define TX_PIN 17
#define DIST_MIN 1
#define DIST_MAX 8
#define T_MIN 95
#define T_MAX 700

#define c3_frequency  1000 //130.81
#define VOLUME_DEFAULT 30
#define T_ALARM 100 // Alarm duration

HardwareSerial SerialPort(2); // use UART2

const byte N_CHARS = 32;
const char SERIAL_MSG_OPEN_CHAR = '<';
const char SERIAL_MSG_CLOSE_CHAR = '>';
const char SERIAL_MSG_DELIMITER_PARSE_CHAR[] = {','};
const int CONFIG_ID = 0;
const int DATA_ID = 1;

char received_chars[N_CHARS]; // Serial comms variables
bool new_data = false;
  
bool flag_volume = false; // flag to toggle volume
unsigned long currentTime = 0;
unsigned long lastTime = 0;

uint16_t dt_alarm = 0; //ms, delay before next alarm
float dist = 100;
float old_dist = 100;
const float dist_to_t_coeff = (float)(T_MAX - T_MIN) / (float)(DIST_MAX - DIST_MIN);

BluetoothA2DPSource a2dp_source;
//char bt_speaker_name[N_CHARS] = "TG-191";
//int n_sensors = 4;
char bt_speaker_name[N_CHARS] = "AAA";
int n_sensors = 1;

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data
int32_t get_data_channels(Frame *frame, int32_t channel_len) {
    static double m_time = 0.0;
    double m_amplitude = 10000.0;  // -32,768 to 32,767
    double m_deltaTime = 1.0 / 44100.0;
    double m_phase = 0.0;
    double double_Pi = PI * 2.0;
    // fill the channel data
    for (int sample = 0; sample < channel_len; ++sample) {
        double angle = double_Pi * c3_frequency * m_time + m_phase;
        frame[sample].channel1 = m_amplitude * sin(angle);
        frame[sample].channel2 = frame[sample].channel1;
        m_time += m_deltaTime;
    }
    return channel_len;
}

// Gets a message sent over serial communication
void RecvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char rc;

  while (SerialPort.available() > 0 && new_data == false)
  {
    rc = SerialPort.read();
    if (recvInProgress == true)
    {
      if (rc != SERIAL_MSG_CLOSE_CHAR)
      {
        received_chars[ndx] = rc;
        ndx++;
        if (ndx >= N_CHARS)
          ndx = N_CHARS - 1;
      }
      else
      {
        received_chars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        new_data = true;
      }
    }

    else if (rc == SERIAL_MSG_OPEN_CHAR)
      recvInProgress = true;
  }
}

int ParseData() // WARNING, this function can parse invalid messages (without tolerance & maybe more cases)
{
  // split the data into its parts
  char *strtokIndx;   // this is used by strtok() as an index                                             // this is used by strtok() as an index
  char tempChars[N_CHARS];
  strcpy(tempChars, received_chars);
  // this copy is necessary to protect the original data
  // because strtok() used in ParseData() replaces the commas with \0  
  
  dist = 100;
  strtokIndx = strtok(tempChars, SERIAL_MSG_DELIMITER_PARSE_CHAR); // get the first part - the string
  if(atoi(strtokIndx) == DATA_ID){  //Detect msg type
    for(int i=0; i<n_sensors; i++)
    {
      strtokIndx = strtok(NULL, SERIAL_MSG_DELIMITER_PARSE_CHAR);      // this continues where the previous call left off
      if(dist > atof(strtokIndx)) dist = atof(strtokIndx);  // Parse only min value
    }
    return DATA_ID;
  }
  else if (atoi(strtokIndx) == CONFIG_ID){  //Detect msg type
    strtokIndx = strtok(NULL, SERIAL_MSG_DELIMITER_PARSE_CHAR);
    n_sensors = atoi(strtokIndx);
    strtokIndx = strtok(NULL, SERIAL_MSG_DELIMITER_PARSE_CHAR);
    strcpy(bt_speaker_name, strtokIndx);
    return CONFIG_ID;
  }
}


void setup() {
  //a2dp_source.set_auto_reconnect(false);
  a2dp_source.start(bt_speaker_name, get_data_channels);  
  a2dp_source.set_volume(0);
  
  // Setup serial com
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(115200);
  Serial.println("Init completed.");
}

void loop() {
  // Read & Parse
  RecvWithStartEndMarkers();
  if (new_data == true)
  {
    Serial.println("Received: " + String(received_chars));
    int msg_type = ParseData(); // updates variable dist
    new_data = false;
    if(msg_type == DATA_ID)
      dt_alarm = (int) (dist * dist_to_t_coeff);  // Convert dist into time
    // else if (msg_type == CONFIG_ID)
      
  }

  // Send BT Audio  
  if (dist <= DIST_MAX || flag_volume)  // Only if we have to
  {
    if(old_dist != dist){
      old_dist = dist;
      Serial.println("D: " + String(dist,1) + " cm;   T: " + dt_alarm + " ms");
    }
    currentTime=millis();
    if((currentTime-lastTime) > dt_alarm && !flag_volume){   //Detect turn ON
      lastTime=currentTime;
      flag_volume = true;
      // Serial.println("ON: " + String(currentTime/1000));
      a2dp_source.set_volume(VOLUME_DEFAULT);
    }
    else if((currentTime-lastTime) > T_ALARM && flag_volume){ //Detect turn OFF 
      // lastTime=currentTime;
      flag_volume = false;
      // Serial.println("OFF: " + String(currentTime/1000));
      if(dt_alarm > T_ALARM)
        a2dp_source.set_volume(0);
    }
  }
    
  // to prevent watchdog in release > 1.0.6
  delay(10); // 10
}
