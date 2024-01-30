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
#include <math.h> 

#define DEBUG 1   // 1: Print serial, 0: Don't print 
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#endif


#define c3_frequency  1000 //130.81
#define VOLUME_DEFAULT 30
#define T_ALARM 100 // Alarm duration
bool flag_volume = false; // flag to toggle volume
unsigned long currentTime = 0;
unsigned long lastTime = 0;

int dt_alarm = 500; //ms, delay before next alarm

BluetoothA2DPSource a2dp_source;
const char* BT_SPEAKER = "TG-191";

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


void setup() {
  //a2dp_source.set_auto_reconnect(false);
  a2dp_source.start(BT_SPEAKER, get_data_channels);  
  a2dp_source.set_volume(VOLUME_DEFAULT);
  
  // Setup serial com
  Serial.begin(115200);
  debugln("Init completed.");
}

void loop() {
    
  currentTime=millis();

  if((currentTime-lastTime) > dt_alarm && !flag_volume){   //Detect turn ON
    lastTime=currentTime;
    flag_volume = true;
    debugln("ON: " + String(currentTime/1000));
    a2dp_source.set_volume(VOLUME_DEFAULT);
    
    dt_alarm += 20;
  }
  else if((currentTime-lastTime) > T_ALARM && flag_volume){ //Detect turn OFF 
    // lastTime=currentTime;
    flag_volume = false;
    debugln("OFF: " + String(currentTime/1000));
    if(dt_alarm > T_ALARM)
      a2dp_source.set_volume(0);
  }

  if (dt_alarm >=500) dt_alarm = 0;
  
  // to prevent watchdog in release > 1.0.6
  delay(10); // 10
}
