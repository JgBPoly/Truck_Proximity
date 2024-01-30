#include <HardwareSerial.h>
#define RX_PIN 16
#define TX_PIN 17
#define DIST_MIN 1
#define DIST_MAX 8
#define T_MIN 95
#define T_MAX 700

HardwareSerial SerialPort(2); // use UART2

const byte N_CHARS = 32;
const char SERIAL_MSG_OPEN_CHAR = '<';
const char SERIAL_MSG_CLOSE_CHAR = '>';
const char SERIAL_MSG_DELIMITER_PARSE_CHAR[] = {','};

const int CONFIG_ID = 0;
const int DATA_ID = 1;

char received_chars[N_CHARS]; // Serial comms variables
bool new_data = false;
uint16_t dt_alarm = 0; //ms, delay before next alarm
float dist = 100;
const float dist_to_t_coeff = (float)(T_MAX - T_MIN) / (float)(DIST_MAX - DIST_MIN);
int n_sensors = 4;
char bt_speaker_name[N_CHARS] = "TG-191";

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

void ParseData() // WARNING, this function can parse invalid messages (without tolerance & maybe more cases)
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
  }
  else if (atoi(strtokIndx) == CONFIG_ID){  //Detect msg type
    strtokIndx = strtok(NULL, SERIAL_MSG_DELIMITER_PARSE_CHAR);
    n_sensors = atoi(strtokIndx);
    strtokIndx = strtok(NULL, SERIAL_MSG_DELIMITER_PARSE_CHAR);
    strcpy(bt_speaker_name, strtokIndx);
  }
}


void setup() {
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
    Serial.println(received_chars);
    ParseData(); // updates variable dist
    dt_alarm = (int) (dist * dist_to_t_coeff);  // Convert dist into time
    Serial.println("Dist: " + String(dist,1) + "      DT: " + dt_alarm);
    new_data = false;
  }
}
