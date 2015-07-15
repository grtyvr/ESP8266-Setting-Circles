/*
 Based on: 
 Chat  Server

 created 18 Dec 2009
 by David A. Mellis
 modified 31 May 2012
 by Tom Igoe

 Modified to accept connections from SkySafari over the network and send back angle data
 from AS5048 angle sensors

 July 5, 2015
 Added running as a WiFi Access point
 Added code to read an AS5048 

 July 6, 2015
 Added parity calculations to the commands

 July 14, 2015
 Added better code to calculate a running average with throwing out the high and low 15% of values
 Added a structure to hold the two sensor readings 
 
 July 15, 2015
 Refactored to make functions for the AS5048A calls (in progress)
 
 */

#include <SPI.h>
#include <ESP8266WiFi.h>

// Define this if you want to run as an Access Point.  If undefined it will connect to the 
// SSID with the password below....

#define AP 

char ssid[] = "braapppp";     //  your network SSID (name)
char pass[] = "";             // your network password (use for WPA, or use as key for WEP)

const char *apssid = "ESPap";
const char *appassword = "gofish";

int keyIndex = 0;             // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;


// These defines are for the AS5048 
// for example...
// http://ams.com/eng/Support/Demoboards/Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5048A-Adapterboard  

#define AS5048_CMD_READ 0x4000  // flag indicating read attement
#define AS5048_CMD_WRITE 0x8000 // flag indicating write attemept
#define AS5048_REG_AGC 0x3FFD   // agc and diagnostics register ( 0 = high, 255 = low)
                                // bit 0 to 7 are agc
                                // bit 8 = 1 Offset Compensation Finished
                                // bit 9 = 0 Cordic OverFlow  (output keeps last valid value
                                // bit 10 = 1 low magnetic field
                                // bit 11 = 1 high magnetic field
#define AS5048_REG_MAG 0x3FFE   // magnitude information after ATAN calculation
#define AS5048_REG_DATA 0x3FFF  // angle data
#define AS5048_REG_ERR 0x1      // error register bit 0 = framing error, bit 1 = invalid command, bit 2 = parity error
#define AS5048_CMD_NOP 0x0      // dummy operation
#define AS5048_NUM_SENSORS 2    // number of sensors in daisy chain


// for smoothing sensor data
#define samplesNumToAverage  20
unsigned int smoothAzimuthValue[samplesNumToAverage];
unsigned int smoothAltitudeValue[samplesNumToAverage];


int ssl=15;
byte cmd_highbyte = 0;
byte cmd_lowbyte = 0;
byte alt_data_highbyte = 0;
byte alt_data_lowbyte = 0;
byte azt_data_highbyte = 0;
byte azt_data_lowbyte = 0;
word command = 0;
unsigned int data = 0;
word smoothAzimuthData = 0;
word smoothAltitudeData = 0;
unsigned int value = 0;
unsigned int altRawData[3];
unsigned int aztRawData[3];
byte altAGC;
bute aztAGC;
unsigned int magnitude;
byte altOCF;
byte aztCOF;
byte altAlarmHigh;
byte altAlarmLow;
byte aztAlarmHigh;
byte aztAlarmLow;
float angle = 0;
int i = 0;
int del = 10;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  //Initialize serial
  Serial.begin(115200);
  
  #ifdef AP
    Serial.println("Setting up WiFi Access Point");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apssid);
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
  #else 
    // attempt to connect to Wifi network:  
    while ( status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);

      // wait 10 seconds for connection:
      delay(5000);
    }
  #endif

  // start the server:
  server.begin();
  // you're connected now, so print out the status:
  #ifdef AP
  #else
    printWifiStatus();
  #endif

  pinMode(ssl, OUTPUT);
  SPI.begin();                                                        // Wake up the bus
  SPI.setBitOrder(MSBFIRST);                                          // AS5048 is a Most Significant Bit first
  SPI.setDataMode(SPI_MODE1);                                         // AS5048 uses Mode 1
  command = AS5048_CMD_READ | AS5048_REG_DATA;                        // Set up the command we will send
  command |= calcEvenParity(command) <<15;                                 // assign the parity bit
  cmd_highbyte = highByte(command);                                   // split it into bytes
  cmd_lowbyte = lowByte(command);                                     //
  // initialize our smoothing array with data
  // first time has dummy data
  digitalWrite(ssl, LOW);                                             // Drop ssl to enable the AS5048's
  alt_data_highbyte = SPI.transfer(cmd_highbyte);                     // send the initial read command
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);
  azt_data_highbyte = SPI.transfer(cmd_highbyte);                     // send the second read command
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);
  digitalWrite(ssl, HIGH);                                            // disable the AS5048's
  for ( i = 0; i <= samplesNumToAverage + 1; i++ ) {
    digitalWrite(ssl, LOW);                                             // Drop ssl to enable the AS5048's
    alt_data_highbyte = SPI.transfer(cmd_highbyte);                     // send the initial read command
    alt_data_lowbyte = SPI.transfer(cmd_lowbyte);
    azt_data_highbyte = SPI.transfer(cmd_highbyte);                     // send the second read command
    azt_data_lowbyte = SPI.transfer(cmd_lowbyte);
    digitalWrite(ssl, HIGH);                                            // disable the AS5048's
    data = azt_data_highbyte;
    data = data << 8;
    data = data | azt_data_lowbyte;
    value = data & 0x3FFF;                          // mask off the top two bits
    smoothAzimuthData = digitalSmooth(data, smoothAzimuthValue);
    data = alt_data_highbyte;
    data = data << 8;
    data = data | alt_data_lowbyte;
    value = data & 0x3FFF;                          // mask off the top two bits
    smoothAltitudeData = digitalSmooth(data, smoothAltitudeValue);
  }
}                                                                     // end setup

void loop() {
  // Get new sensor readings
  delay(1000);
  command = AS5048_CMD_READ | AS5048_REG_DATA;      // read data register
  command |= calcEvenParity(command) <<15;          // or with the parity of the command
  cmd_highbyte = highByte(command);                 // split it into high and low byte
  cmd_lowbyte = lowByte(command);                   //
  digitalWrite(ssl,LOW);                            // select the chip
  alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
  digitalWrite(ssl,HIGH);                           // but throw those two away as we don't know what the previous command was
  digitalWrite(ssl,LOW);                            // select the chip
  alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
  digitalWrite(ssl,HIGH);                           // 
  data = azt_data_highbyte;
  data = data << 8;
  data = data | azt_data_lowbyte;
  value = data & 0x3FFF;                          // mask off the top two bits
  smoothAzimuthData = digitalSmooth(data, smoothAzimuthValue);
  Serial.print("Smooth Azimuth: ");
  Serial.println(smoothAzimuthData);
  data = alt_data_highbyte;
  data = data << 8;
  data = data | alt_data_lowbyte;
  value = data & 0x3FFF;                          // mask off the top two bits
  smoothAltitudeData = digitalSmooth(data, smoothAltitudeValue);

  //
  // wait for a new client:
  // Serial.print(".");
  delay(1);
  WiFiClient thisClient = server.available();


  // when the client sends the first byte, say hello:
  while (thisClient) {
    if (!alreadyConnected) {
      Serial.println("");
      Serial.println("We have a new client");
      alreadyConnected = true;
    }
    if (thisClient.connected()) {
      if (thisClient.available() > 0) {
        // if there are chars to read....
        Serial.print("There are ");
        Serial.print(thisClient.available());
        Serial.println(" characters to be read");
        // lets print a response and discard the rest of the bytes
        thisClient.print(PadTic(Tic("Azimuth")));
        thisClient.print("\t");
        thisClient.print(PadTic(Tic("Altitude")));
        thisClient.print("\r\n");
        Serial.print("Azimuth Angle: ");
        Serial.print(Angle("Azimuth"));
        Serial.print(" Altituge Angle: ");
        Serial.println(Angle("Altitude"));
        Serial.print("Azimuth tic: ");
        Serial.print(PadTic(Tic("Azimuth")));
        Serial.print(" Altitude tic: ");
        Serial.println(PadTic(Tic("Altitude")));
        Serial.print("Smoothed Azimuth Value: ");
        Serial.print(smoothAzimuthData);
        Serial.print(" Smoothed Altitude Value: ");
        Serial.println(smoothAltitudeData);
        // discard remaining bytes
        thisClient.flush();
      }
    }
    else {
      Serial.println("diconnecting");
      thisClient.stop();
      alreadyConnected = false;
    }
  }
}

float Angle(String axis) {
  // take an axis and read that sensor to get the angle
  float angles[samplesNumToAverage];
  float angle;
  float AverageAngle = 0;
  int myCounter = 0;
  command = AS5048_CMD_READ | AS5048_REG_DATA;      // read data register
  command |= calcEvenParity(command) <<15;          // or with the parity of the command
  cmd_highbyte = highByte(command);                 // split it into high and low byte
  cmd_lowbyte = lowByte(command);                   // 
  digitalWrite(ssl,LOW);                            // select the chip
  alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
  digitalWrite(ssl,HIGH);                           // but throw those two away as we don't know what the previous command was

  for (myCounter = 0; myCounter <samplesNumToAverage; myCounter++ ){    
    digitalWrite(ssl,LOW);                          
    alt_data_highbyte = SPI.transfer(cmd_highbyte); // send the highbyte and lowbyte 
    alt_data_lowbyte = SPI.transfer(cmd_lowbyte);   // and read high and low byte for altitude
    azt_data_highbyte = SPI.transfer(cmd_highbyte); // same for azimuth
    azt_data_lowbyte = SPI.transfer(cmd_lowbyte);   // read high and low
    digitalWrite(ssl,HIGH);                         // close the chip
    if (axis ==  "Altitude") {
      data = alt_data_highbyte;                     // Store the high byte in my 16 bit varriable
      data = data << 8;                             // shift left 8 bits
      data = data | alt_data_lowbyte;               // tack on the low byte
    } else {
      data = azt_data_highbyte;
      data = data << 8;
      data = data | azt_data_lowbyte;
    }
    value = data & 0x3FFF;                          // mask off the top two bits
    angles[myCounter] = (float(value)/16383)*360;   // calculate the angle that represents
  }
  for (myCounter = 0; myCounter < samplesNumToAverage; myCounter++){
    AverageAngle = AverageAngle + angles[myCounter];
  }
  AverageAngle = AverageAngle / samplesNumToAverage;
  return AverageAngle;
}

unsigned int Tic(String axis) {
  // take an axis and read that sensor to get the raw encoder value
  unsigned int tics[samplesNumToAverage];
  unsigned int tic;
  float averageTic = 0;
  int myCounter = 0;
  command = AS5048_CMD_READ | AS5048_REG_DATA;      // read data register
  command |= calcEvenParity(command) <<15;               // or with the parity of the command
  cmd_highbyte = highByte(command);                 // split it into high and low byte
  cmd_lowbyte = lowByte(command);                   //
  digitalWrite(ssl,LOW);                            // select the chip
  alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
  digitalWrite(ssl,HIGH);                           // but throw those two away as we don't know what the previous command was 


  for (myCounter = 0; myCounter < samplesNumToAverage; myCounter++ ){    
    digitalWrite(ssl,LOW);                          
    alt_data_highbyte = SPI.transfer(cmd_highbyte); // send the highbyte and lowbyte 
    alt_data_lowbyte = SPI.transfer(cmd_lowbyte);   // and read high and low byte for altitude
    azt_data_highbyte = SPI.transfer(cmd_highbyte); // same for azimuth
    azt_data_lowbyte = SPI.transfer(cmd_lowbyte);   // 
    digitalWrite(ssl,HIGH);                         // close the chip

    if (axis ==  "Altitude") {
      data = alt_data_highbyte;                     // Store the high byte in my 16 bit varriable
      data = data << 8;                             // shift left 8 bits
      data = data | alt_data_lowbyte;               // tack on the low byte
    } else {
      data = azt_data_highbyte;
      data = data << 8;
      data = data | azt_data_lowbyte;
    }
    value = data & 0x3FFF;                          // mask off the top two bits
    tics[myCounter] = (value);                      // calculate the angle that represents
  }
  for (myCounter = 0; myCounter < samplesNumToAverage; myCounter++){
    averageTic = averageTic + tics[myCounter];
  }
  averageTic = averageTic / samplesNumToAverage;
  return averageTic;
}

// pad the Tics value with leading zeros and return a string
String PadTic(unsigned int tic){
  String paddedTic;
  if (tic < 10) 
    paddedTic = "+0000" + String(tic);
  else if ( tic < 100 )
    paddedTic = "+000" + String(tic);
  else if ( tic < 1000 )
    paddedTic = "+00" + String(tic);
  else if ( tic < 10000 )
    paddedTic = "+0" + String(tic);
  else if ( tic < 100000 )
    paddedTic = "+" + String(tic);

  return paddedTic;
}

// Calculate Even parity of word
byte calcEvenParity(word value) {
  byte count = 0;
  byte i;
  // loop through the 16 bits 
  for (i = 0; i < 16; i++) {
    // if the rightmost bit is 1 increment our counter
    if (value & 0x1) {
      count++;
    }
    // shift off the rightmost bit
    value >>=1;
  }
  // all odd binaries end in 1
  return count & 0x1;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/*
 AS5048 Functions:
 */
 
 // send a two byte command to two daisy chained AS5048As
 // we use pointers to the unsigned int's that we want to hold the returned data in
void sendAS5048_two(unsigned int cmd,unsigned int &d1, unsigned int &d2){
 cmd_highbyte = highByte(cmd);                     // split the command into high and low bytes
 cmd_lowbyte = lowByte(cmd);
 digitalWrite(ssl, LOW);                           // take the slave select LOW to issue a command
 alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
 alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
 azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
 azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
 digitalWrite(ssl,HIGH);                           // close the bus by taking slave select HIGH
 d1 = alt_data_highbyte;                      // reconstruct our data into unsigned int
 d1 = d1 << 8;
 d1 |= alt_data_lowbyte;
 d2 = azt_data_highbyte;
 d2 = d2 << 8;
 d2 |= azt_data_lowbyte;
}

int readAGC(byte &altAGC, &aztAGC){
 // try to read the AGC.
 // retruns -1 if there was an error reading either axis
  command = AS5048_CMD_READ | AS5048_REG_AGC;       // read data register
  command |= calcEvenParity(command) <<15;          // or with the parity of the command
  sendAS5048_two(command, altData, aztData);
  command = AS5048_CMD_NOP;
  sendAS5048_two(command, altData, aztData);
  if ((altData & 0x4000) || (aztData & 0x4000)) {
   // error flag for one of the axis.  need to reset it
   Serial.println("Error reading AGC");
   sendAS5048_two((SPI_CMD_READ | SPI_REG_CLRERR), altData, aztData);
   return -1;
  } else {
   altAGC = lowByte(altData);
   aztAGC = lowByte(aztData);
   return 0;
  }
}

int readData(unsigned int altRawData[], unsigned int aztRawData[] ){
 unsigned int lData;
 unsigned int zData;
 unsigned int lAGC;
 unsigned int zAGC;
 unsigned int lMag;
 unsigned int zMag;
 // send the READ_AGC command.  The received data is thrown away
  command = AS5048_CMD_READ | AS5048_REG_AGC;       // read data register
  command |= calcEvenParity(command) <<15;          // or with the parity of the command
  sendAS5048_two(command, lData, zData);
  // send the READ_MAG command.  the received data is the AGC data
  command = AS5048_CMD_READ | AS5048_REG_MAG;       // read data register
  command |= calcEvenParity(command) <<15;          // or with the parity of the command
  sendAS5048_two(command, lData, zData);
  lAGC = lData;
  zAGC = zData;
  // send the READ ANGLE command.  the received data is the magnitude 
  command = AS5048_CMD_READ | AS5048_REG_DATA
  command |= calcEvenParity(command) <<15;
  sendAS5048_two(command, lData, zData);
  lMag = lData;
  zMag = zData;
  // send the NOP command. the received data is the angle
  command = AS5048_CMD_NOP;
  sendAS5048_two(command, lData, zData);
  if ((lData & 0x4000) || (zData & 0x4000) || (lMag & 0x4000) || (zMag & 0x4000) || (lAGC & 0x4000) || (zAGC & 0x4000) ) {
   // error flag for one of the axis.  need to reset it
   Serial.println("Error reading");
   sendAS5048_two((SPI_CMD_READ | SPI_REG_CLRERR), altData, aztData);
   return -1;
  } else {
   altRawData[0] = lAGC;
   altRawData[1] = lMag;
   altRawData[2] = lData;
   aztRawData[0] = zAGC;
   aztRawData[1] = zMag;
   aztRawData[2] = zData;
   return 0;
  }
}

// this function takes a rawSensorData reading, inserts the value into the 'oldest' slot
// transfer the data into a pair of intermediate arrays used for sorting the values
// inserts the value into the respective sorted arrays according to the following
// if the value is not an overrun value ( new value << smallest current value )
unsigned int digitalSmooth(unsigned int rawSensorData, unsigned int *smoothSensorData){
  int j, k, temp, top, bottom;
  unsigned int minVal, maxVal;
  unsigned int offset = 0;
  long total;
  static int i;
  static unsigned int sortedValues[samplesNumToAverage];

  Serial.print("Raw: ");
  Serial.println(rawSensorData);
  i = ( i + 1 ) % samplesNumToAverage; // increment the counter and roll over if needed
  
  smoothSensorData[i] = rawSensorData; // insert the new value into the oldest slot

  // lets find the min and max in our array
  minVal = smoothSensorData[0];
  maxVal = smoothSensorData[0];  
  for ( j = 1; j < samplesNumToAverage; j++){ // look for min and max values on both axes
    if ( smoothSensorData[j] < minVal ){
      minVal = smoothSensorData[j];
    }
    if ( smoothSensorData[j] > maxVal){
      maxVal= smoothSensorData[j];
    }
  }
  /*
  Serial.print("minVal: ");
  Serial.println(minVal);
  Serial.print("maxVal: ");
  Serial.println(maxVal);
  */
  
  if ( maxVal - minVal > 10000 ) { // if we have a big gap between big and small values it must be
    offset = 5000; // that we are crossing the zero. So shift away from zero
  }

  for ( j = 0; j < samplesNumToAverage; j++) { // transfer data to arrays for sorting and averaging
    sortedValues[j] = (smoothSensorData[j] + offset ) % 16384;
  }
  // print unsorted array
  Serial.println("Unsorted");
  for ( j = 0; j < samplesNumToAverage; j++){
    Serial.print(sortedValues[j]);
    Serial.print(" : ");
  }
  Serial.println("");
  Serial.println("Sorted");
  // insertion sort the arrays
  for ( k = 1 ; k < samplesNumToAverage; k++){
    j = k;
    while (j > 0 && sortedValues[j-1] > sortedValues[j]) {
      temp = sortedValues[j-1];
      sortedValues[j-1] = sortedValues[j];
      sortedValues[j] = temp;
      j -= 1;
    }
  }
  for ( j = 0; j < samplesNumToAverage; j++){
    Serial.print(sortedValues[j]);
    Serial.print(" : ");
  }
  Serial.println("");
// throw out the top x% and bottom x% of samples but limit to throw out at least one
  bottom = max(((samplesNumToAverage *15) / 100), 1);
  top = min((((samplesNumToAverage * 85) / 100) + 1), samplesNumToAverage - 1);
  k = 0;
  total = 0;
  for ( j = bottom ; j < top ; j++ ){
    total += sortedValues[j];
    k++;
  }
  return (unsigned int) (( total / k ) - offset);
}
