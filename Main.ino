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

 August 5, 2015
 So now it works, but not because of code, but because of good power supply.  AS5048A is sensitive
 removed a buch of non working code and included a debuging ifdef
 
 */

#include <SPI.h>
#include <ESP8266WiFi.h>

// Define this if you want to run as an Access Point.  If undefined it will connect to the 
// SSID with the password below....

#define AP 
// uncomment the next line to turn on debugging
//#define DEBUGGING

char ssid[] = "braapppp"; //  your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)

const char *apssid = "ESPap";
const char *appassword = "gofish";

int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// These defines are for the AS5048 
// for example...
// http://ams.com/eng/Support/Demoboards/Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5048A-Adapterboard  

#define AS5048_CMD_READ 0x4000    
#define AS5048_REG_AGC 0x3FFD
#define AS5048_REG_MAG 0x3FFE
#define AS5048_REG_DATA 0x3FFF
#define AS5048_REG_ERR 0x1
#define AS5048_CMD_NOP 0x0

#define numToAverage 50
int azimuthSensorPin=15;
int altitudeSensorPin=4;
byte cmd_highbyte = 0;
byte cmd_lowbyte = 0;
byte alt_data_highbyte = 0;
byte alt_data_lowbyte = 0;
byte azt_data_highbyte = 0;
byte azt_data_lowbyte = 0;
word command = 0;
word data = 0;
unsigned int value = 0;
unsigned int rawData;
float angle = 0;
int i = 0;
int del = 10;

// to support digital smooth
unsigned int smoothAzimuthValues[numToAverage];
unsigned int smoothAltitudeValues[numToAverage];
// the array that will sotre the most recent numToAverage angles
//float smoothAzimuthAngles[numToAverage];
//float smoothAltitudeAngles[numToAverage];
float smoothAzimuthValuesX[numToAverage];
float smoothAzimuthValuesY[numToAverage];
float smoothAltitudeValuesX[numToAverage];
float smoothAltitudeValuesY[numToAverage];
unsigned int smoothAzimuthValue = 0;
unsigned int smoothAltitudeValue = 0;
//float advancedCircularSmoothAzimuthAngle = 0;
//float advancedCircularSmoothAltitudeAngle = 0;
unsigned int circularSmoothAzimuthValue = 0;
unsigned int circularSmoothAltitudeValue = 0;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  //Initialize serial
  Serial.begin(115200);
  delay(1000);
  
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
  
  SPI.begin();                                                        // Wake up the buss
  SPI.setBitOrder(MSBFIRST);                                          // AS5048 is a Most Significant Bit first
  SPI.setDataMode(SPI_MODE1);                                         // AS5048 uses Mode 1
  // fill up our smoothing arrays with data
  for (i = 1; i <= numToAverage; i++){
    rawData = readTic(azimuthSensorPin);
    circularSmoothAzimuthValue = circularSmooth(rawData, smoothAzimuthValuesX, smoothAzimuthValuesY);
    rawData = readTic(altitudeSensorPin);
    circularSmoothAltitudeValue = circularSmooth(rawData, smoothAltitudeValuesX, smoothAltitudeValuesY);
  }
} // end setup

void loop() {
  // wait for a new client:
  rawData = readTic(azimuthSensorPin);
  circularSmoothAzimuthValue = circularSmooth(rawData, smoothAzimuthValuesX, smoothAzimuthValuesY);
  Serial.print(" CircSmooth Az: ");
  Serial.print(circularSmoothAzimuthValue);
  rawData = readTic(altitudeSensorPin);
  circularSmoothAltitudeValue = circularSmooth(rawData, smoothAltitudeValuesX, smoothAltitudeValuesY);
  Serial.print(" CircSmooth Al: ");
  Serial.println(circularSmoothAltitudeValue);
  delay(1);
  
  WiFiClient thisClient = server.available();
  // when the client sends the first byte, say hello:
  while (thisClient) {
    if (!alreadyConnected) {
      alreadyConnected = true;
    }
    if (thisClient.connected()) {
      if (thisClient.available() > 0) {
        // if there are chars to read....
        // lets print a response and discard the rest of the bytes
        thisClient.print(PadTic(circularSmoothAzimuthValue));
        thisClient.print("\t");
        thisClient.print(PadTic(circularSmoothAltitudeValue));
        thisClient.print("\r\n");
        Serial.print("Azimuth tic: ");
        Serial.print(PadTic(smoothAzimuthValue));
        Serial.print(" Altitude tic: ");
        Serial.println(PadTic(smoothAltitudeValue));
        // discard remaining bytes
        thisClient.flush();
      }
    }
    else {
      thisClient.stop();
      alreadyConnected = false;
    }
  }
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
// This just trims the bottom 14 bits off of a sensor read
unsigned int readTic(int cs){
  unsigned int rawData;
  unsigned int realData;
  rawData = readSensor(cs);
  realData = rawData & 0x3fff; 
  return realData;
}
// Read the sensor REG_DATA register
unsigned int readSensor(int cs){
  unsigned int data;
  unsigned int data_highbyte;
  unsigned int data_lowbyte;
  pinMode(cs, OUTPUT);                                       
  command = AS5048_CMD_READ | AS5048_REG_DATA;                        // Set up the command we will send
  command |= calcEvenParity(command) <<15;                            // assign the parity bit
  cmd_highbyte = highByte(command);                                   // split it into bytes
  cmd_lowbyte = lowByte(command);                                     //
  digitalWrite(cs, LOW);                                             // Drop ssl to enable the AS5048's
  data_highbyte = SPI.transfer(cmd_highbyte);                         // send the initial read command
  data_lowbyte = SPI.transfer(cmd_lowbyte);
  digitalWrite(cs, HIGH);                                            // disable the AS5048's
  digitalWrite(cs, LOW);                                             // Drop ssl to enable the AS5048's
  data_highbyte = SPI.transfer(cmd_highbyte);                         // send the initial read command
  data_lowbyte = SPI.transfer(cmd_lowbyte);
  digitalWrite(cs, HIGH);                                            // disable the AS5048's  
  data = data_highbyte;                                               // Store the high byte in my 16 bit varriable
  data = data << 8;                                                   // shift left 8 bits
  data = data | alt_data_lowbyte;                                     // tack on the low byte
  return data;
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

// this function takes a rawSensorData reading converts it to and angle
// inserts the X and Y coordinates of that angle on the unit ciricle into two 
// arrays that hold the last 50 values read from the sensor
// 
unsigned int circularSmooth(unsigned int rawSensorData, float *smoothSensorDataX, float *smoothSensorDataY){
  int j;
  double totalX = 0;
  double averageX = 0;
  double totalY = 0;
  double averageY = 0;
  double angle = 0;
  static int currentCircularSmoothPosition;
  float currentAngle;
//  Serial.print("Raw: ");
//  Serial.println(rawSensorData);
  angle = (((float) rawSensorData)/16393 ) * 360;
//  Serial.print("angle: ");
//  Serial.println(angle);
//  Serial.print("Current Circular Smooth Position: ");
//  Serial.println(currentCircularSmoothPosition);
  currentCircularSmoothPosition = ( currentCircularSmoothPosition + 1 ) % numToAverage; // increment the counter and roll over if needed
  smoothSensorDataX[currentCircularSmoothPosition] = cos((angle * PI) /  180); // insert the new value into the oldest slot
  smoothSensorDataY[currentCircularSmoothPosition] = sin((angle * PI) /  180); // insert the new value into the oldest slot
  for ( j = 0 ; j < numToAverage ; j++ ){
    totalX += smoothSensorDataX[j];
    totalY += smoothSensorDataY[j];
  }
  averageX = totalX / numToAverage;
  averageY = totalY / numToAverage;
  currentAngle = atan2(averageY , averageX);
  if (currentAngle >=0 ) {
    // if the retruned value of angle is positive it is a positive rotation between 0 and 180 degres
    return (unsigned int) ((((currentAngle / PI) * 180) / 360 ) * 16393);
  } else {
    // convert the negative angel to a posive rotation from 0 degres counterclockwise
    return (unsigned int) (((((2 * PI + currentAngle) / PI) * 180) / 360) * 16393);
  }
}


