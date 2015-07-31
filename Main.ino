/*
 * Based on:
 * Chat Server 
 * 
 * created 18 Dec 2009
 * by David A. Mellis
 * 
 * modified 31 May 2012
 * by Tom Igoe
 * 
 * Modified to accept connections from SkySafari over the network and send back angle data
 * from AS5048 angle sensors
 * 
 * July 5, 2015
 * Added running as a WiFi Access point
 * Added code to read an AS5048
 * 
 * July 6, 2015
 * Added parity calculations to the commands
 * 
 * July 14, 2015
 * Added better code to calculate a running average with throwing out the high and low 15% of values
 * Added a structure to hold the two sensor readings ( this will have to wait till i create a library for it
 *  sounds like there are no struct in sketches on arduino, only in libraries
 * 
 * July 15, 2015
 * Refactored to make functions for the AS5048A calls (in progress)
*/

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <AS5048A.h>


// Define this if you want to run as an Access Point. If undefined it will connect to the
// SSID with the password below....
#define AP

char ssid[] = "braapppp"; // your network SSID (name)
char pass[] = ""; // your network password (use for WPA, or use as key for WEP)

const char *apssid = "ESPap";
const char *appassword = "gofish";

int keyIndex = 0; // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;

// for smoothing sensor data
#define samplesNumToAverage 20
unsigned int smoothAzimuthValues[samplesNumToAverage];
//unsigned int smoothAltitudeValue[samplesNumToAverage];

//int ssl1=15;
//int ssl2=16;
//byte cmd_highbyte = 0;
//byte cmd_lowbyte = 0;
//byte alt_data_highbyte = 0;
//byte alt_data_lowbyte = 0;
//byte azt_data_highbyte = 0;
//byte azt_data_lowbyte = 0;
//word command = 0;
//unsigned int data = 0;
unsigned int smoothAzimuthData = 0;
//word smoothAltitudeData = 0;
//unsigned int value = 0;
unsigned int Data[5];
//byte altAGC;
//byte aztAGC;
//unsigned int magnitude;
//byte altOCF;
//byte aztCOF;
//byte altAlarmHigh;
//byte altAlarmLow;
//byte aztAlarmHigh;
//byte aztAlarmLow;
//float angle = 0;
//int i = 0;
//int del = 10;
int readError=-1;

WiFiServer server(23);
AS5048A azimuthSensor(15);

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

// initialize and read the sensor till we get a good reading
// we can expand on this later to fill up our smoothing array
while (readError != -1){
  readError = azimuthSensor.readData(Data);
}

} // end setup

void loop() {
  // Get new sensor readings each time we loop.
  
  Serial.println("Reading Sensors");
  readError = azimuthSensor.readData(Data);
  if (readError == -1 ) {
    Serial.println("Error on read");
  } else {
    Serial.println("Read successful");
    Serial.print("Data-0: ");
    Serial.print(Data[0]);
    Serial.println("     Gain");
    Serial.print("Data-1: ");
    Serial.print(Data[1]);
    Serial.println("   Magnitude");
    Serial.print("Data-2: ");
    Serial.print(Data[2]);
    Serial.println("      Angle");
    Serial.print("Data-3: ");
    Serial.print(Data[3]);
    Serial.println("    High magnetic field");
    Serial.print("Data-4: ");
    Serial.print(Data[4]);
    Serial.println("    Low Magnetic field");
  }
  
  delay(1000);

  smoothAzimuthData = digitalSmooth(Data[2], smoothAzimuthValues);
  Serial.print("Smooth Azimuth: ");
  Serial.println(smoothAzimuthData);

  // wait for a new client:
  // Serial.print(".");
  delay(1);
  WiFiClient thisClient = server.available();
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
        thisClient.print(PadTic(smoothAzimuthData));
        thisClient.print("\t000000\r\n");
        Serial.print("Smoothed Azimuth Value: ");
        Serial.print(smoothAzimuthData);
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
  //Serial.print("Raw: ");
  //Serial.println(rawSensorData);
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
  bottom = 1;//max(((samplesNumToAverage *15) / 100), 1);
  top = samplesNumToAverage; //min((((samplesNumToAverage * 85) / 100) + 1), samplesNumToAverage - 1);
  k = 0;
  total = 0;
  for ( j = bottom ; j < top ; j++ ){
    total += sortedValues[j];
    k++;
  }
  return (unsigned int) (( total / k ) - offset);
}
