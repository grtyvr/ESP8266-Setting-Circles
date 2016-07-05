/*
 Based on: 
 Chat  Server
 created 18 Dec 2009
 by David A. Mellis
 modified 31 May 2012
 by Tom Igoe

 Modified by grtyvr to accept connections from SkySafari over the network and send back angle data
 from AS5048 angle sensors.
 
 Thanks to the folks at ZoetropeLabs for example code to access the AS5048's
 https://github.com/ZoetropeLabs/AS5048A-Arduino
 July 5, 2015
 Added running as a WiFi Access point
 Added code to read an AS5048 
 July 6, 2015
 Added parity calculations to the commands
 August 5, 2015
 So now it works, but not because of code, but because of good power supply.  AS5048A is sensitive
 removed a buch of non working code and included a debuging ifdef
 August 8, 2015
 Well, it actually did not work.  There were two bugs and despite them it gave close to accurate results.
  1) 2^14 is not 16393 it is 16383.
  2) when you truncate off the bottom 8 bits of a 14 bit number that is only losing about 1.5 degres of precision.
 
 To Do:
 1) Make the AS5048 stuff into a library and add some functions/features for sensor positioning using the MAG register.
 2) Do some error checking on the sensor readings.  ( low priority.  They seen very reliable )
 3) Figure out how to do OTA updates on the ESP8246.  At the very least that would be cool!

 July 4 2016
 major refactoring.  I was naieve to think that a linear boxcar smoothing with discarding outlier data could be
 made to work.  I forgot totaly about edge cases for sorting an array of values that wrap around from 360 == 0
 Circular smoothing using the atan2 method with boxcar works much better.  The basic idea is to keep a ring buffer
 with the last n smoothed data points in it. When you collect a new data point you add it to the oldest position in
 the buffer and calculate the circular mean of the resulting angles, then store the new smooted value in the buffer
 at the oldest value position. Emperical testing with n=10 sesms to make a nice feel to the data with negligible jitter.
 
 */

#include <SPI.h>
#include <ESP8266WiFi.h>

// Define this if you want to run as an Access Point.  If undefined it will connect to the 
// SSID with the password below....

#define AP 
// uncomment the next line to turn on debugging
// #define DEBUGGING

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

#define numToAve 10         // This is the number of past readings we will use to create our moving average
int azPin=15;               // azimuth sensor CSEL pin. Depending on how you build your hardware you might need to change
int altPin=5;               // altitude sensor CSEL pin. Same with this.
int curArrayPos=0;          // this is a counter to store the current array position that we will place the new reading in
byte cmd_highbyte = 0;      //
byte cmd_lowbyte = 0;
byte alt_data_highbyte = 0;
byte alt_data_lowbyte = 0;
byte azt_data_highbyte = 0;
byte azt_data_lowbyte = 0;
word command = 0;
unsigned int value = 0;
unsigned int rawData;
double increment = (double) 360 / 16384;  // this is the smallest angular increment that is reportable by the sensors
int i = 0;
int del = 10;

// the arrays that will store the most recent numToAverage angles
double smoothAzAngles[numToAve];
double smoothAltAngles[numToAve];
// the value of the current Azimuth and Altitude angle that we will report back to Sky Safari
double smoothAzAngle = 0;
double smoothAltAngle = 0;

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
  for (i = 0; i < numToAve; i++){
    rawData = readTic(azPin);
    smoothAzAngles[i] = ticsToAngle(rawData);
    rawData = readTic(altPin);
    smoothAltAngles[i] = ticsToAngle(rawData);
  }
  curArrayPos = numToAve - 1;       // set our position counter so that we know where the oldest value is
                                    // in this case it is the last element of the array numToAve - 1
} // end setup

void loop() {
  // update a counter to keep track of the current position in the arrays of readings
  // this will always be an integer between 0 and numToAverage inclusive 
  curArrayPos = (curArrayPos + 1 ) % numToAve;
  // each time through the main loop we will get the current sensor readings for each axis
  // and update the array with the smoothed value
  rawData = readTic(azPin);                             // read the tics of Azimuth pin
  smoothAzAngles[curArrayPos] = ticsToAngle(rawData);   // put the unsmoothed value in the array at the curArrayPos
  smoothAzAngle = circularMean(smoothAzAngles);         // send the array off for calculating the circularMean
  smoothAzAngles[curArrayPos] = smoothAzAngle;          // and store the smoothed angle in our array
  #ifdef DEBUGGING
    Serial.print("Raw Az Angle: ");
    Serial.print(ticsToAngle(rawData));
    Serial.print(" CircSmooth Az: ");
    Serial.print(smoothAzAngle);
  #endif
  rawData = readTic(altPin);
  smoothAltAngles[curArrayPos] = ticsToAngle(rawData);
  smoothAltAngle = circularMean(smoothAltAngles);
  smoothAltAngles[curArrayPos] = smoothAltAngle;
  #ifdef DEBUGGING
    Serial.print(" Raw Alt Angle: ");
    Serial.print(ticsToAngle(rawData));
    Serial.print(" CircSmooth Al: ");
    Serial.println(smoothAltAngle);
  #endif
  delay(del);
  
  // wait for a new client:  
  WiFiClient thisClient = server.available();
  // when the client sends the first byte, say hello:
  while (thisClient) {
    if (!alreadyConnected) {
      alreadyConnected = true;
    }
    if (thisClient.connected()) {
      if (thisClient.available() > 0) {
        // if there are chars to read....
        // this is where we need to actually read what the client is sending but we assume that it 
        // is a R = read command
        // it is expecting 
        // lets print a response and discard the rest of the bytes
        //
        thisClient.print(PadTic(angleToTics(smoothAzAngle), "+"));
        thisClient.print("\t");
        thisClient.print(PadTic(angleToTics(smoothAltAngle), "+"));
        thisClient.print("\r\n");
        #ifdef DEBUGGING
          Serial.print("Azimuth tic: ");
          Serial.print(PadTic(angleToTics(smoothAzAngle), "-"));
          Serial.print(" Altitude tic: ");
          Serial.println(PadTic(angleToTics(smoothAltAngle), "-"));
        #endif
        // discard remaining bytes
        thisClient.flush();
      }
    }
    else {
      thisClient.stop();
      alreadyConnected = false;
    }
  }
} // End loop

///////////////// Functions and procedures
//
// pad the Tics value with leading zeros and return a string
String PadTic(unsigned int tic, String Sign){
  String paddedTic;
  if (tic < 10) 
    paddedTic = "0000" + String(tic);
  else if ( tic < 100 )
    paddedTic = "000" + String(tic);
  else if ( tic < 1000 )
    paddedTic = "00" + String(tic);
  else if ( tic < 10000 )
    paddedTic = "0" + String(tic);
  else if ( tic < 100000 )
    paddedTic = "" + String(tic);
  paddedTic = Sign + paddedTic;
  return paddedTic;
}

// calcEvenParity of word
// this is not currently used but I should!
//
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

// readTic
// This just trims the bottom 14 bits off of a sensor read
unsigned int readTic(int cs){
  unsigned int rawData;
  unsigned int realData;
  rawData = readSensor(cs);
  realData = rawData & 0x3fff; 
  return realData;
}

// readSensor
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
  data = data | data_lowbyte;                                         // tack on the low byte
//  #ifdef DEBUGGING
//    Serial.println(data);
//  #endif
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
} // end printWifiStatus


// return the circular mean of the angles using the atan2 method
// takes a pointer to an array of angles
double circularMean( double *anglesToAverage){
  int j;
  int k;
  double totalX = 0;
  double totalY = 0;
  double angle = 0;
  double retVal = 0;
  k = numToAve;
  for ( j = 0; j < k ; j++) { 
    yield(); 
   totalX += cos((double) ((anglesToAverage[j] * PI) / 180)); // convert angle to radians
   totalY += sin((double) ((anglesToAverage[j] * PI) / 180)); // convert angle to radians
  }
  // strictly speaking the circular mean using the atan2 method is defined as taking the mean of the x and y components
  // of each angle and using the resulting mean x value and mean y value as inputs to the atan2 function
  // So we should divide the totalX and TotalY by the number of values, but in ATAN2 the linear factor
  // does not make a difference in the value of the function i.e. atan2( x , y) = atan2 (mx , my)
  if ( totalX == 0 && totalY == 0 ) { 
    angle = 0;                                 // just to be safe define a value where atan2 is undefined
  } else {
    angle = atan2(totalY , totalX);
    if (angle >= 0) {
      // if the returned value of angle is positive it is a positive rotation (CCW) between 0 and 180 degress
      retVal = (double) ((angle / PI) * 180);
    } else {
      // convert the negative angle to a positive rotation from 0 degrees (CCW)
      retVal =  (double) ((( 2 * PI ) + angle) / PI ) * 180;
    }
  }
  return retVal;
} // end circularMean

// convert an angle to tics
unsigned int angleToTics( double angle){
  unsigned int retVal = 0; 
  retVal = (unsigned int) (((angle + (increment / 2)) / 360 ) * 16384);
  if ( retVal > 16383 ) {
    retVal = retVal - 16384;
  }
  return retVal;
} // end angleToTics

// convert tics to angle
double ticsToAngle ( unsigned int tics) {
  double retVal;
  retVal = tics * increment;
  return retVal;
} // end ticsToAngle
