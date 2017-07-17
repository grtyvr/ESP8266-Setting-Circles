 /*
 
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

#define AP                                                  // Define this if you want to run as an Access Point.
                                                            // If undefined it will connect to the SSID with the password below....
//#define DEBUGGING                                         // uncomment to turn on debugging
char ssid[] = "braapppp";                                   //  your network SSID (name)
char pass[] = "";                                           // your network password (use for WPA, or use as key for WEP)
const char *apssid = "ESPap";
const char *appassword = "gofish";
int keyIndex = 0;                                           // your network key Index number (needed only for WEP)
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

#define numToAve 10                                         // This is the number of past readings we will use to create our
                                                            // moving average
int azPin=15;                                               // azimuth sensor CSEL pin. Depending on how you build your
                                                            // hardware you might need to change
int altPin=5;                                               // altitude sensor CSEL pin. Same with this.
int curArrayPos=0;                                          // this is a counter to store the current array position that
                                                            // we will place the new reading in
byte cmd_highbyte = 0;                                      //
byte cmd_lowbyte = 0;
byte alt_data_highbyte = 0;
byte alt_data_lowbyte = 0;
byte azt_data_highbyte = 0;
byte azt_data_lowbyte = 0;
word command = 0;
unsigned int value = 0;
unsigned int rawData;
double increment = (double) 360 / 16384;                    // this is the smallest angular increment 
                                                            // that is reportable by the sensors
int i = 0;                                                  // declare a counter
int del = 2;                                                // 
double smoothAzAngles[numToAve];                            // the arrays that will store the most recent numToAverage angles
double smoothAltAngles[numToAve];                           //
double smoothAzAngle = 0;                                   // the value of the current Azimuth and Altitude angle that we will 
double smoothAltAngle = 0;                                  // report back to Sky Safari
WiFiServer server(23);                                      // define an instance of a WIfi Server called 'server'
boolean alreadyConnected = false;                           // whether or not the client was connected previously

/////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
//

  Serial.begin(115200);                                     // Initialize serial
  delay(1000);                                              // and pause
  
  #ifdef AP                                                 // If we starting as an access point
    Serial.println("Setting up WiFi Access Point");         // alert the console that we are going to...
    WiFi.mode(WIFI_AP);                                     // start up as access point
    WiFi.softAP(apssid);                                    // use the const defined at the top for the SSID
    Serial.print("AP IP Address: ");                        // alert the console with our IP address
    Serial.println(WiFi.softAPIP());                        //
  #else                                                     // else attempt to connect to Wifi network:  
    while ( status != WL_CONNECTED) {                       //   try to connect, forever!
      Serial.print("Attempting to connect to SSID: ");      //  alert the console of our plight
      Serial.println(ssid);                                 //  in case they can help?
      status = WiFi.begin(ssid, pass);                      // Connect to WPA/WPA2 network. Change this line if using open or
                                                            // WEP network:
      delay(5000);                                          // wait 5 seconds for connection:
    }
  #endif
 
  server.begin();                                           // start the WIfi server:
  #ifdef AP                                                 // if we are an access point....
  #else                                                     // if we are connecting to an AP
    printWifiStatus();                                      // you're connected now, so print out the status:
  #endif
  
  SPI.begin();                                              // Wake up the buss
  SPI.setBitOrder(MSBFIRST);                                // AS5048 is a Most Significant Bit first
  SPI.setDataMode(SPI_MODE1);                               // AS5048 uses Mode 1
  for (i = 0; i < numToAve; i++){                           // fill up our smoothing arrays with data
    rawData = readTic(azPin);                               // read the azimuth pin
    smoothAzAngles[i] = ticsToAngle(rawData);               // smooth the angle ( we should probably just ignore this in setup )
    rawData = readTic(altPin);                              // read the altitude pin
    smoothAltAngles[i] = ticsToAngle(rawData);              // since we are just filling up the array with data smoothing might not make
  }                                                         // sense here.....
  curArrayPos = numToAve - 1;                               // set our position counter so that we know where the oldest value is
                                                            // in this case it is the last element of the array numToAve - 1
} // end setup()
//
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop() {
//
//

  curArrayPos = (curArrayPos + 1 ) % numToAve;              // update a counter to keep track of the current
                                                            // position in the arrays of readings. This will
                                                            // always be an integer between 0 and numToAverage inclusive
                                                          
                                                            // each time through the main loop we will get the current
                                                            // sensor readings for each axis and update the array with
                                                            // the smoothed value
  rawData = readTic(azPin);                                 // read the tics of Azimuth pin
  smoothAzAngles[curArrayPos] = ticsToAngle(rawData);       // put the unsmoothed value in the array at the curArrayPos
  smoothAzAngle = circularMean(smoothAzAngles);             // send the array off for calculating the circularMean
  smoothAzAngles[curArrayPos] = smoothAzAngle;              // and store the filtered angle in our array
  #ifdef DEBUGGING
    Serial.print("Raw Az Data: ");
    Serial.print(rawData);
    Serial.print(" Raw Az Angle: ");
    Serial.print(ticsToAngle(rawData));
    Serial.print(" CircSmooth Az: ");
    Serial.print(smoothAzAngle);
  #endif
  rawData = readTic(altPin);                                // and the same for the altitude pin
  smoothAltAngles[curArrayPos] = ticsToAngle(rawData);      // store the raw data in the array
  smoothAltAngle = circularMean(smoothAltAngles);           // send the array off for processing
  smoothAltAngles[curArrayPos] = smoothAltAngle;            // store the filtered value.
  #ifdef DEBUGGING
    Serial.print("Raw Alt Data: ");
    Serial.print(rawData);
    Serial.print(" Raw Alt Angle: ");
    Serial.print(ticsToAngle(rawData));
    Serial.print(" CircSmooth Al: ");
    Serial.println(smoothAltAngle);
  #endif
  delay(del);                                               // and hang for a bit...
  
  WiFiClient thisClient = server.available();               // wait for a new client:  
  Serial.println("Waiting");                                // let the console know what we are doing

  while (thisClient) {                                      // if we have a connection then thisClient will be non-zero 
    if (!alreadyConnected) {                                // if this is a not a new connection
      alreadyConnected = true;                              // say we have a connection
    }
    if (thisClient.connected()) {                           // if we are connected to a client with characters to be read
      if (thisClient.available() > 0) {                     //  if there are chars to read.
                                                            //    this is where we need to actually read what the client
                                                            //    is sending but we assume that what is sent is a R = read
                                                            //    command 
                                                            //    lets print a response and discard the rest of the bytes
        thisClient.print(PadTic(angleToTics(smoothAzAngle), "+"));  //  send back the data packet
        thisClient.print("\t");                                     //  starts with the data and then a + \t
        thisClient.print(PadTic(angleToTics(smoothAltAngle), "+")); //  then another data packet with a +
        thisClient.print("\r\n");                                   //  end with \r\n
                                                                    //  the data packet needs to have fixed length so
                                                                    //  we pad the acual data to 7 digits
        #ifdef DEBUGGING                                            
          Serial.println("sending data to client");
          Serial.print("Azimuth tic: ");
          Serial.print(PadTic(angleToTics(smoothAzAngle), "+"));
          Serial.print(" Altitude tic: ");
          Serial.println(PadTic(angleToTics(smoothAltAngle), "+"));
        #endif
        // discard remaining bytes
        thisClient.flush();                                 // discard anything else the client sent
      }
    }
    else {                                                  // this was not a new connection !?
      thisClient.stop();                                    // stop the client, since we must have processed it above
      alreadyConnected = false;                             // say we are ready for a new connection
    }
  }
} // End loop()
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////// Functions and procedures ///////////////////////////////////////////////////////////////
//
/////////////////
//
// readSensor
// Read the sensor REG_DATA register
// we send two read commands so that the REG_DATA will have the results of a read command
//
unsigned int readSensor(int cs){
  unsigned int data;
  unsigned int data_highbyte;
  unsigned int data_lowbyte;
  pinMode(cs, OUTPUT);                                       
  command = AS5048_CMD_READ | AS5048_REG_DATA;                // Set up the command we will send
  command |= calcEvenParity(command) <<15;                    // assign the parity bit of the command we are sending (error check)
  cmd_highbyte = highByte(command);                           // split it into bytes
  cmd_lowbyte = lowByte(command);                             //
  digitalWrite(cs, LOW);                                      // Drop Cable Select pin to enable the AS5048
  data_highbyte = SPI.transfer(cmd_highbyte);                 // send the initial read command
  data_lowbyte = SPI.transfer(cmd_lowbyte);
  digitalWrite(cs, HIGH);                                     // disable the AS5048's
  digitalWrite(cs, LOW);                                      // Drop ssl to enable the AS5048's
  data_highbyte = SPI.transfer(cmd_highbyte);                 // send a second read command
  data_lowbyte = SPI.transfer(cmd_lowbyte);                   //
  digitalWrite(cs, HIGH);                                     // Bring CS high again to disable the AS5048 
  data = data_highbyte;                                       // Store the high byte in my 16 bit varriable
  data = data << 8;                                           // shift left 8 bits
  data = data | data_lowbyte;                                 // tack on the low byte
//  #ifdef DEBUGGING
//    Serial.println(data);
//  #endif
  return data;                                                // and return the full 16 bits of data
}
// end readSensor(cs)

/////////////////
//
//  circularMean
//  return the circular mean of the angles using the atan2 method
//  takes a pointer to an array of angles
//
double circularMean( double *anglesToAverage){
  int j;
  int k;
  double totalX = 0;
  double totalY = 0;
  double angle = 0;
  double retVal = 0;
  k = numToAve;                                               //  
  for ( j = 0; j < k ; j++) {                                 // Loop through the anglesToAverage array
    yield();                                                  //  Each time through yied so that WIfi can catch up
   totalX += cos((double) ((anglesToAverage[j] * PI) / 180)); //  convert angle to radians and add it to our totalX
   totalY += sin((double) ((anglesToAverage[j] * PI) / 180)); //  convert angle to radians and add it to our totalY
  }                                                           //  strictly speaking the circular mean using the atan2 method is
                                                              //  defined as taking the mean of the x and y components of each angle
                                                              //  and using the resulting mean x value and mean y value as inputs to the
                                                              //  atan2 function, so we should divide the totalX and TotalY by
                                                              //  the number of values, but in ATAN2 the linear factor does not make
                                                              //  a difference in the value of the function
                                                              //     i.e. atan2( x , y) = atan2 (mx , my)
  if ( totalX == 0 && totalY == 0 ) { 
    angle = 0;                                                // just to be safe assign a value where atan2 is undefined
  } else {
    angle = atan2(totalY , totalX);                           // get the arctan of the totalY,totalX value
    if (angle >= 0) {                                         // if the arctan is positive then it is a positive rotation (CCW)
                                                              // between 0 and 180 degress
      retVal = (double) ((angle / PI) * 180);                 // and we can convert it back to degress to send back
    } else {                                                  // if the arctan is negative...
      retVal =  (double) ((( 2 * PI ) + angle) / PI ) * 180;  // convert the negative rotation to a positive rotation from 
                                                              // 0 degrees (CCW) by taking the complement of the angle
    }
  }
  return retVal;
} // end circularMean(*anglesToAverage)

/////////////////
//
// padTic
// pad the Tics value with leading zeros and a plus sign and return a string
// since sky safari expects a sream of characters "+nnnnn and not a number
// 
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
} // end padTic(tic,sign)

/////////////////
//
// calcEvenParity of word
//
byte calcEvenParity(word value) {
  byte count = 0;
  byte i; 
  for (i = 0; i < 16; i++) {																	  	// loop through the 16 bits counting the 1's
    if (value & 0x1) {																						// 	if the rightmost bit is 1 
      count++;																										// 	increment our counter
    }																															// 	and then 
    value >>=1;																										// 	shift off the rightmost bit
  }																																
  return count & 0x1;																							// all odd binaries end in 1
} // end calcEvenParity(value)

/////////////////
//
// readTic
// This just trims the bottom 14 bits off of a sensor read
//
unsigned int readTic(int cs){
  unsigned int rawData;
  unsigned int realData;
  rawData = readSensor(cs);
  realData = rawData & 0x3fff; 
  return realData;
} // end readTic(cs)


/////////////////
//
//  convert an angle to tics
//
unsigned int angleToTics( double angle){
  unsigned int retVal = 0; 
  retVal = (unsigned int) (((angle + (increment / 2)) / 360 ) * 16384);
  if ( retVal > 16383 ) {
    retVal = retVal - 16384;
  }
  return retVal;
} // end angleToTics(angle)

/////////////////
//
//  convert tics to angle
//
double ticsToAngle ( unsigned int tics) {
  double retVal;
  retVal = tics * increment;
  return retVal;
} // end ticsToAngle

/////////////////
//
//  printWifiSataus
// 
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
