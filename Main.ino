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
 Added second AS5048 with daisychain mode but for some reason it has both axis getting the same value.

 
 */

#include <SPI.h>
#include <ESP8266WiFi.h>

// Define this if you want to run as an Access Point.  If undefined it will connect to the 
// SSID with the password below....

#define AP 

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

int numToAverage=50;
int ssl=15;
byte cmd_highbyte = 0;
byte cmd_lowbyte = 0;
byte alt_data_highbyte = 0;
byte alt_data_lowbyte = 0;
byte azt_data_highbyte = 0;
byte azt_data_lowbyte = 0;
word command = 0;
word data = 0;
unsigned int value = 0;
float angle = 0;
int i = 0;
int del = 10;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  //Initialize serial and wait for port to open:
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
  SPI.begin();                                                        // Wake up the buss
  SPI.setBitOrder(MSBFIRST);                                          // AS5048 is a Most Significant Bit first
  SPI.setDataMode(SPI_MODE1);                                         // AS5048 uses Mode 1
  command = AS5048_CMD_READ | AS5048_REG_DATA;                        // Set up the command we will send
  command = command | calcEvenParity(command)<<15;                    // assign the parity bit
  cmd_highbyte = highByte(command);                                   // split it into bytes
  cmd_lowbyte = lowByte(command);                                     //
  digitalWrite(ssl, LOW);                                             // Drop ssl to enable the AS5048's
  alt_data_highbyte = SPI.transfer(cmd_highbyte);                     // send the initial read command
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);
  azt_data_highbyte = SPI.transfer(cmd_highbyte);                     // send the second read command
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);
  digitalWrite(ssl, HIGH);                                            // disable the AS5048's
}

void loop() {
  // wait for a new client:
  Serial.print(".");
  delay(1000);
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
  float angles[numToAverage];
  float angle;
  float offset;
  float AverageAngle = 0;
  int myCounter = 0;
  command = AS5048_CMD_READ | AS5048_REG_DATA;      // read data register
  command |= calcEvenParity(command)<<15;           // or with the parity of the command
  cmd_highbyte = highByte(command);                 // split it into high and low byte
  cmd_lowbyte = lowByte(command);
  digitalWrite(ssl,LOW);                            // select the chip
  alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
  digitalWrite(ssl,HIGH);                           // but throw those two away as we don't know what the previous command was

  for (myCounter = 0; myCounter <numToAverage; myCounter++ ){    
    digitalWrite(ssl,LOW);                          
    alt_data_highbyte = SPI.transfer(cmd_highbyte); // send the highbyte and lowbyte 
    alt_data_lowbyte = SPI.transfer(cmd_lowbyte);   // and read high and low byte for altitude
    azt_data_highbyte = SPI.transfer(cmd_highbyte); // same for azimuth
    azt_data_lowbyte = SPI.transfer(cmd_lowbyte);
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
    value = data & 0x3FFF;                      // mask off the top two bits
    angles[myCounter] = (float(value)/16383)*360;// calculate the angle that represents
  }
  for (myCounter = 0; myCounter < numToAverage; myCounter++){
    // in the first loop we do not have anything to calculate an offest from
    if (myCounter > 0){
     offset = abs(angles[0]-angles[myCounter]);
     // if the offset is too big that means we are crossing zero
     if (offset > 10){
      offset = abs(angles[0] - (angles[myCounter] - 360));
     }
     AverageAngle = AverageAngle + angles[0] + offset;
    }
   }
  AverageAngle = AverageAngle/numToAverage;
  return AverageAngle;
}

unsigned int Tic(String axis) {
  // take an axis and read that sensor to get the raw encoder value
  unsigned int tics[numToAverage];
  unsigned int offsets[numToAverage];
  unsigned int tic;
  unsigned int offset;
  float averageTic = 0;
  int myCounter = 0;
  command = AS5048_CMD_READ | AS5048_REG_DATA;      // read data register
  command |= calcEvenParity(command)<<15;           // or with the parity of the command
  cmd_highbyte = highByte(command);                 // split it into high and low byte
  cmd_lowbyte = lowByte(command);                   
  digitalWrite(ssl,LOW);                            // select the chip
  alt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command  
  digitalWrite(ssl,HIGH);                           // but throw those two away as we don't know what the previous command was


  for (myCounter = 0; myCounter <numToAverage; myCounter++ ){    
    digitalWrite(ssl,LOW);                          
    alt_data_highbyte = SPI.transfer(cmd_highbyte); // send the highbyte and lowbyte 
    alt_data_lowbyte = SPI.transfer(cmd_lowbyte);   // and read high and low byte for altitude
    azt_data_highbyte = SPI.transfer(cmd_highbyte); // same for azimuth
    azt_data_lowbyte = SPI.transfer(cmd_lowbyte);   
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
  for (myCounter = 0; myCounter <numToAverage; myCounter++){
   if ( myCounter > 0){
    // calculate the offset from the initial reading
    offset = tics[0] - tics[myCounter];
    if (abs(offset) > 1000){
     // recalculate offset but with the reflected value
     if ( offset < 0 ) {
      offset = tics[0] - ( tics[myCounter] - 16383 );
     } else {
      offset = tics[0] - 16383  - tics[myCounter];
     }
     offsets[myCounter] = offset;
     averageOffset = averageOffset + offsets[myCounter];
     averageTic = averageTic + tics[0] + offset;
    }
  }
  averageTic = averageTic/numToAverage;
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

