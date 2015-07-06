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
 
 */

#include <SPI.h>
#include <ESP8266WiFi.h>

// Define this if you want to run as an Access Point.  If undefined it will connect to the 
// SSID with the password below....

#define AP 

char ssid[] = "********"; //  your network SSID (name)
char pass[] = "********";    // your network password (use for WPA, or use as key for WEP)

const char *apssid = "ESPap";
const char *appassword = "gofish";

int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// These defines are for the AS5048 
// for example...
// http://ams.com/eng/Support/Demoboards/Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5048A-Adapterboard  

#define SPI_CMD_READ 0x4000
#define SPI_REG_AGC 0x3FFD
#define SPI_REG_MAG 0x3FFE
#define SPI_REG_DATA 0x3FFF
#define SPI_REG_ERR 0x1

int numToAverage=50;
int sslAzimuth=15;
//int sslAltitude=10;
byte cmd_highbyte = 0;
byte cmd_lowbyte = 0;
byte data_highbyte = 0;
byte data_lowbyte = 0;
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

  pinMode(sslAzimuth, OUTPUT);
//  pinMode(sslAltitude, OUTPUT);
  SPI.begin();                                          // Wake up the buss
  SPI.setBitOrder(MSBFIRST);                            // AS5048 is a Most Significant Bit first
  SPI.setDataMode(SPI_MODE1);                           // AS5048 uses Mode 1
  digitalWrite(sslAzimuth, LOW);                        // Select the Azimuth axis
  data_highbyte=SPI.transfer(SPI_CMD_READ&SPI_REG_ERR); // send the initial read command
  digitalWrite(sslAzimuth, HIGH);                       // disable the chip
  
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
        thisClient.print(PadTic(Tic(sslAzimuth)));
        thisClient.print("\t+00000\r\n");
        Serial.println(Angle(sslAzimuth));
        Serial.println(Tic(sslAzimuth));
        Serial.println(PadTic(Tic(sslAzimuth)));
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

float Angle(int axis) {
  // take an axis and read that sensor to get the angle
  float angles[numToAverage];
  float angle;
  float AverageAngle = 0;
  int myCounter = 0;
  command = 0x8000 | SPI_CMD_READ;              // read command is 15 1's so to make it even parity add a 1 in the 16th bit
  command |= 0x3FFF;                            // this code is not idiomatic.  Should use a parity procedure need to fix it later
  cmd_highbyte = highByte(command);             // split it into high and low byte
  cmd_lowbyte = lowByte(command);
  digitalWrite(axis,LOW);                        // select the chip
  data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  digitalWrite(axis,HIGH);                       // but throw those two away as we don't know what the previous command was

  for (myCounter = 0; myCounter <numToAverage; myCounter++ ){    
    digitalWrite(axis,LOW);
    data_highbyte = SPI.transfer(cmd_highbyte);  
    data_lowbyte = SPI.transfer(cmd_lowbyte);    
    digitalWrite(axis,HIGH);                     // close the chip
    data = data_highbyte;                        // Store the high byte in my 16 bit varriable
    data = data << 8;                            // shift left 8 bits
    data = data | data_lowbyte;                  // tack on the low byte
    value = data & 0x3FFF;                       // mask off the bottom 14 bits
    angles[myCounter] = (float(value)/16383)*360;// calculate the angle that represents
  }
  for (myCounter = 0; myCounter <numToAverage; myCounter++){
    AverageAngle = AverageAngle + angles[myCounter];
  }
  AverageAngle = AverageAngle/numToAverage;
  return AverageAngle;
}

unsigned int Tic(int axis) {
  // take an axis and read that sensor to get the angle
  unsigned int tics[numToAverage];
//  unsigned int tic;
  unsigned int averageTics = 0;
  int myCounter = 0;
  command = 0x8000 | SPI_CMD_READ;              // read command is 15 1's so to make it even parity add a 1 in the 16th bit
  command |= 0x3FFF;                            // this code is not idiomatic.  Should use a parity procedure need to fix it later
  cmd_highbyte = highByte(command);             // split it into high and low byte
  cmd_lowbyte = lowByte(command);               //
  digitalWrite(axis,LOW);                       // select the chip
  data_highbyte = SPI.transfer(cmd_highbyte);   // send a read command, and store the return value of the previous command in data
  data_lowbyte = SPI.transfer(cmd_lowbyte);     // rest of the read command
  digitalWrite(axis,HIGH);                      // but throw those two away as we don't know what the previous command was

  for (myCounter = 0; myCounter <numToAverage; myCounter++ ){    
    digitalWrite(axis,LOW);
    data_highbyte = SPI.transfer(cmd_highbyte);  
    data_lowbyte = SPI.transfer(cmd_lowbyte);   //
    digitalWrite(axis,HIGH);                    // close the chip
    data = data_highbyte;                       // Store the high byte in my 16 bit varriable
    data = data << 8;                           // shift left 8 bits
    data = data | data_lowbyte;                 // tack on the low byte
    value = data & 0x3FFF;                      // mask off the bottom 14 bits
    tics[myCounter] = value;                    // put the value into the array
  }
  for (myCounter = 0; myCounter <numToAverage; myCounter++){
    averageTics = averageTics + tics[myCounter];
  }
  averageTics = averageTics/numToAverage;
  return averageTics;
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

