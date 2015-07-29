/* AS5048A.cpp
 * AS5048A.h Library for interacting with AS5048 magnetic angular position Sensors.
 * Guy R. Thomas July 21, 2015.
 * Released under the MIT licesnce.
 */

#include "Arduino.h"
#include "AS5048A.h"

AS5048A::AS5048A(int cs){
  pinMode(cs, OUTPUT);
  _cs = cs;
}

// Calculate Even parity of word
byte _calEvenParity(unsigned int value) {
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
/*
AS5048 Functions:
*/
// send a two byte command to two daisy chained AS5048As
// we need to be able to read back two unsigned int's
// we use pointers to the unsigned int's that we want to hold the returned data in
void AS5048A::_sendTwoDaisychain(unsigned int cmd,unsigned int &d1, unsigned int &d2){
  int cmd_highbyte;
  int cmd_lowbyte;
  word alt_data_highbyte;
  word alt_data_lowbyte;
  word azt_data_highbyte;
  word azt_data_lowbyte;
  cmd_highbyte = highByte(cmd); // split the command into high and low bytes
  cmd_lowbyte = lowByte(cmd);
  digitalWrite(_cs, LOW); // take the slave select LOW to issue a command
  alt_data_highbyte = SPI.transfer(cmd_highbyte); // send a read command, and store the return value of the previous command in data
  alt_data_lowbyte = SPI.transfer(cmd_lowbyte); // rest of the read command
  azt_data_highbyte = SPI.transfer(cmd_highbyte); // send a read command, and store the return value of the previous command in data
  azt_data_lowbyte = SPI.transfer(cmd_lowbyte); // rest of the read command
  digitalWrite(_cs,HIGH); // close the bus by taking slave select HIGH
  d1 = alt_data_highbyte; // reconstruct our data into unsigned int
  d1 = d1 << 8;
  d1 |= alt_data_lowbyte;
  d2 = azt_data_highbyte;
  d2 = d2 << 8;
  d2 |= azt_data_lowbyte;
}

// send a two byte command to a suingle AS5048As
// we use pointers to the unsigned int that we want to hold the returned data in
void AS5048A::_sendOne(unsigned int cmd, int _cs, unsigned int &d1){
  int cmd_highbyte;
  int cmd_lowbyte;
  unsigned int data_highbyte;
  unsigned int data_lowbyte;
  cmd_highbyte = highByte(cmd); // split the command into high and low bytes
  cmd_lowbyte = lowByte(cmd);
  digitalWrite(_cs, LOW); // take the slave select LOW to issue a command
  data_highbyte = SPI.transfer(cmd_highbyte); // send a read command, and store the return value of the previous command in data
  data_lowbyte = SPI.transfer(cmd_lowbyte); // rest of the read command
  digitalWrite(_cs,HIGH); // close the bus by taking slave select HIGH
  d1 = data_highbyte; // reconstruct our data into unsigned int
  d1 = d1 << 8;
  d1 |= data_lowbyte;
}

// read data from the Sensor
int AS5048A::readData(unsigned int Data[]){
  word command;
  unsigned int rawData;
  unsigned int rawAGC;
  unsigned int rawMag;
  // send the READ_AGC command. The received data is thrown away
  command = AS5048_CMD_READ | AS5048_REG_AGC; // read data register
  command |= _calcEvenParity(command) <<15; // or with the parity of the command
  _sendOne(command, _cs, rawData);
  // send the READ_MAG command. the received data is the AGC data
  command = AS5048_CMD_READ | AS5048_REG_MAG; // read data register
  command |= _calcEvenParity(command) <<15; // or with the parity of the command
  _sendOne(command, _cs, rawData);
  rawAGC = rawData;
  Serial.print("AGC: ");
  Serial.print(rawData,BIN);
  // send the READ ANGLE command. the received data is the magnitude
  command = AS5048_CMD_READ | AS5048_REG_DATA;
  command |= _calcEvenParity(command) <<15;
  _sendOne(command, _cs, rawData);
  rawMag = rawData;
  Serial.print(" MAG: ");
  Serial.print(rawData,BIN);
  // send the NOP command. the received data is the angle
  command = AS5048_CMD_NOP;
  _sendOne(command, _cs, rawData);
  Serial.print(" DAT: ");
  Serial.println(rawData,BIN);
  if ((rawData & 0x4000) || (rawMag & 0x4000) || (rawAGC & 0x4000)) {
    // error flag for one of the axis. need to reset it
    Serial.println("Error reading");
    _sendOne((AS5048_CMD_READ | AS5048_REG_ERR), _cs, rawData);
    return -1;
  } else {
    Data[0] = rawAGC & 0xFF; // Automatic Gain Controll
    Data[1] = rawMag & 0x3FFF; // Magnitude
    Data[2] = rawData & 0x3FFF; // Angle
    Data[3] = rawAGC & 0x400; // High Magnetic Field
    Data[4] = rawAGC & 0x200; // Low Magnetic Field
    return 0;
  }
}
