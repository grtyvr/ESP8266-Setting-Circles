/*
 * AS5048A.h Library for interacting with AS5048 magnetic angular position Sensors.
 * Guy R. Thomas July 21, 2015.
 * Released under the MIT licesnce.
 */
 
#ifndef AS5048_h
#define AS5048_h
#define LIBRARY_VERION 0.1

#include "Arduino.h"
#include "SPI.h"

#define AS5048_CMD_READ 0x4000  // flag indicating read attement
#define AS5048_CMD_WRITE 0x8000 // flag indicating write attemept
#define AS5048_REG_AGC 0x3FFD   // agc and diagnostics register ( 0 = high, 255 = low)
                                // bit 0 to 7 are agc
                                // bit 8 = 1 Offset Compensation Finished
                                // bit 9 = 0 Cordic OverFlow (output keeps last valid value
                                // bit 10 = 1 low magnetic field
                                // bit 11 = 1 high magnetic field
#define AS5048_REG_MAG 0x3FFE   // magnitude information after ATAN calculation
#define AS5048_REG_DATA 0x3FFF  // angle data
#define AS5048_REG_ERR 0x1      // error register bit 0 = framing error, bit 1 = invalid command, bit 2 = parity error
#define AS5048_CMD_NOP 0x0      // dummy operation
/*
 *
 * We can use the following some time in the future to support daisy chain sensors
 * #define AS5048_NUM_SENSORS 2    // number of sensors in daisy chain
 *
 */

class AS5048A
{
  public:
    AS5048A(int cs);
    int readData(unsigned int Data[]);
  private:
    int _cs;
    void _sendOne(unsigned int cmd, int _cs, unsigned int &d1);
    void _sendTwoDaisychain(unsigned int cmd, unsigned int &d1, unsigned int &d2);
    unsigned int _calcEvenParity(unsigned int data);
};

#endif
