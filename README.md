# ESP8266-Setting-Circles

This is a project that will use an ESP8266 together with two AS5048A magnetic rotary position sensor ( Perhaps more for drive motor control, focuser control, filter control etc.).  

The ESP8266 is advertised as a Serial to WiFi adapter, but it has EEPROM and a 32Bit processor on it, and there is integration into the Arduino IDE.  https://github.com/esp8266/Arduino

The AS5048A has 14bit resolution (16,384 tics per revolution), and is an absolute position sensor.  The company that makes them have nice demo boards available:

http://ams.com/eng/Support/Demoboards/Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5048A-Adapterboard

The chips supports SPI interface and runs in either 3.3v or 5v.

Or you can just get the chip and design a board yourself, or put it on a breakout board http://www.digikey.com/product-detail/en/1210/1528-1069-ND/5022798.

